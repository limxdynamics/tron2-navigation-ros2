#!/usr/bin/env python3
# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

"""Forward a ROS 2 Twist stream to the LimX signaling WebSocket safely.

The ``sync`` protocol uses ``request_set_walk_vel_sync`` with acknowledged
x/y/yaw velocities.  Older signaling builds use acknowledged
``request_walk_mode`` plus fire-and-forget ``request_twist`` x/y/z commands.
This bridge deliberately starts paused and uses wall-clock watchdogs, bounded
commands, repeated zero commands, and reconnect interlocks.  It must be the
robot's only velocity command source.
"""

import json
import math
import re
import threading
import time
import uuid
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import websocket
except ImportError as exc:  # Reported again with the Ubuntu package name in __init__.
    websocket = None
    _WEBSOCKET_IMPORT_ERROR = exc
else:
    _WEBSOCKET_IMPORT_ERROR = None


_SYNC_PROTOCOL = "sync"
_LEGACY_PROTOCOL = "legacy"
_SUPPORTED_PROTOCOLS = {_SYNC_PROTOCOL, _LEGACY_PROTOCOL}
_SYNC_ZERO_VELOCITY = {"x": 0.0, "y": 0.0, "yaw": 0.0}
_LEGACY_ZERO_VELOCITY = {"x": 0.0, "y": 0.0, "z": 0.0}
_ROBOT_ACCID_PATTERN = re.compile(r"^WF_[A-Za-z0-9]+_[A-Za-z0-9]+$")


def _valid_robot_accid(value: str) -> bool:
    return len(value) <= 64 and _ROBOT_ACCID_PATTERN.fullmatch(value) is not None


def _response_succeeded(data: Any) -> bool:
    """Match the signaling API convention used by the official LimX client."""
    return isinstance(data, dict) and data.get("result", "success") == "success"


def _walk_mode_ready(data: Any) -> bool:
    if _response_succeeded(data):
        return True
    return (
        isinstance(data, dict)
        and data.get("result") == "fail_state_not_allowed"
        and data.get("current_state") == "Walk"
    )


class LimxWebSocketCmdBridge(Node):
    """Fail-closed ROS 2 velocity gateway for the LimX signaling service."""

    def __init__(self) -> None:
        super().__init__("limx_websocket_cmd_bridge")
        if websocket is None:
            raise RuntimeError(
                "websocket-client is unavailable; install Ubuntu package "
                f"python3-websocket ({_WEBSOCKET_IMPORT_ERROR})"
            )

        self.sdk_url = str(
            self.declare_parameter("sdk_url", "").value
        )
        self.protocol = str(
            self.declare_parameter("protocol", _LEGACY_PROTOCOL).value
        ).strip().lower()
        self.configured_accid = str(
            self.declare_parameter("robot_accid", "").value
        ).strip()
        self.cmd_vel_topic = str(
            self.declare_parameter("cmd_vel_topic", "/sdk_cmd_vel").value
        )
        self.pause_topic = str(
            self.declare_parameter("pause_topic", "/scan_planner/pause").value
        )
        self.send_rate = float(self.declare_parameter("send_rate", 10.0).value)
        self.command_timeout = float(
            self.declare_parameter("command_timeout", 0.3).value
        )
        self.command_threshold = float(
            self.declare_parameter("command_threshold", 1.0e-3).value
        )
        self.zero_hold = float(self.declare_parameter("zero_hold", 0.15).value)
        self.request_timeout = float(
            self.declare_parameter("request_timeout", 0.5).value
        )
        self.mode_request_timeout = float(
            self.declare_parameter("mode_request_timeout", 5.0).value
        )
        self.reconnect_interval = float(
            self.declare_parameter("reconnect_interval", 1.0).value
        )
        self.transition_retry_interval = float(
            self.declare_parameter("transition_retry_interval", 1.0).value
        )
        self.zero_repeat_period = float(
            self.declare_parameter("zero_repeat_period", 0.5).value
        )
        self.zero_repeat_count = int(
            self.declare_parameter("zero_repeat_count", 3).value
        )
        self.zero_repeat_interval = float(
            self.declare_parameter("zero_repeat_interval", 0.05).value
        )
        self.max_linear_speed = float(
            self.declare_parameter("max_linear_speed", 0.60).value
        )
        self.max_lateral_speed = float(
            self.declare_parameter("max_lateral_speed", 0.60).value
        )
        self.max_angular_speed = float(
            self.declare_parameter("max_angular_speed", 0.20).value
        )
        self._paused = bool(
            self.declare_parameter("start_paused", True).value
        )

        self._validate_parameters()

        self._state_lock = threading.Lock()
        self._transport_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._control_stop = threading.Event()
        self._transport_stop = threading.Event()
        self._control_wakeup = threading.Event()
        self._zero_sequence_completed = threading.Event()

        self._last_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_cmd_time: Optional[float] = None
        self._zero_since = time.monotonic()
        self._walk_mode_ready = False
        self._force_zero = True
        self._last_zero_time = 0.0
        self._last_transition_attempt = 0.0

        self._app: Any = None
        self._connected = False
        self._active_accid = self.configured_accid
        self._pending: Dict[str, Tuple[threading.Event, Dict[str, Any], str]] = {}
        self._legacy_request_guids: Dict[str, float] = {}
        self._last_log_time: Dict[str, float] = {}
        self._shutdown_started = False

        self._pause_pub = self.create_publisher(Bool, self.pause_topic, 10)
        self._pause_sub = self.create_subscription(
            Bool, self.pause_topic, self._pause_callback, 10
        )
        self._cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_callback, 20
        )

        self._transport_thread = threading.Thread(
            target=self._connection_loop,
            name="limx-signaling-websocket",
            daemon=True,
        )
        self._control_thread = threading.Thread(
            target=self._control_loop,
            name="limx-websocket-command",
            daemon=True,
        )
        self._transport_thread.start()
        self._control_thread.start()

        self.get_logger().warning(
            "Direct LimX WebSocket command bridge enabled but initially %s: "
            "protocol=%s cmd=%s pause=%s sdk=%s rate=%.1f Hz "
            "limits=(%.3f, %.3f, %.3f). "
            "Do not run another chassis velocity source concurrently."
            % (
                "paused" if self._paused else "active",
                self.protocol,
                self.cmd_vel_topic,
                self.pause_topic,
                self.sdk_url,
                self.send_rate,
                self.max_linear_speed,
                self.max_lateral_speed,
                self.max_angular_speed,
            )
        )
        if self.protocol == _LEGACY_PROTOCOL:
            self.get_logger().warning(
                "Legacy request_twist has no success acknowledgement; the bridge "
                "can prove only that each command was written to the live socket. "
                "Disconnects, explicit rejections, stale ROS commands, and mode "
                "loss still latch the safety pause."
            )

    def _validate_parameters(self) -> None:
        if not self.sdk_url.startswith(("ws://", "wss://")):
            raise ValueError("sdk_url must start with ws:// or wss://")
        if self.protocol not in _SUPPORTED_PROTOCOLS:
            raise ValueError(
                "protocol must be one of: " + ", ".join(sorted(_SUPPORTED_PROTOCOLS))
            )
        if self.protocol == _LEGACY_PROTOCOL and not _valid_robot_accid(
            self.configured_accid
        ):
            raise ValueError(
                "legacy robot_accid must match WF_<MODEL>_<ID> and be at most "
                "64 characters"
            )
        numeric = {
            "send_rate": self.send_rate,
            "command_timeout": self.command_timeout,
            "command_threshold": self.command_threshold,
            "zero_hold": self.zero_hold,
            "request_timeout": self.request_timeout,
            "mode_request_timeout": self.mode_request_timeout,
            "reconnect_interval": self.reconnect_interval,
            "transition_retry_interval": self.transition_retry_interval,
            "zero_repeat_period": self.zero_repeat_period,
            "zero_repeat_interval": self.zero_repeat_interval,
            "max_linear_speed": self.max_linear_speed,
            "max_lateral_speed": self.max_lateral_speed,
            "max_angular_speed": self.max_angular_speed,
        }
        non_finite = [
            name for name, value in numeric.items() if not math.isfinite(value)
        ]
        if non_finite:
            raise ValueError(
                "parameters must be finite: " + ", ".join(non_finite)
            )
        positive = {
            name: value
            for name, value in numeric.items()
            if name not in {
                "command_threshold", "zero_hold", "zero_repeat_interval"
            }
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError("parameters must be positive: " + ", ".join(invalid))
        if self.command_threshold < 0.0 or self.zero_hold < 0.0:
            raise ValueError("command_threshold and zero_hold must be non-negative")
        if self.zero_repeat_count < 1 or self.zero_repeat_interval < 0.0:
            raise ValueError(
                "zero_repeat_count must be >= 1 and zero_repeat_interval non-negative"
            )

    def _pause_callback(self, msg: Bool) -> None:
        now = time.monotonic()
        with self._state_lock:
            changed = self._paused != bool(msg.data)
            self._paused = bool(msg.data)
            if self._paused:
                self._zero_since = now
                self._force_zero = True
                self._zero_sequence_completed.clear()
        self._control_wakeup.set()
        if changed:
            self.get_logger().warning(
                "WebSocket chassis output %s"
                % ("paused" if self._paused else "resumed")
            )

    def _cmd_callback(self, msg: Twist) -> None:
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        now = time.monotonic()
        if not all(math.isfinite(value) for value in values):
            with self._state_lock:
                self._last_cmd = (0.0, 0.0, 0.0)
                self._last_cmd_time = None
                self._zero_since = now
                self._force_zero = True
                self._zero_sequence_completed.clear()
            self._latch_pause("non-finite Twist")
            self._log_throttled(
                "nonfinite",
                1.0,
                "error",
                "Rejected non-finite Twist and forced a chassis stop",
            )
            self._control_wakeup.set()
            return

        unsupported = max(
            abs(msg.linear.z), abs(msg.angular.x), abs(msg.angular.y)
        )
        if unsupported > self.command_threshold:
            self._log_throttled(
                "unsupported_axes",
                2.0,
                "warning",
                "Ignoring unsupported Twist axes linear.z/angular.x/angular.y",
            )

        bounded = (
            self._clamp(msg.linear.x, self.max_linear_speed),
            self._clamp(msg.linear.y, self.max_lateral_speed),
            self._clamp(msg.angular.z, self.max_angular_speed),
        )
        requested = (msg.linear.x, msg.linear.y, msg.angular.z)
        if bounded != requested:
            self._log_throttled(
                "velocity_clamp",
                1.0,
                "warning",
                "Clamped Twist to WebSocket safety limits: "
                "x=%.3f y=%.3f yaw=%.3f" % bounded,
            )

        magnitude = max(abs(value) for value in bounded)
        with self._state_lock:
            self._last_cmd = bounded
            self._last_cmd_time = now
            if magnitude > self.command_threshold:
                self._zero_since = 0.0
            elif self._zero_since == 0.0:
                self._zero_since = now
        self._control_wakeup.set()

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _desired_command(self) -> Tuple[bool, Tuple[float, float, float], str]:
        now = time.monotonic()
        with self._state_lock:
            paused = self._paused
            command = self._last_cmd
            last_cmd_time = self._last_cmd_time
            zero_since = self._zero_since
            walk_ready = self._walk_mode_ready

        if paused:
            return False, (0.0, 0.0, 0.0), "paused"
        if last_cmd_time is None:
            return False, (0.0, 0.0, 0.0), "no command"
        if now - last_cmd_time > self.command_timeout:
            return False, (0.0, 0.0, 0.0), "command timeout"

        magnitude = max(abs(value) for value in command)
        if magnitude > self.command_threshold:
            return True, command, "non-zero command"
        if zero_since > 0.0 and now - zero_since >= self.zero_hold:
            return False, (0.0, 0.0, 0.0), "held zero"
        return walk_ready, (0.0, 0.0, 0.0), "zero hold"

    def _connection_loop(self) -> None:
        while not self._transport_stop.is_set():
            app = websocket.WebSocketApp(
                self.sdk_url,
                on_open=self._on_open,
                on_message=self._on_message,
                on_error=self._on_error,
                on_close=self._on_close,
            )
            with self._transport_lock:
                self._app = app
            try:
                app.run_forever()
            except Exception as exc:  # websocket-client exposes backend errors here.
                self._log_throttled(
                    "run_forever",
                    2.0,
                    "error",
                    f"WebSocket connection loop failed: {exc}",
                )
            finally:
                self._mark_disconnected(app, "connection loop exited")
            if self._transport_stop.wait(self.reconnect_interval):
                break

    def _on_open(self, app: Any) -> None:
        with self._transport_lock:
            if app is not self._app:
                return
            self._connected = True
            self._active_accid = self.configured_accid
        with self._state_lock:
            # Never replay a pre-disconnect command; require a fresh Twist sample.
            self._last_cmd_time = None
            self._walk_mode_ready = False
            self._force_zero = True
            self._zero_sequence_completed.clear()
        self.get_logger().info(f"Connected to LimX signaling service {self.sdk_url}")
        if not self.configured_accid:
            self.get_logger().info("Waiting for signaling server to report robot accid")
        self._control_wakeup.set()

    def _on_message(self, _app: Any, message: str) -> None:
        try:
            root = json.loads(message)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(root, dict):
            return

        reported_accid = root.get("accid")
        if isinstance(reported_accid, str) and reported_accid.strip():
            reported_accid = reported_accid.strip()
            changed = False
            conflicting_accid = ""
            with self._transport_lock:
                if not self._active_accid:
                    self._active_accid = reported_accid
                    changed = True
                elif self._active_accid != reported_accid:
                    conflicting_accid = self._active_accid
            if conflicting_accid:
                self._latch_pause("conflicting robot accid from signaling")
                self._log_throttled(
                    "accid_conflict",
                    2.0,
                    "error",
                    "Ignored signaling message for robot accid %r; this bridge "
                    "is pinned to %r" % (reported_accid, conflicting_accid),
                )
                return
            if changed:
                self.get_logger().info(
                    f"Using robot accid reported by signaling: {reported_accid}"
                )
                self._control_wakeup.set()

        title = root.get("title", "")
        if not isinstance(title, str) or not title.startswith("response_"):
            return
        guid = root.get("guid", "")

        if self.protocol == _LEGACY_PROTOCOL and title == "response_twist":
            if isinstance(guid, str) and guid:
                with self._pending_lock:
                    self._legacy_request_guids.pop(guid, None)
            # Some legacy builds omit or replace guid.  Because the bridge is
            # pinned to one accid and is the sole command source, any explicit
            # request_twist rejection for that robot must fail closed.
            data = root.get("data", {})
            if not _response_succeeded(data):
                self._handle_command_failure(data)
            return

        pending = None
        with self._pending_lock:
            if isinstance(guid, str) and guid:
                pending = self._pending.get(guid)
            if pending is None and len(self._pending) == 1:
                # Older signaling builds may omit/replace guid. Only one request is
                # issued by the control worker at a time. Enforce that invariant
                # before accepting the compatibility title match.
                candidate = next(iter(self._pending.values()))
                if candidate[2] == title:
                    pending = candidate
        if pending is not None:
            event, holder, _expected_title = pending
            holder["data"] = root.get("data", {})
            event.set()

    def _on_error(self, app: Any, error: Any) -> None:
        if not self._transport_stop.is_set():
            self._log_throttled(
                "websocket_error",
                2.0,
                "error",
                f"LimX WebSocket error: {error}",
            )
        self._mark_disconnected(app, str(error))

    def _on_close(self, app: Any, *_args: Any) -> None:
        self._mark_disconnected(app, "connection closed")

    def _mark_disconnected(self, app: Any, reason: str) -> None:
        changed = False
        with self._transport_lock:
            if app is not self._app:
                return
            if self._connected:
                self._connected = False
                changed = True
        with self._state_lock:
            self._walk_mode_ready = False
            self._force_zero = True
            self._zero_sequence_completed.clear()
        if changed and not self._transport_stop.is_set():
            self._latch_pause("WebSocket connection lost")
        self._fail_pending(reason)
        self._control_wakeup.set()
        if changed and not self._transport_stop.is_set():
            self.get_logger().warning(
                "Disconnected from LimX signaling; motion is inhibited until "
                "reconnect, zeroing, a fresh Twist, and explicit resume"
            )

    def _fail_pending(self, reason: str) -> None:
        with self._pending_lock:
            pending_items = list(self._pending.values())
        for event, holder, _expected_title in pending_items:
            holder.setdefault(
                "data", {"result": "fail_websocket_error", "message": reason}
            )
            event.set()

    def _request(self, title: str, data: Dict[str, Any], timeout: float) -> Any:
        with self._transport_lock:
            app = self._app
            connected = self._connected
            accid = self._active_accid
        if not connected or app is None or not accid:
            return {"result": "fail_not_connected"}

        guid = str(uuid.uuid4())
        event = threading.Event()
        holder: Dict[str, Any] = {}
        response_title = title.replace("request_", "response_", 1)
        with self._pending_lock:
            self._pending[guid] = (event, holder, response_title)

        payload = {
            "accid": accid,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": guid,
            "data": data,
        }
        try:
            with self._send_lock:
                app.send(json.dumps(payload, ensure_ascii=False))
        except Exception as exc:
            with self._pending_lock:
                self._pending.pop(guid, None)
            self._log_throttled(
                "send_error", 1.0, "error", f"WebSocket send failed: {exc}"
            )
            try:
                app.close()
            except Exception:
                pass
            return {"result": "fail_websocket_error", "message": str(exc)}

        if not event.wait(timeout):
            with self._pending_lock:
                self._pending.pop(guid, None)
            self._log_throttled(
                "request_timeout",
                1.0,
                "error",
                f"LimX request timed out: {title}",
            )
            try:
                app.close()
            except Exception:
                pass
            return {"result": "fail_timeout"}

        with self._pending_lock:
            self._pending.pop(guid, None)
        return holder.get("data", {"result": "fail_empty_response"})

    def _send_legacy_request(self, title: str, data: Dict[str, Any]) -> bool:
        """Write one legacy request without treating the missing ACK as success."""
        with self._transport_lock:
            app = self._app
            connected = self._connected
            accid = self._active_accid
        if not connected or app is None or not accid:
            return False

        guid = str(uuid.uuid4())
        payload = {
            "accid": accid,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": guid,
            "data": data,
        }
        now = time.monotonic()
        with self._pending_lock:
            cutoff = now - max(5.0, self.request_timeout * 4.0)
            self._legacy_request_guids = {
                request_guid: sent_at
                for request_guid, sent_at in self._legacy_request_guids.items()
                if sent_at >= cutoff
            }
            self._legacy_request_guids[guid] = now
        try:
            with self._send_lock:
                app.send(json.dumps(payload, ensure_ascii=False))
        except Exception as exc:
            with self._pending_lock:
                self._legacy_request_guids.pop(guid, None)
            self._log_throttled(
                "send_error", 1.0, "error", f"WebSocket send failed: {exc}"
            )
            self._mark_disconnected(app, str(exc))
            try:
                app.close()
            except Exception:
                pass
            return False
        return True

    def _send_velocity(self, command: Tuple[float, float, float]) -> bool:
        if self.protocol == _LEGACY_PROTOCOL:
            sent = self._send_legacy_request(
                "request_twist",
                {
                    "x": round(command[0], 4),
                    "y": round(command[1], 4),
                    "z": round(command[2], 4),
                },
            )
            if not sent:
                self._handle_command_failure(
                    {"result": "fail_websocket_send"}
                )
            return sent

        result = self._request(
            "request_set_walk_vel_sync",
            {
                "x": round(command[0], 4),
                "y": round(command[1], 4),
                "yaw": round(command[2], 4),
            },
            self.request_timeout,
        )
        if not _response_succeeded(result):
            self._handle_command_failure(result)
            return False
        return True

    def _control_loop(self) -> None:
        period = 1.0 / self.send_rate
        while not self._control_stop.is_set():
            cycle_start = time.monotonic()
            desired, command, reason = self._desired_command()
            connected, have_accid = self._transport_ready()

            if reason == "command timeout":
                self._latch_pause("ROS 2 velocity command timeout")
                desired = False
                command = (0.0, 0.0, 0.0)

            if connected and have_accid:
                if desired:
                    if not self._is_walk_mode_ready():
                        self._try_enter_walk_mode()
                    desired, command, _reason = self._desired_command()
                    if desired and self._is_walk_mode_ready():
                        if self._send_velocity(command):
                            if max(abs(value) for value in command) > self.command_threshold:
                                with self._state_lock:
                                    self._force_zero = False
                                self._zero_sequence_completed.clear()
                            else:
                                self._record_zero_completion()
                else:
                    with self._state_lock:
                        zero_due = (
                            self._force_zero
                            or cycle_start - self._last_zero_time
                            >= self.zero_repeat_period
                        )
                    if zero_due:
                        repeat = self.zero_repeat_count if self._force_zero else 1
                        if self._send_zero_sequence(repeat):
                            self._record_zero_completion()
                    with self._state_lock:
                        self._walk_mode_ready = False

            elapsed = time.monotonic() - cycle_start
            self._control_wakeup.wait(max(0.0, period - elapsed))
            self._control_wakeup.clear()

    def _transport_ready(self) -> Tuple[bool, bool]:
        with self._transport_lock:
            return self._connected, bool(self._active_accid)

    def _is_walk_mode_ready(self) -> bool:
        with self._state_lock:
            return self._walk_mode_ready

    def _try_enter_walk_mode(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            if now - self._last_transition_attempt < self.transition_retry_interval:
                return
            self._last_transition_attempt = now

        request_title = (
            "request_walk_mode"
            if self.protocol == _LEGACY_PROTOCOL
            else "request_set_walk_mode"
        )
        mode = self._request(request_title, {}, self.mode_request_timeout)
        if not _walk_mode_ready(mode):
            self._latch_pause("robot did not enter Walk mode")
            self._log_throttled(
                "walk_mode",
                1.0,
                "error",
                f"Robot did not enter Walk mode: {mode}",
            )
            with self._state_lock:
                self._walk_mode_ready = False
                self._force_zero = True
            return

        # A mode transition must never replay a previous non-zero setpoint.
        if not self._send_zero_sequence(self.zero_repeat_count):
            with self._state_lock:
                self._walk_mode_ready = False
                self._force_zero = True
            return
        self._record_zero_completion()
        with self._state_lock:
            self._walk_mode_ready = True
        self.get_logger().warning(
            "Robot Walk mode ready; forwarding fresh bounded Twist commands"
        )

    def _send_zero_sequence(self, count: int) -> bool:
        success = True
        for index in range(max(1, count)):
            if self.protocol == _LEGACY_PROTOCOL:
                sent = self._send_legacy_request(
                    "request_twist", dict(_LEGACY_ZERO_VELOCITY)
                )
                result: Any = {"result": "attempted" if sent else "fail_send"}
                command_succeeded = sent
            else:
                result = self._request(
                    "request_set_walk_vel_sync",
                    dict(_SYNC_ZERO_VELOCITY),
                    self.request_timeout,
                )
                command_succeeded = _response_succeeded(result)
            if not command_succeeded:
                success = False
                self._latch_pause("chassis zero command could not be sent safely")
                self._log_throttled(
                    "zero_failure",
                    1.0,
                    "error",
                    f"Chassis zero command failed: {result}",
                )
                break
            if index + 1 < count and self.zero_repeat_interval > 0.0:
                time.sleep(self.zero_repeat_interval)
        return success

    def _record_zero_completion(self) -> None:
        with self._state_lock:
            self._force_zero = False
            self._last_zero_time = time.monotonic()
        self._zero_sequence_completed.set()

    def _handle_command_failure(self, result: Any) -> None:
        with self._state_lock:
            self._walk_mode_ready = False
            self._force_zero = True
            self._zero_sequence_completed.clear()
        self._latch_pause("LimX velocity request failed")
        self._log_throttled(
            "velocity_failure",
            1.0,
            "error",
            f"Velocity command failed or was explicitly rejected; forcing stop: {result}",
        )

    def _latch_pause(self, reason: str) -> None:
        with self._state_lock:
            changed = not self._paused
            self._paused = True
            self._zero_since = time.monotonic()
            self._force_zero = True
            self._zero_sequence_completed.clear()
        if changed:
            if rclpy.ok():
                try:
                    msg = Bool()
                    msg.data = True
                    self._pause_pub.publish(msg)
                    self.get_logger().error(
                        f"Safety pause latched ({reason}); inspect the fault and publish "
                        f"{self.pause_topic}=false explicitly before motion can resume"
                    )
                except Exception:
                    # SIGINT may invalidate the context between ok() and publish().
                    pass
        self._control_wakeup.set()

    def _log_throttled(
        self, key: str, interval: float, level: str, message: str
    ) -> None:
        if not rclpy.ok():
            return
        now = time.monotonic()
        with self._state_lock:
            last = self._last_log_time.get(key, 0.0)
            if now - last < interval:
                return
            self._last_log_time[key] = now
        logger = self.get_logger()
        getattr(logger, level)(message)

    def shutdown_bridge(self) -> None:
        if self._shutdown_started:
            return
        self._shutdown_started = True

        with self._state_lock:
            self._paused = True
            self._force_zero = True
            self._zero_sequence_completed.clear()
        pause_msg = Bool()
        pause_msg.data = True
        if rclpy.ok():
            try:
                self._pause_pub.publish(pause_msg)
            except Exception:
                # SIGINT may invalidate the context between ok() and publish().
                pass
        self._control_wakeup.set()

        # Let the normal control worker issue the first stop immediately.
        connected, have_accid = self._transport_ready()
        if connected and have_accid:
            self._zero_sequence_completed.wait(
                self.mode_request_timeout
                + self.request_timeout * self.zero_repeat_count
                + 0.5
            )
        self._control_stop.set()
        self._control_wakeup.set()
        self._control_thread.join(timeout=self.request_timeout + 1.0)

        # Repeat once after the worker has stopped so process teardown cannot leave
        # a stale non-zero setpoint in the signaling service.
        connected, have_accid = self._transport_ready()
        if not self._control_thread.is_alive() and connected and have_accid:
            self._send_zero_sequence(self.zero_repeat_count)

        self._transport_stop.set()
        with self._transport_lock:
            app = self._app
        if app is not None:
            try:
                app.close()
            except Exception:
                pass
        self._fail_pending("bridge shutdown")
        self._transport_thread.join(timeout=self.reconnect_interval + 1.0)
        if rclpy.ok():
            self.get_logger().info("LimX WebSocket bridge stopped after zeroing")


def main(args: Optional[Any] = None) -> None:
    rclpy.init(args=args)
    node: Optional[LimxWebSocketCmdBridge] = None
    try:
        node = LimxWebSocketCmdBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown_bridge()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
