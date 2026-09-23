# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

"""Unit tests for sync/legacy LimX WebSocket command dispatch."""

import importlib.util
import json
from pathlib import Path
import sys
import threading
import types

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "limx_websocket_cmd_bridge.py"
)


def _load_bridge_module():
    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: False
    rclpy.init = lambda **_kwargs: None
    rclpy.spin = lambda _node: None
    rclpy.shutdown = lambda: None

    executors = types.ModuleType("rclpy.executors")

    class ExternalShutdownException(Exception):
        pass

    executors.ExternalShutdownException = ExternalShutdownException

    node_module = types.ModuleType("rclpy.node")

    class Node:
        pass

    node_module.Node = Node

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class Twist:
        pass

    geometry_msgs_msg.Twist = Twist
    geometry_msgs.msg = geometry_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class Bool:
        def __init__(self):
            self.data = False

    std_msgs_msg.Bool = Bool
    std_msgs.msg = std_msgs_msg

    websocket = types.ModuleType("websocket")
    websocket.WebSocketApp = object

    stubs = {
        "rclpy": rclpy,
        "rclpy.executors": executors,
        "rclpy.node": node_module,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "websocket": websocket,
    }
    previous = {name: sys.modules.get(name) for name in stubs}
    sys.modules.update(stubs)
    try:
        spec = importlib.util.spec_from_file_location(
            "limx_websocket_cmd_bridge_under_test", SCRIPT
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        for name, old_module in previous.items():
            if old_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old_module


BRIDGE_MODULE = _load_bridge_module()
Bridge = BRIDGE_MODULE.LimxWebSocketCmdBridge


class FakeLogger:
    def __init__(self):
        self.messages = []

    def __getattr__(self, level):
        return lambda message: self.messages.append((level, message))


class FakeApp:
    def __init__(self, error=None):
        self.error = error
        self.messages = []
        self.close_count = 0

    def send(self, message):
        if self.error is not None:
            raise self.error
        self.messages.append(json.loads(message))

    def close(self):
        self.close_count += 1


def make_transport_bridge(protocol="legacy", app=None):
    bridge = object.__new__(Bridge)
    bridge.protocol = protocol
    bridge.request_timeout = 0.2
    bridge._transport_lock = threading.Lock()
    bridge._pending_lock = threading.Lock()
    bridge._send_lock = threading.Lock()
    bridge._app = app if app is not None else FakeApp()
    bridge._connected = True
    bridge._active_accid = "WF_TESTMODEL_001"
    bridge._pending = {}
    bridge._legacy_request_guids = {}
    bridge._log_throttled = lambda *_args, **_kwargs: None
    bridge._mark_disconnected = lambda *_args, **_kwargs: None
    return bridge


def test_legacy_protocol_requires_explicit_robot_accid():
    bridge = object.__new__(Bridge)
    bridge.sdk_url = "ws://192.0.2.2:5000"
    bridge.protocol = "legacy"
    bridge.configured_accid = ""

    with pytest.raises(ValueError, match="robot_accid must match"):
        bridge._validate_parameters()


@pytest.mark.parametrize(
    ("accid", "expected"),
    [
        ("WF_TESTMODEL_001", True),
        ("WF_TESTMODEL_002", True),
        ("WF_TESTMODEL_123456", True),
        ("", False),
        ("TRON2A_333", False),
        ("WF_TESTMODEL", False),
        ("WF_TESTMODEL_bad-id", False),
        ("WF_TESTMODEL_001;reboot", False),
        ("WF_" + "A" * 60 + "_1", False),
    ],
)
def test_robot_accid_format_validation(accid, expected):
    assert BRIDGE_MODULE._valid_robot_accid(accid) is expected


def test_legacy_twist_is_fire_and_forget_without_pending_request():
    app = FakeApp()
    bridge = make_transport_bridge(app=app)

    assert bridge._send_velocity((0.08, -0.02, 0.1)) is True

    assert len(app.messages) == 1
    request = app.messages[0]
    assert request["accid"] == "WF_TESTMODEL_001"
    assert request["title"] == "request_twist"
    assert request["data"] == {"x": 0.08, "y": -0.02, "z": 0.1}
    assert bridge._pending == {}
    assert request["guid"] in bridge._legacy_request_guids


def test_sync_velocity_keeps_acknowledged_request_path():
    bridge = make_transport_bridge(protocol="sync")
    requests = []
    bridge._request = lambda title, data, timeout: (
        requests.append((title, data, timeout)) or {"result": "success"}
    )

    assert bridge._send_velocity((0.08, -0.02, 0.1)) is True
    assert requests == [
        (
            "request_set_walk_vel_sync",
            {"x": 0.08, "y": -0.02, "yaw": 0.1},
            bridge.request_timeout,
        )
    ]


@pytest.mark.parametrize(
    ("protocol", "expected_title"),
    [
        ("sync", "request_set_walk_mode"),
        ("legacy", "request_walk_mode"),
    ],
)
def test_walk_mode_request_is_dispatched_by_protocol(protocol, expected_title):
    bridge = make_transport_bridge(protocol=protocol)
    bridge.mode_request_timeout = 1.0
    bridge.transition_retry_interval = 0.0
    bridge.zero_repeat_count = 3
    bridge._state_lock = threading.Lock()
    bridge._last_transition_attempt = 0.0
    bridge._walk_mode_ready = False
    bridge._force_zero = True
    bridge._request = lambda title, data, timeout: (
        setattr(bridge, "mode_request", (title, data, timeout))
        or {"result": "success"}
    )
    bridge._send_zero_sequence = lambda count: (
        setattr(bridge, "zero_count", count) or True
    )
    bridge._record_zero_completion = lambda: setattr(
        bridge, "zero_completed", True
    )
    logger = FakeLogger()
    bridge.get_logger = lambda: logger

    bridge._try_enter_walk_mode()

    assert bridge.mode_request == (
        expected_title,
        {},
        bridge.mode_request_timeout,
    )
    assert bridge.zero_count == bridge.zero_repeat_count
    assert bridge.zero_completed is True
    assert bridge._walk_mode_ready is True


def test_legacy_zero_sequence_repeats_fire_and_forget_twists():
    bridge = make_transport_bridge()
    sent = []
    bridge.zero_repeat_interval = 0.0
    bridge._send_legacy_request = lambda title, data: (
        sent.append((title, data)) or True
    )
    bridge._latch_pause = lambda reason: pytest.fail(reason)

    assert bridge._send_zero_sequence(3) is True
    assert sent == [
        ("request_twist", {"x": 0.0, "y": 0.0, "z": 0.0}),
        ("request_twist", {"x": 0.0, "y": 0.0, "z": 0.0}),
        ("request_twist", {"x": 0.0, "y": 0.0, "z": 0.0}),
    ]


@pytest.mark.parametrize("guid", ["known-guid", ""])
def test_legacy_async_twist_rejection_latches_failure_even_without_guid(guid):
    bridge = make_transport_bridge()
    bridge._legacy_request_guids = {"known-guid": 1.0}
    failures = []
    bridge._handle_command_failure = failures.append

    bridge._on_message(
        None,
        json.dumps(
            {
                "accid": "WF_TESTMODEL_001",
                "title": "response_twist",
                "guid": guid,
                "data": {"result": "fail_invalid_cmd"},
            }
        ),
    )

    assert failures == [{"result": "fail_invalid_cmd"}]


def test_legacy_send_exception_marks_disconnect_and_fails_closed():
    app = FakeApp(RuntimeError("socket closed"))
    bridge = make_transport_bridge(app=app)
    disconnected = []
    failures = []
    bridge._mark_disconnected = lambda failed_app, reason: disconnected.append(
        (failed_app, reason)
    )
    bridge._handle_command_failure = failures.append

    assert bridge._send_velocity((0.01, 0.0, 0.0)) is False
    assert disconnected == [(app, "socket closed")]
    assert failures == [{"result": "fail_websocket_send"}]
    assert app.close_count == 1


class FakeThread:
    def __init__(self):
        self.join_timeouts = []

    def join(self, timeout=None):
        self.join_timeouts.append(timeout)

    def is_alive(self):
        return False


def test_shutdown_repeats_final_zero_after_control_worker_stops():
    bridge = make_transport_bridge()
    bridge._shutdown_started = False
    bridge._state_lock = threading.Lock()
    bridge._paused = False
    bridge._force_zero = False
    bridge._zero_sequence_completed = threading.Event()
    bridge._control_wakeup = threading.Event()
    bridge._control_stop = threading.Event()
    bridge._transport_stop = threading.Event()
    bridge._pause_pub = types.SimpleNamespace(publish=lambda _msg: None)
    bridge.mode_request_timeout = 0.1
    bridge.zero_repeat_count = 3
    bridge.reconnect_interval = 0.1
    bridge._control_thread = FakeThread()
    bridge._transport_thread = FakeThread()
    bridge._transport_ready = lambda: (True, True)
    bridge._zero_sequence_completed.wait = lambda _timeout: True
    zero_counts = []
    bridge._send_zero_sequence = lambda count: zero_counts.append(count) or True
    bridge._fail_pending = lambda _reason: None
    logger = FakeLogger()
    bridge.get_logger = lambda: logger

    bridge.shutdown_bridge()

    assert zero_counts == [3]
    assert bridge._paused is True
    assert bridge._control_stop.is_set()
    assert bridge._transport_stop.is_set()
    assert bridge._app.close_count == 1
