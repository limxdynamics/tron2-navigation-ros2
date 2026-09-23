# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

"""Regression checks for the single legacy WebSocket chassis route."""

import re
from pathlib import Path
import os
import shutil
import subprocess


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[3]

RUN_NX = WORKSPACE_ROOT / "run_nx_navigation_humble.sh"
RUN_FAST_LIO = WORKSPACE_ROOT / "run_fast_lio_navigation_humble.sh"
NAVI = WORKSPACE_ROOT / "navi.sh"
STOP_NX = WORKSPACE_ROOT / "stop_nx_navigation_humble.sh"
LAUNCH = PACKAGE_ROOT / "launch" / "nx_pct_navigation.launch.py"
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"
BRIDGE = PACKAGE_ROOT / "scripts" / "limx_websocket_cmd_bridge.py"
GRID_MAP_HEADER = PACKAGE_ROOT.parent / "plan_env" / "include" / "plan_env" / "grid_map.h"
GRID_MAP_SOURCE = PACKAGE_ROOT.parent / "plan_env" / "src" / "grid_map.cpp"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _environment_without_vendor_processes(tmp_path: Path):
    mock_bin = tmp_path / "mock-bin"
    mock_bin.mkdir()
    for command in ("systemctl", "pgrep"):
        executable = mock_bin / command
        executable.write_text("#!/usr/bin/env bash\nexit 1\n", encoding="utf-8")
        executable.chmod(0o755)

    env = os.environ.copy()
    env["PATH"] = f"{mock_bin}:{env['PATH']}"
    return env


def test_mros_command_route_is_not_built_or_launched():
    for removed in (
        PACKAGE_ROOT / "src" / "mros_ros2_bridge.cpp",
        PACKAGE_ROOT / "scripts" / "limx_websocket_mode_probe.py",
        PACKAGE_ROOT / "test" / "test_limx_websocket_mode_probe.py",
    ):
        assert not removed.exists()

    for source in (CMAKE, LAUNCH):
        text = _read(source)
        assert "mros_ros2_bridge" not in text
        assert "enable_mros_commands" not in text
        assert "start_mros_bridge" not in text


def test_runtime_launchers_expose_only_off_or_legacy_websocket():
    run_nx = _read(RUN_NX)
    run_fast_lio = _read(RUN_FAST_LIO)
    navi = _read(NAVI)

    for text in (run_nx, run_fast_lio, navi):
        for removed_setting in (
            "MROS_ROOT",
            "START_MROS_BRIDGE",
            "ENABLE_MROS_COMMANDS",
            "MROS_MAX_LINEAR_SPEED",
            "MROS_MAX_ANGULAR_SPEED",
            "WEBSOCKET_LEGACY_REQUIRED_ACCID",
        ):
            assert removed_setting not in text

    assert "off|websocket" in run_nx
    assert "off, websocket" in run_nx
    assert "off|websocket|mros|auto" not in run_nx
    assert 'WEBSOCKET_PROTOCOL="${WEBSOCKET_PROTOCOL:-legacy}"' in run_nx
    assert 'WEBSOCKET_PROTOCOL="${WEBSOCKET_PROTOCOL:-legacy}"' in run_fast_lio
    assert "Only WEBSOCKET_PROTOCOL=legacy" in run_nx
    for launcher in (run_nx, run_fast_lio):
        assert 'CHASSIS_OUTPUT_MODE="${CHASSIS_OUTPUT_MODE:-off}"' in launcher
        assert (
            'ENABLE_WEBSOCKET_COMMANDS="${ENABLE_WEBSOCKET_COMMANDS:-'
            not in launcher
        )

    assert "NAVI_CHASSIS_OUTPUT_MODE" not in navi
    assert "CHASSIS_OUTPUT_MODE=websocket" in navi
    assert "WEBSOCKET_PROTOCOL=legacy" in navi
    assert "grep -qx '/limx_websocket_cmd_bridge'" in navi


def test_all_chassis_layers_default_to_point_six_metres_per_second():
    run_nx = _read(RUN_NX)
    run_fast_lio = _read(RUN_FAST_LIO)
    navi = _read(NAVI)
    launch = _read(LAUNCH)
    bridge = _read(BRIDGE)

    for launcher in (run_nx, run_fast_lio):
        assert (
            'WEBSOCKET_MAX_LINEAR_SPEED="${WEBSOCKET_MAX_LINEAR_SPEED:-0.60}"'
            in launcher
        )
        assert (
            'WEBSOCKET_MAX_LATERAL_SPEED="${WEBSOCKET_MAX_LATERAL_SPEED:-0.60}"'
            in launcher
        )
        assert (
            'WEBSOCKET_MAX_ANGULAR_SPEED="${WEBSOCKET_MAX_ANGULAR_SPEED:-0.20}"'
            in launcher
        )

    assert 'local speed="${NAV_SPEED:-0.60}"' in navi
    assert re.search(
        r"'websocket_max_linear_speed', default_value='0\.60'", launch
    )
    assert re.search(
        r"'websocket_max_lateral_speed', default_value='0\.60'", launch
    )
    assert 'self.declare_parameter("max_linear_speed", 0.60)' in bridge
    assert 'self.declare_parameter("max_lateral_speed", 0.60)' in bridge


def test_obstacle_cloud_bypass_is_explicit_and_fail_closed():
    navi = _read(NAVI)
    run_nx = _read(RUN_NX)
    run_fast_lio = _read(RUN_FAST_LIO)
    launch = _read(LAUNCH)
    grid_map_header = _read(GRID_MAP_HEADER)
    grid_map_source = _read(GRID_MAP_SOURCE)

    assert "--no-obstacle-avoidance" in navi
    assert "--stair-centerline" in navi
    assert "NO_OBSTACLE_GO" in navi
    assert "grid_map.enable_cloud_subscription" in navi
    assert "ENABLE_LOCAL_OBSTACLE_AVOIDANCE" in run_fast_lio
    assert "ENABLE_LOCAL_OBSTACLE_AVOIDANCE" in run_nx
    assert "ENABLE_PCT_STAIR_CENTERLINE" in run_fast_lio
    assert "ENABLE_PCT_STAIR_CENTERLINE" in run_nx
    assert "stair_centerline.enabled" in run_nx
    assert "pct_planner:stair_centerline.enabled" in run_nx
    assert "START_PAUSED=true" in run_nx
    assert "0.20" in run_nx
    assert "'enable_cloud_subscription', default_value='true'" in launch
    assert "_NO_OBSTACLE_MAX_LINEAR_SPEED = 0.60" in launch
    assert "grid_map.enable_cloud_subscription" in launch
    assert "enable_cloud_subscription_" in grid_map_header
    assert "if (mp_.enable_cloud_subscription_)" in grid_map_source


def test_navi_requires_explicit_bounded_speed_for_obstacle_bypass(tmp_path):
    shutil.copy2(NAVI, tmp_path / "navi.sh")
    for name, content in {
        "stop_nx_navigation_humble.sh": "#!/usr/bin/env bash\nexit 0\n",
        "run_fast_lio_navigation_humble.sh": (
            "#!/usr/bin/env bash\n"
            "printf 'AVOIDANCE=%s SPEED=%s\\n' "
            '"$ENABLE_LOCAL_OBSTACLE_AVOIDANCE" '
            '"$CONTROLLER_MAX_LINEAR_SPEED"\n'
        ),
        "select_navigation_map.py": "# test stub\n",
    }.items():
        path = tmp_path / name
        path.write_text(content, encoding="utf-8")
        path.chmod(0o755)

    env = _environment_without_vendor_processes(tmp_path)
    env["PYTHON_EXECUTABLE"] = "/bin/true"

    missing_speed = subprocess.run(
        [
            "bash", str(tmp_path / "navi.sh"), "nav",
            "--no-obstacle-avoidance",
        ],
        text=True,
        capture_output=True,
        check=False,
        env=env,
    )
    assert missing_speed.returncode == 2
    assert "必须显式指定 --speed" in missing_speed.stderr

    excessive_speed = subprocess.run(
        [
            "bash", str(tmp_path / "navi.sh"), "nav",
            "--no-obstacle-avoidance", "--speed", "0.61",
        ],
        text=True,
        capture_output=True,
        check=False,
        env=env,
    )
    assert excessive_speed.returncode == 2
    assert "不超过 0.60" in excessive_speed.stderr

    accepted = subprocess.run(
        [
            "bash", str(tmp_path / "navi.sh"), "nav",
            "--no-obstacle-avoidance", "--speed", "0.60",
            "--robot-accid", "WF_TESTMODEL_002",
        ],
        text=True,
        capture_output=True,
        check=False,
        env=env,
    )
    assert accepted.returncode == 0, accepted.stderr
    assert "LOCAL_OBSTACLE_AVOIDANCE=false" in accepted.stdout
    assert "AVOIDANCE=false SPEED=0.60" in accepted.stdout


def test_stair_centerline_requires_obstacle_bypass_and_is_forwarded(tmp_path):
    shutil.copy2(NAVI, tmp_path / "navi.sh")
    for name, content in {
        "stop_nx_navigation_humble.sh": "#!/usr/bin/env bash\nexit 0\n",
        "run_fast_lio_navigation_humble.sh": (
            "#!/usr/bin/env bash\n"
            "printf 'AVOIDANCE=%s CENTERLINE=%s SPEED=%s\\n' "
            '"$ENABLE_LOCAL_OBSTACLE_AVOIDANCE" '
            '"$ENABLE_PCT_STAIR_CENTERLINE" '
            '"$CONTROLLER_MAX_LINEAR_SPEED"\n'
        ),
        "select_navigation_map.py": "# test stub\n",
    }.items():
        path = tmp_path / name
        path.write_text(content, encoding="utf-8")
        path.chmod(0o755)

    env = _environment_without_vendor_processes(tmp_path)
    env["PYTHON_EXECUTABLE"] = "/bin/true"

    missing_bypass = subprocess.run(
        [
            "bash", str(tmp_path / "navi.sh"), "nav",
            "--stair-centerline", "--speed", "0.60",
            "--robot-accid", "WF_TESTMODEL_002",
        ],
        text=True,
        capture_output=True,
        check=False,
        env=env,
    )
    assert missing_bypass.returncode == 2
    assert "必须与 --no-obstacle-avoidance 同时使用" in missing_bypass.stderr

    accepted = subprocess.run(
        [
            "bash", str(tmp_path / "navi.sh"), "nav",
            "--no-obstacle-avoidance", "--stair-centerline",
            "--speed", "0.60",
            "--robot-accid", "WF_TESTMODEL_002",
        ],
        text=True,
        capture_output=True,
        check=False,
        env=env,
    )
    assert accepted.returncode == 0, accepted.stderr
    assert "PCT_STAIR_CENTERLINE=true" in accepted.stdout
    assert "AVOIDANCE=false CENTERLINE=true SPEED=0.60" in accepted.stdout


def test_robot_identity_requires_explicit_site_configuration():
    navi = _read(NAVI)
    assert "请输入底盘身份" in navi
    assert "--robot-accid|--accid" in navi
    assert 'default_websocket_robot_accid="${WEBSOCKET_ROBOT_ACCID:-}"' in navi
    assert 'WEBSOCKET_ROBOT_ACCID="${websocket_robot_accid}"' in navi

    for source in (RUN_NX, RUN_FAST_LIO):
        text = _read(source)
        assert "is_valid_robot_accid" in text
        assert 'WEBSOCKET_URL="${WEBSOCKET_URL:-}"' in text
        assert 'WEBSOCKET_ROBOT_ACCID="${WEBSOCKET_ROBOT_ACCID:-}"' in text
        assert "config/navigation.env" in text

    launch = _read(LAUNCH)
    bridge = _read(BRIDGE)
    assert "'websocket_url', default_value=''" in launch
    assert "'websocket_robot_accid', default_value=''" in launch
    assert 'self.declare_parameter("sdk_url", "")' in bridge
    assert 'self.declare_parameter("robot_accid", "")' in bridge
    assert "_valid_robot_accid(websocket_robot_accid)" in _read(LAUNCH)
    assert "_valid_robot_accid(" in _read(BRIDGE)


def test_vendor_mros_service_is_never_killed_as_a_navigation_child():
    navi = _read(NAVI)
    stop_nx = _read(STOP_NX)

    assert "require_vendor_mros_stopped" in navi
    assert "sudo systemctl stop camera-mros.service" in navi
    assert "/opt/limx/install/bin/([r]slidar_sdk_node|[n]avigation_node)" in navi
    assert "${ROOT_DIR}/build/native_fast_lio/install/lib/rslidar_sdk/" in stop_nx
    assert "  '[r]slidar_sdk_node'" not in stop_nx


def test_navi_rejects_malformed_robot_identity_before_starting(tmp_path):
    result = subprocess.run(
        ["bash", str(NAVI), "nav", "--robot-accid", "bad identity"],
        text=True,
        capture_output=True,
        check=False,
        env=_environment_without_vendor_processes(tmp_path),
    )

    assert result.returncode == 2
    assert "底盘身份格式错误" in result.stderr


def test_navi_passes_selected_robot_identity_to_navigation(tmp_path):
    shutil.copy2(NAVI, tmp_path / "navi.sh")
    for name, content in {
        "stop_nx_navigation_humble.sh": "#!/usr/bin/env bash\nexit 0\n",
        "run_fast_lio_navigation_humble.sh": (
            "#!/usr/bin/env bash\n"
            "printf 'ROBOT_ACCID=%s\\n' \"$WEBSOCKET_ROBOT_ACCID\"\n"
        ),
        "select_navigation_map.py": "# test stub\n",
    }.items():
        path = tmp_path / name
        path.write_text(content, encoding="utf-8")
        path.chmod(0o755)

    env = _environment_without_vendor_processes(tmp_path)
    env["PYTHON_EXECUTABLE"] = "/bin/true"
    result = subprocess.run(
        [
            "bash",
            str(tmp_path / "navi.sh"),
            "nav",
            "--robot-accid",
            "WF_TESTMODEL_002",
        ],
        text=True,
        capture_output=True,
        check=False,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "SELECTED_CHASSIS_ACCID=WF_TESTMODEL_002" in result.stdout
    assert "ROBOT_ACCID=WF_TESTMODEL_002" in result.stdout
