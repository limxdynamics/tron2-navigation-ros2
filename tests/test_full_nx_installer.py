# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INSTALLER = ROOT / "deployment" / "install_full_navigation_nx.sh"


def installer_text():
    return INSTALLER.read_text(encoding="utf-8")


def test_fresh_nx_bootstrap_contains_observed_missing_dependencies():
    script = installer_text()
    for package in (
        "python3-open3d",
        "python3-transforms3d",
        "python3-websocket",
        "libpcl-dev",
        "ros-humble-pcl-ros",
    ):
        assert package in script


def test_fresh_nx_bootstrap_installs_and_executes_map_tool_self_check():
    script = installer_text()
    assert "INSTALL_MAP_TOOLS=true" in script
    assert "cupy-cuda12x==13.6.0" in script
    assert "rosbags==0.9.23" in script
    assert "pypcd4==1.4.3" in script
    assert "cupy.cuda.runtime.getDeviceCount()" in script
    assert "cupy.arange(8, dtype=cupy.int32).sum().get()" in script


def test_full_environment_preflight_checks_required_ros_runtime_packages():
    script = installer_text()
    for package in (
        "geometry_msgs",
        "nav_msgs",
        "pcl_ros",
        "rclcpp",
        "rclpy",
        "sensor_msgs",
        "std_srvs",
        "tf2_ros",
    ):
        assert package in script
    assert 'ros2 pkg prefix "${ros_package}"' in script


def test_ros_cli_is_checked_after_sourcing_humble():
    script = installer_text()
    preflight = script.split("preflight_environment() {", 1)[1].split(
        "\n}\n\nverify_pct_build", 1
    )[0]
    assert preflight.index('source "${ROS_SETUP}"') < preflight.index(
        "command -v ros2"
    )


def test_fresh_overlays_are_sourced_before_ctest():
    script = installer_text()
    fast_overlay = script.index('source "${fast_install}/setup.bash"')
    scan_overlay = script.index('source "${scan_install}/setup.bash"')
    ctest_loop = script.index(
        "for package in rslidar_msg rslidar_sdk fast_lio fast_lio_localization"
    )
    assert fast_overlay < ctest_loop
    assert scan_overlay < ctest_loop
