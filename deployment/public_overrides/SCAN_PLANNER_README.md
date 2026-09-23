# SCAN-Planner ROS 2 - Production Source Subset

This directory is a production-oriented ROS 2 Humble subset of the upstream
[SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) project, branch
`ros2-community`, at baseline commit
`d0b921c9b05a6d291d144d60882b2e0e88d2c0e0`. See `UPSTREAM_COMMIT` and `NOTICE`
for provenance and `LICENSE` for the Apache-2.0 terms.

## Attribution

Two separately owned contributions are combined here.

- **The ROS 2 port layer is an external community contribution.** The ROS 2
  Humble, Ubuntu 22.04, ament_cmake, colcon, rclcpp, tf2_ros, RViz2 and launch
  integration was written in full by an individual community contributor,
  `xiaoqi371317`, and published as the upstream `ros2-community` branch. It is
  not the work of the original SCAN-Planner research authors (Han Zheng, Zhe
  Chen, Yiwen Fu, Ming Yang, and Tong Qin), and that contributor is not
  affiliated with LimX Dynamics.
- **The production integration built on top of that layer is LimX Dynamics'
  own work.** Copyright 2026 LimX Dynamics Technology Co., Ltd., Apache-2.0. It adds the `nx_pct_*`
  production configuration and launch files, the fail-closed safe closed-loop
  controller, the LimX WebSocket velocity gateway with its tests, the odometry
  adapter, and the modifications to a number of baseline files.

`NOTICE` lists the exact new and modified files and carries the Apache-2.0
Section 4(b) modification notice. Any file not listed there is retained
byte-for-byte from the baseline commit.

## Not included in this subset

The public-source exporter intentionally omits the optional simulator/demo
packages and their demo launch/configuration files. Those packages are not
started by `nx_pct_navigation.launch.py`, are not compile-time dependencies of
the retained planner packages, and have independent license declarations that
are outside this production source subset. They are not distributed here.

## Build

On Ubuntu 22.04 with ROS 2 Humble and the declared dependencies installed:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to scan_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Production launch

The production launch starts the planner, pose adapter, safe closed-loop
controller, and optionally the LimX WebSocket gateway:

```bash
source install/setup.bash
ros2 launch scan_planner nx_pct_navigation.launch.py \
  start_paused:=true start_websocket_bridge:=false
```

The gateway does not include a LimX signaling server or robot firmware. Before
enabling it, provide an authorized compatible `ws://` or `wss://` endpoint and
a robot identity matching `WF_<MODEL>_<ID>`. It starts fail-closed and must be
the only chassis velocity output.

The repository-level `run_nx_navigation_humble.sh` and `navi.sh` coordinate this
package with PCT and enforce the normal paused startup flow.

`grid_map.enable_cloud_subscription` defaults to `true`. Setting it to `false`
removes the local obstacle-cloud subscriber and leaves the local occupancy map
empty. Do not disable it by launching this package directly. Use the top-level
`navi.sh nav --no-obstacle-avoidance --speed MPS` flow, which requires paused
startup, limits the speed to 0.60 m/s, and requires a separate operator
confirmation before motion.
