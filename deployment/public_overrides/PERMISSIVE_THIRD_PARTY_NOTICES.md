# Third-Party Notices — Permissive Source Subset

The top-level Apache-2.0 license applies only to original integration files for
which the repository owner holds copyright. It does not replace or modify the
licenses of the components listed below.

## Included components

| Component | Upstream and recorded baseline | License | Local status |
|---|---|---|---|
| SCAN-Planner production subset | `https://github.com/wuyi2121/SCAN-Planner.git`, branch `ros2-community`, commit `d0b921c9b05a6d291d144d60882b2e0e88d2c0e0` | Apache-2.0 | ROS 2 port by the external community contributor `xiaoqi371317`; LimX Dynamics production integration on top, including a fail-closed, explicitly gated local-cloud subscription switch; see its `NOTICE` and `UPSTREAM_COMMIT` |
| rslidar_sdk | `https://github.com/RoboSense-LiDAR/rslidar_sdk.git`, commit `8b4b4b7ff910799260347821084c59e1c73d50d5` | BSD-3-Clause | LimX Dynamics modifications for RSFAIRY filtering, IMU/thread shutdown, and ROS 2 integration; see its `NOTICE` |
| embedded rs_driver | `https://github.com/RoboSense-LiDAR/rs_driver.git`, commit `897b14d3bdb6186a75df27ba51b65b5bd5557723` | BSD-3-Clause | Retained under its own `LICENSE`, byte-for-byte except for documentation links, which point at the pinned baseline; see the rslidar_sdk `NOTICE` |
| rslidar_msg | `https://github.com/RoboSense-LiDAR/rslidar_msg.git`, commit `fe8a95cb242bd294cc3d5e3422f2093fb49a56ee` | BSD-3-Clause | LimX Dynamics package metadata corrections use the exact SPDX identifier; see its `NOTICE` |

The recorded SCAN-Planner baseline is the tip of the upstream `ros2-community`
branch; the upstream default branch (`main`) carries the original ROS 1 tree and
shares no common ancestor with it. The baseline commit is therefore only
reachable through `ros2-community`.

### SCAN-Planner attribution

The SCAN-Planner subset combines two separately owned contributions.

The ROS 2 port layer - the ROS 2 Humble / Ubuntu 22.04 package layout, the
ament_cmake and colcon build integration, the rclcpp node and message
adaptation, tf2_ros frame handling, the RViz2 configuration, and the ROS 2
launch files - is an external community contribution. It was written in full by
an individual contributor, `xiaoqi371317`, and published as the upstream
`ros2-community` branch; it is not the work of the original SCAN-Planner
research authors, and that contributor is not affiliated with LimX Dynamics.

The production integration built on top of that layer is original work of LimX
Dynamics under the top-level Apache-2.0 license: the `nx_pct_*` production
configuration and launch files, the fail-closed safe closed-loop controller,
the LimX WebSocket velocity gateway with its tests, the odometry adapter, and
the modifications to baseline files. It is not covered by the community
contributor's copyright notice.

`SCAN-Planner-ros2-community/NOTICE` records the exact new and modified files
and the Apache-2.0 Section 4(b) modification notice.

The SCAN simulator/demo tree is not included. In particular, no `mockamap`,
`local_sensing_node`, `go2_description`, `map_generator`, or
`odom_visualization` package or dependency is distributed in this subset.

## External components not included

The full navigation system may use the following separate programs, but no
source, header, generated module, binary, plaintext mirror, or Git history from
these programs is present in this repository:

| Component | Recorded license | Canonical upstream baseline |
|---|---|---|
| FAST_LIO | GPL-2.0-only | `https://github.com/Ericsii/FAST_LIO.git` at `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| FAST_LIO_LOCALIZATION2 | GPL-2.0-only | `https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2.git` at `f04974907c8da976dd0495b272d18ac4c534d41f` |
| PCT_planner | GPL version 2 or later | `https://github.com/byangw/PCT_planner.git` at `35cd73fd82bcd51bc538429294af7646b2a09815` |

Compatible derivative source is published separately at
<https://github.com/limxdynamics/tron2-navigation-fastlio-gpl> commit
`0fbf6e9cca72a66c330a22d13823f52d49f05948` and
<https://github.com/limxdynamics/tron2-navigation-pct-gpl> commit
`1c553202e50798819715679a7f8c8a878b247ace`. Both commits are tagged
`v1.1.0` in their respective repositories.

See `docs/GPL_EXTERNAL_DEPENDENCIES.md` for compatibility and separate-source
requirements. Mentioning an external program or communicating with a separate
ROS 2 process does not relicense either program. The legal treatment of a
particular distribution remains a matter for qualified counsel.
