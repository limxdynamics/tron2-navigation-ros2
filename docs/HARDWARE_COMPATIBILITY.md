# Hardware and Firmware Compatibility

## Included functionality

The permissive repository can build and test its RoboSense ROS 2 driver,
production SCAN packages, and fail-closed LimX WebSocket client without moving a
robot. Unit tests use local mock objects, generic model identifiers, and RFC
5737 documentation addresses.

## Requirements for real chassis operation

Real movement additionally requires all of the following external items:

1. an authorized compatible LimX robot;
2. compatible robot firmware exposing the LimX legacy signaling WebSocket
   commands used by the gateway;
3. a reachable signaling service supplied with the robot environment;
4. a valid robot identity assigned by the authorized system, matching
   `WF_<MODEL>_<ID>`;
5. developer/command authorization enabled through the vendor-supported process;
6. calibrated LiDAR/IMU extrinsics, base-yaw convention, maps, and compatible
   external localization/global-planning processes;
7. a physical emergency-stop method and a supervised clear test area.

The signaling server, robot firmware, authorization, credentials, hardware
identity, maps, and vendor services are not part of this repository and are not
licensed by it. No default production endpoint or identity is provided.

## Safe compatibility procedure

1. Build and run all included tests with chassis output disabled.
2. Verify that no MROS command bridge, legacy duplicate gateway, or other
   velocity publisher is active.
3. Configure the untracked `config/navigation.env` on the robot; never commit
   operational values.
4. Start with `START_PAUSED=true` and `CHASSIS_OUTPUT_MODE=off`.
5. Verify localization, frames, point cloud, map, path, QoS, timestamps, topic
   publisher/subscriber counts, and terminal zero velocity.
6. Enable the single legacy WebSocket route at a supervised low test limit.
7. Confirm pause, watchdog, disconnect, rejection, and shutdown each produce
   repeated zero commands before testing motion.
8. Resume only from a real terminal by typing uppercase `GO`.
9. Immediately pause and stop if firmware responses, frames, motion direction,
   speed, or localization differ from the validated contract.

Passing software tests does not certify a new robot model or firmware release.
Each hardware/firmware combination requires supervised acceptance testing.
