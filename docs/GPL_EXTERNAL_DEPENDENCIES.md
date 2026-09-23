# External GPL Dependencies

The following programs are required only for the complete navigation runtime.
They are not included in the permissive source repository, its source
inventory, release archive, or Git history.

| Runtime unit | License recorded by the pinned source | Canonical upstream clone URL | Recorded baseline commit |
|---|---|---|---|
| FAST-LIO mapping | GPL-2.0-only | `https://github.com/Ericsii/FAST_LIO.git` | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| FAST-LIO localization | GPL-2.0-only | `https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2.git` | `f04974907c8da976dd0495b272d18ac4c534d41f` |
| PCT global planner | GPL version 2 or later | `https://github.com/byangw/PCT_planner.git` | `35cd73fd82bcd51bc538429294af7646b2a09815` |

## Compatible derivative source

Use these separately licensed repositories for this integration:

| Runtime unit | Public repository | Immutable commit | Release tag |
|---|---|---|---|
| FAST-LIO mapping and localization | <https://github.com/limxdynamics/tron2-navigation-fastlio-gpl> | `0fbf6e9cca72a66c330a22d13823f52d49f05948` | `v1.1.0` |
| PCT global planner | <https://github.com/limxdynamics/tron2-navigation-pct-gpl> | `1c553202e50798819715679a7f8c8a878b247ace` | `v1.1.0` |

Clone the main repository, then fetch and verify the two fixed GPL releases:

```bash
git clone https://github.com/limxdynamics/tron2-navigation-ros2.git
cd tron2-navigation-ros2
./deployment/link_external_gpl_sources.sh --fetch --verify-only
```

This creates the two public GPL clones beside `tron2-navigation-ros2`. Existing
paths are never fetched, updated, or checked out automatically. Running
`./install.sh full-nx --jobs 4 --with-tests` also fetches missing GPL clones
before building on a prepared Ubuntu 22.04 aarch64 ROS 2 Humble system.

The two FAST-LIO programs embed `ikd-Tree` source in their executables. Merely
replacing or deleting `ikd-Tree` would not change the FAST-LIO packages'
GPL-2.0-only declarations. PCT's Python process imports its C++ planner
extensions in-process, so the wrapper and extensions are treated as one
external GPL runtime unit for repository separation.

## Compatibility warning

The commits in the first table identify upstream provenance baselines. They are
not drop-in replacements for the separately maintained ROS 2 Humble/aarch64,
RoboSense Fairy, localization, map-save, gravity-alignment, and navigation
adaptations.

Use the compatible derivative commits in the preceding table for the complete
runtime. Each repository includes its complete corresponding source, build
scripts, GPL text, upstream provenance, local modification notices, source
inventory, and immutable commit ID. Do not copy those sources or generated
binaries into this permissive repository.

## Process interfaces

The permissive components do not link against a FAST-LIO or PCT library:

- RoboSense publishes standard ROS 2 point-cloud and IMU messages;
- localization publishes standard pose and point-cloud messages;
- PCT publishes a standard `nav_msgs/Path`;
- SCAN consumes those messages in separate ROS 2 processes.

This documents the engineering boundary only. Qualified counsel must determine
the legal treatment of any particular source or binary distribution.

## PCT embedded dependencies

The excluded PCT baseline retains GTSAM 4.1.1 (BSD-3-Clause), OSQP 0.6.2 and
QDLDL (Apache-2.0), pybind11 (BSD-style), and an Eigen snapshot containing
MPL-2.0, BSD, and LGPL notices. The byte-level provenance result is
summarized in [PCT_EXTERNAL_PROVENANCE.md](PCT_EXTERNAL_PROVENANCE.md). These
files are not part of this permissive repository.
