# Excluded PCT Dependency Provenance

PCT_planner and all of its embedded dependencies are excluded from the
permissive repository. This record summarizes the source comparison without
copying those sources into this repository.

The comparison baseline was PCT_planner commit
`35cd73fd82bcd51bc538429294af7646b2a09815`.

| Dependency | Prior comparison result | Recorded license information |
|---|---|---|
| GTSAM 4.1.1 | 3,824 upstream files and 3,824 retained files; two local changes were SPDX/package metadata | BSD-3-Clause |
| OSQP 0.6.2 | 212 upstream files and 214 retained files; two added offline-build configuration headers and no modified/deleted upstream paths | Apache-2.0 |
| pybind11 | 230 retained files byte-identical to the baseline | BSD-style |
| Eigen snapshot inside GTSAM | 1,147 retained files byte-identical to the baseline snapshot | Primarily MPL-2.0; retained tree also contains per-file BSD/LGPL notices |

Eigen's upstream documentation describes the source tree as primarily MPL-2.0
and provides `EIGEN_MPL2_ONLY` to reject non-MPL2 implementation files. The
recorded PCT build did not define that option, so this document does not claim that
every possible Eigen source file is exclusively MPL-2.0 or that a particular
final binary used only MPL-2.0 files.

The companion PCT repository preserves the complete upstream license set and
source-file provenance. No GTSAM, OSQP, QDLDL, pybind11, or Eigen source is
shipped in this permissive archive.
