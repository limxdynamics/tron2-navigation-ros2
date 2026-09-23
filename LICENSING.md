# Licensing Boundary

## Repository scope

The copyright owner, LimX Dynamics Technology Co., Ltd., has selected
Apache-2.0 for the original top-level integration scripts, tests, and
documentation. The top-level `LICENSE` applies only to files whose copyright is
held by LimX Dynamics Technology Co., Ltd. and which do not carry a different
license or notice.

`FAST_LIO`, `FAST_LIO_LOCALIZATION2`, and PCT_planner are GPL components and
must remain outside this permissive-source repository. Their licenses are not
changed by the top-level Apache-2.0 license. Compatible versions are published
in the separate repositories listed in
[docs/GPL_EXTERNAL_DEPENDENCIES.md](docs/GPL_EXTERNAL_DEPENDENCIES.md).

Third-party directories remain available only under their own licenses. See
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the LICENSE/NOTICE files in
each component directory.

## Redistribution requirements

Before publishing a release:

1. Export only the permissive subset; never relabel independently
   licensed third-party files as Apache-2.0.
2. Keep all GPL source, headers, generated modules, binaries, plaintext mirrors,
   and Git history outside the permissive repository.
3. Keep SCAN's optional simulator packages outside this production-source
   subset unless they receive a separate provenance and license assessment.
4. Publish any modified GPL component, if required, from a separate GPL-scoped
   repository with its complete corresponding source and notices.
5. Run the permissive-source exporter and integrity check, then verify the
   generated source inventory and checksum before redistributing it.

## Path-based license map

- each named third-party component directory: that component's own LICENSE and
  NOTICE;
- original top-level shell/Python integration, deployment helpers, tests, and
   original documentation: Apache-2.0 under the top-level `LICENSE`; and
- maps, recordings, logs, generated binaries, machine configuration, and
  deployment snapshots: excluded from every public source release.

The LimX WebSocket gateway requires an external compatible signaling service,
robot firmware, authorization, and a valid robot identity. Those external
components are not supplied or licensed by this repository.
