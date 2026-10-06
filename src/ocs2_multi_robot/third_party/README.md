# Bundled qpOASES

`qpOASES/` contains the source and headers from coin-or/qpOASES commit
`268b2f2659604df27c82aa6e32aeddb8c1d5cc7f` (3.2.0), the revision previously
used by the standalone catkin wrapper. The upstream copyright notices,
`LICENSE.txt` (LGPL 2.1 or later), and author information are retained.

`../cmake/QpOases.cmake` builds the shared `qpOASES` library as part of
`ocs2_multi_robot`, without downloading sources during configuration or changing
the workspace's `BUILD_SHARED_LIBS` setting. Headers are installed with their
original directory structure. Catkin exports its headers and library through
`ocs2_multi_robot` in both development and install spaces. The IK module links
this target directly.

This is a source/header subset, not the full upstream release: examples,
interfaces and `doc/manual.pdf` are not bundled. The upstream README is retained
for attribution and background. Build from the workspace root with
`catkin build ocs2_multi_robot`; do not configure the retained upstream `CMakeLists.txt`
standalone, which expects the omitted examples unless explicitly disabled.
