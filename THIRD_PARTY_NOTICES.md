# Third-party notices and provenance

This notice covers the workspace's submodules: OCS2 (`src/ocs2`),
OCS2 Robotic Assets (`src/ocs2_robotic_assets`), and Mapping Third-Party
(`src/mapping_third_party`), including their embedded
and fetched dependencies. Other workspace packages, assets, and system packages
are outside this inventory.

The root [MIT license](LICENSE) applies only to original ADMM-CLM contributions
and does not replace these dependencies' licenses. Preserve individual file
copyright notices. This inventory is not a complete software bill of materials
or a legal opinion.

## OCS2

- Location: `src/ocs2`.
- Original upstream: [leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2).
- Workspace fork: [GTLIDAR/ocs2](https://github.com/GTLIDAR/ocs2).
- Reviewed revision: `1255cbdc85e41a85ad2e783a68ad1026e282766b`.

Core source headers contain BSD-3-Clause terms and individual copyright holders.
Embedded and fetched libraries have separate licenses, listed below.
Some manifests contain `TODO`, which is not a license grant. The fork and
workspace integration differ from upstream; retain upstream copyright headers.

| Dependency | Version/source | License evidence |
| --- | --- | --- |
| BLASFEO | [giaf/blasfeo](https://github.com/giaf/blasfeo), `ae6e2d1dea015862a09990b95905038a756ffc7d` | [BSD-2-Clause](LICENSES/BLASFEO.txt), copied from the pinned downloaded source |
| HPIPM | [giaf/hpipm](https://github.com/giaf/hpipm), `255ffdf38d3a5e2c3285b29568ce65ae286e5faf` | [BSD-2-Clause](LICENSES/HPIPM.txt), copied from the pinned downloaded source |
| CppAD | `src/ocs2/ocs2_thirdparty/include/cppad` | [COPYING](src/ocs2/ocs2_thirdparty/include/cppad/COPYING): EPL-2.0 with conditional GPL secondary licensing; retain the bundled terms |
| CppADCodeGen | `src/ocs2/ocs2_thirdparty/include/cppad/cg` | [COPYING](src/ocs2/ocs2_thirdparty/include/cppad/cg/COPYING): EPL-1.0 / GPL-3.0 license options; inspect and comply with the chosen option |

The fetched BLASFEO/HPIPM revisions are defined in their catkin wrappers.

## OCS2 Robotic Assets

- Location: `src/ocs2_robotic_assets`.
- Workspace repository: [GTLIDAR/ocs2_robotic_assets](https://github.com/GTLIDAR/ocs2_robotic_assets).
- Original upstream: [leggedrobotics/ocs2_robotic_assets](https://github.com/leggedrobotics/ocs2_robotic_assets).
- ROS 1-compatible revision: `b126d00d55f1e67905c3c1516995466df6873c5c`.

Provides robot descriptions and meshes for OCS2 examples and tests. The
repository retains an [Apache-2.0 license text](src/ocs2_robotic_assets/LICENSE),
while its package manifest declares BSD-3. These differing declarations do not
establish a single license for all resources;
individual resource directories also contain their own license files, which
remain applicable. The default `ros2` branch is not used in this catkin workspace.

## Mapping Third-Party

`src/mapping_third_party` is a vendored collection, not an original mapping
library or a single-license project. The maintainer identifies its direct source as
[DRCL-USC/Quadruped_Wrapper](https://github.com/DRCL-USC/Quadruped_Wrapper/tree/837f51807e192241437c642bc5dc9c93a46224f0/third_party),
commit `837f51807e192241437c642bc5dc9c93a46224f0`.
That commit's `.gitmodules` and `third_party` Git tree identify the upstreams
and gitlinks below. These are **source-checkout pins**, not a claim that our
copied and modified directories are byte-identical to those commits.
The collection baseline reviewed here is `da19c97ac6977a3396abc3c4eecb168f870212eb`
in [GTLIDAR/mapping_third_party](https://github.com/GTLIDAR/mapping_third_party).

| Directory | Original upstream | Pin in the direct source | Retained license |
| --- | --- | --- | --- |
| `elevation_mapping` | [ANYbotics/elevation_mapping](https://github.com/ANYbotics/elevation_mapping) | `3f75435d09415a166851f039035f5b2df2d338e7` | [BSD-3-Clause](src/mapping_third_party/elevation_mapping/LICENSE) |
| `elevation_mapping_cupy` | [leggedrobotics/elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) | `adac6213dcfa6e865b88ade3cab5d96b41bb20cd` | [MIT](src/mapping_third_party/elevation_mapping_cupy/LICENSE); nested dependencies have their own terms |
| `grid_map` | [ANYbotics/grid_map](https://github.com/ANYbotics/grid_map) | `552ed01ed3b39886f52b93be0fc32f188fbbc1c5` | [BSD-3-Clause](src/mapping_third_party/grid_map/LICENSE); embedded EigenLab has a [separate license](src/mapping_third_party/grid_map/grid_map_filters/include/EigenLab/LICENSE) |
| `kindr` | [ANYbotics/kindr](https://github.com/ANYbotics/kindr) | `97676b0577400eb67f7b6f68ff2fa05e536ea64a` | [BSD-3-Clause](src/mapping_third_party/kindr/LICENSE) |
| `kindr_ros` | [ANYbotics/kindr_ros](https://github.com/ANYbotics/kindr_ros) | `c448a55a003a93f82404766faa2fa51af40358be` | [BSD-3-Clause](src/mapping_third_party/kindr_ros/LICENSE) |
| `message_logger` | [ANYbotics/message_logger](https://github.com/ANYbotics/message_logger) | `cf7fd265327a690f560a0be5a622ebfca76baed8` | [BSD-3-Clause](src/mapping_third_party/message_logger/LICENSE) |
| `point_cloud_io` | [ANYbotics/point_cloud_io](https://github.com/ANYbotics/point_cloud_io) | `dd33bee05034e71fd220398a12db9df09b7b05e2` | [BSD-3-Clause](src/mapping_third_party/point_cloud_io/LICENSE) |

The workspace uses the C++ elevation mapper and the CPU plane-decomposition
components under `elevation_mapping_cupy/plane_segmentation`, not the GPU
CuPy mapper or its multi-modal demos. Local integration includes build and
configuration adaptations; a complete file-by-file upstream patch inventory
has not been established. Documentation changes are not changes of authorship.

### Fetched dependency

| Dependency | Version/source | License evidence |
| --- | --- | --- |
| CGAL | [CGAL/cgal](https://github.com/CGAL/cgal/tree/v5.3), 5.3, fetched by `cgal5_catkin` | [Release licensing overview](LICENSES/CGAL.txt); GPL-3.0-or-later and LGPL-3.0-or-later by component, or separate commercial licensing |

CGAL versions are defined in the mapping collection's wrapper.
The submodule revisions record reviewed baselines, not new upstream releases.
The parent Git commit and each submodule's Git history identify the actual
version in a checkout. This notice does not replace a release-specific
dependency audit.

## Distribution conditions and open questions

- **BSD and MIT:** retain copyright, complete conditions, and disclaimers.
  For BSD binary redistribution, reproduce notices in accompanying materials.
  Source headers contain distinct authors and years; do not replace them with
  a single project copyright.
- **CGAL is a release blocker for an MIT-only binary claim:** plane decomposition
  directly includes `CGAL/Shape_detection/Efficient_RANSAC.h`. In CGAL 5.3,
  `Shape_detection` is GPL-3.0-or-later; `Polygon` is LGPL-3.0-or-later.
  Header-only use does not remove these conditions. Assess the GPL requirements
  for the combined binaries and corresponding source before publishing an
  image or executable, or obtain a suitable commercial license / change the
  dependency. An MIT grant on original code can coexist with distribution of
  a combined work under applicable GPL conditions, but does not waive them.
- **Other dependency terms:** comply with the applicable EPL/GPL option for
  OCS2's embedded libraries. Shared linking alone does not establish LGPL
  compliance; satisfy applicable source-availability, replacement/relinking,
  and notice requirements for the actual packaging.
- **Remaining provenance:** OCS2 manifest values such as `TODO` must not be
  treated as authorization. A complete file-by-file inventory of changes to
  the submodules has not been established.

The Dockerfile includes this notice, the root MIT grant, `CITATION.cff`, and
`LICENSES/`, as well as the copied source tree and its existing license files.
The core install rules also ship these workspace-level materials. This improves
notice availability; it is **not** a declaration that every binary/source
distribution obligation is satisfied. The previously published Docker image
has not been audited and is not updated by editing this checkout.

## Academic attribution

Cite upstream methods actually used. [CITATION.cff](CITATION.cff) also records
the OCS2, grid_map, and elevation_mapping references:

- **OCS2:** Farbod Farshidian and others, *OCS2: An open source library for
  Optimal Control of Switched Systems*. Use the upstream toolbox entry in
  [Citing OCS2](src/ocs2/ocs2_doc/docs/overview.rst), and identify this project's
  fork separately when describing reproduction.
- **grid_map:** Peter Fankhauser and Marco Hutter, *A Universal Grid Map Library:
  Implementation and Use Case for Rough Terrain Navigation*, 2016,
  DOI [10.1007/978-3-319-26054-9_5](https://doi.org/10.1007/978-3-319-26054-9_5).
  See its [publication entry](src/mapping_third_party/grid_map/README.md#publications).
- **C++ elevation_mapping:** Peter Fankhauser, Michael Bloesch, and Marco Hutter,
  *Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization*,
  RA-L, 2018, DOI [10.1109/LRA.2018.2849506](https://doi.org/10.1109/LRA.2018.2849506).
  Its [Citing section](src/mapping_third_party/elevation_mapping/README.md#citing)
  also includes the earlier robot-centric mapping publication.
- **Plane decomposition / optional GPU mapping:** acknowledge the
  [elevation_mapping_cupy project](src/mapping_third_party/elevation_mapping_cupy/README.md)
  as the source of the plane-segmentation components. Do not describe the
  current CPU pipeline as using the CuPy GPU mapper. If GPU or multi-modal
  methods are actually used in additional experiments, follow that README's
  corresponding citation instructions.
- **Internal dependencies:** consult BLASFEO, HPIPM, CGAL, and other dependencies
  of these submodules for the algorithms actually used in a publication.
  Do not replace their original authors with the dependency collection's
  maintainer.

We acknowledge the original library authors and
[DRCL-USC/Quadruped_Wrapper](https://github.com/DRCL-USC/Quadruped_Wrapper)
as the direct source of the mapping dependency collection. Citation is
academic attribution, not a substitute for license compliance.
