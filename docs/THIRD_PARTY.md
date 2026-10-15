# Third-party sources and licenses

- **LTAOM:** upstream attribution and the root LICENSE are retained, along with licenses in the FAST-LIO subtree and notices in individual source files.
- **Eigen:** version 3.4.0 headers are pinned from an Ubuntu package. Source notices, COPYRIGHT.debian, and the LGPL text are retained. Header hashes are recorded in [locks/eigen_headers.json](../locks/eigen_headers.json).
- **Scan Context:** sourced from `cpp/module/Scancontext` at commit `93672835bb85e9c03fedf0fbaf2d0491df12161e`, with upstream license notices and documentation retained.
- **Quatro, Nano-GICP, TEASER++, and PMC:** only the source files needed for compilation and their licenses are included. Original commits and license hashes are recorded in [locks/registration_sources.json](../locks/registration_sources.json). Upstream example point clouds and demo images are omitted.
- **GTSAM and Ceres:** commits are pinned in [locks/build_dependencies.json](../locks/build_dependencies.json). The [patches](../patches) directory contains compatibility and snapshot restoration patches. Downloaded sources retain their respective licenses.

Quatro's README states CC-BY-NC-SA-4.0, while its License file states GPL-3.0. This discrepancy remains unresolved; both are retained as supplied upstream. This repository does not replace third-party licenses with a single license or claim that commercial licensing has been reviewed.
