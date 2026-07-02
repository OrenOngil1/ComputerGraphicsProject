# Third-Party Notices

This project (licensed under the MIT License — see [`LICENSE`](LICENSE)) bundles
or depends on the following third-party components. Each is the property of its
respective authors and is used under its own license.

## Vendored in this repository

| Component | Location | Version | License |
| --------- | -------- | ------- | ------- |
| **GLM** (OpenGL Mathematics) | `include/glm/` | 0.9.9.8 | MIT / The Happy Bunny License — see [`include/glm/copying.txt`](include/glm/copying.txt) |
| **glad** (OpenGL loader, generated) | `external/glad/` | glad 0.1.36 (gl 3.3 core) | Generated loader is public-domain; `KHR/khrplatform.h` carries the Khronos Group MIT-style license inline in the file header |
| **BasicOpenGL toolkit** | `external/engine/` | course-issued | Course-distributed teaching toolkit, vendored and used with permission |

## Fetched at build time (not redistributed here)

These are resolved by the build (vcpkg on Windows, system packages on
Linux/macOS) and are **not** included in this repository:

| Component | License |
| --------- | ------- |
| **GLFW** | zlib/libpng License |
| **OpenCV** | Apache License 2.0 |
