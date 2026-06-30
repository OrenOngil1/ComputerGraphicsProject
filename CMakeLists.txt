# ── Project ───────────────────────────────────────────────────
# 3.16 is the floor that ships with Ubuntu 20.04; everything below
# (imported targets, CONFIGURE_DEPENDS, cxx_std_17) is available there.
cmake_minimum_required(VERSION 3.16)
project(drone_sim LANGUAGES C CXX)

# Default to a Release build if the user didn't pick one. CMake, not us,
# then supplies the right optimization flag per compiler (-O2 / -O3 / /O2),
# which is why -O2 is absent from our own flag lists below.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Collect executables under the build tree (build/bin), not the source root.
# In CMake the out-of-source build/ directory is the single home for every
# generated artifact -- intermediates *and* binaries -- so it subsumes the
# Makefile's separate obj/ + bin/. Keeping output inside build/ means the two
# build systems never write to each other's directories.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ── Dependencies ──────────────────────────────────────────────
# Each find_package yields an *imported target* that already knows that
# platform's library name and include path. OpenGL::GL, for instance,
# resolves to libGL on Linux, opengl32 on Windows, and the GL framework on
# macOS -- so nothing here branches per-OS. How the libs get *installed*
# differs (apt / vcpkg / brew); how we *consume* them does not.
find_package(OpenGL REQUIRED)
find_package(glfw3  REQUIRED)
find_package(OpenCV REQUIRED)

# ── Core library ──────────────────────────────────────────────
# Everything except the entry point goes into one static library that both
# the app and the test binary link against -- so shared translation units
# compile once. main.cpp is excluded so the test can supply its own main().
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM CORE_SOURCES ${CMAKE_SOURCE_DIR}/src/main.cpp)

add_library(drone_core STATIC
  ${CORE_SOURCES}
  src/glad.c            # vendored C loader; CMake compiles it with the C toolchain
)

# PUBLIC = these propagate to anything that links drone_core (the app *and*
# the tests), mirroring how the Makefile fed $(LIBS) to both targets.
target_include_directories(drone_core PUBLIC
  ${CMAKE_SOURCE_DIR}/include       # vendored glad / KHR / glm headers
  ${CMAKE_SOURCE_DIR}/src/engine    # matches the Makefile's -Isrc/engine
)
target_compile_definitions(drone_core PUBLIC GLFW_INCLUDE_NONE)
target_compile_features(drone_core PUBLIC cxx_std_17)
target_link_libraries(drone_core PUBLIC
  OpenGL::GL
  glfw
  ${OpenCV_LIBS}
)

# ── Warnings (the one genuinely per-compiler knob) ────────────
# Library/include flags above are OS-agnostic; only warning syntax differs
# between MSVC and GCC/Clang, so this is the whole per-toolchain surface.
if(MSVC)
  target_compile_options(drone_core PRIVATE /W4)
else()
  target_compile_options(drone_core PRIVATE -Wall -Wextra)
endif()

# ── Executables ───────────────────────────────────────────────
add_executable(drone_sim src/main.cpp)
target_link_libraries(drone_sim PRIVATE drone_core)

# Headless math checks (PnP solvers, blob centroids, terrain normals) -- no
# window or GPU. We link the whole core lib; the static linker keeps only the
# objects headless_checks.cpp actually references, so this gets the same
# "just the GL-free pieces" effect the Makefile spelled out by hand.
add_executable(headless_checks tests/headless_checks.cpp)
target_link_libraries(headless_checks PRIVATE drone_core)

enable_testing()
add_test(NAME headless_checks COMMAND headless_checks)
