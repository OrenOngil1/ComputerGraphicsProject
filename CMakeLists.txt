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

# Collect executables under the build tree (<binaryDir>/bin -- the presets
# point that at build/<preset-name>), not the source root.
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

# ── Vendored components (external/) ───────────────────────────
# Third-party code built from source, one target each; default warnings
# (the strict flags below are reserved for code authored in src/).

# The glad OpenGL loader, in its upstream layout (a C file + its headers).
add_library(glad STATIC external/glad/src/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_SOURCE_DIR}/external/glad/include)

# The BasicOpenGL course toolkit: thin GL object wrappers, included as
# <Shader.h> etc. Its headers use glad (gl* types) and glm (Shader.h).
file(GLOB ENGINE_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/external/engine/*.cpp)
add_library(engine STATIC ${ENGINE_SOURCES})
target_include_directories(engine PUBLIC
  ${CMAKE_SOURCE_DIR}/external/engine
  ${CMAKE_SOURCE_DIR}/include       # glm (header-only)
)
target_compile_features(engine PUBLIC cxx_std_17)
target_link_libraries(engine PUBLIC glad)

# ── Core library ──────────────────────────────────────────────
# Everything except the entry point goes into one static library that both
# the app and the test binary link against -- so shared translation units
# compile once. main.cpp is excluded so the test can supply its own main().
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM CORE_SOURCES ${CMAKE_SOURCE_DIR}/src/main.cpp)

add_library(drone_core STATIC ${CORE_SOURCES})

# PUBLIC = these propagate to anything that links drone_core (the app *and*
# the tests).
target_include_directories(drone_core PUBLIC
  ${CMAKE_SOURCE_DIR}/include       # glm (header-only)
)
target_compile_definitions(drone_core PUBLIC GLFW_INCLUDE_NONE)
target_compile_features(drone_core PUBLIC cxx_std_17)
target_link_libraries(drone_core PUBLIC
  engine
  OpenGL::GL
  glfw
  ${OpenCV_LIBS}
)

# ── Warnings (the one genuinely per-compiler knob) ────────────
# Library/include flags above are OS-agnostic; only warning syntax differs
# between MSVC and GCC/Clang, so this is the whole per-toolchain surface.
# Scoped to drone_core: vendored targets keep default warnings.
if(MSVC)
  target_compile_options(drone_core PRIVATE /W4)
else()
  target_compile_options(drone_core PRIVATE -Wall -Wextra)
endif()

# ── Executables ───────────────────────────────────────────────
add_executable(drone_sim src/main.cpp)
target_link_libraries(drone_sim PRIVATE drone_core)

# Headless checks -- no window or GPU. One binary built from tests/*.cpp,
# one file per topic; it links the whole core lib and the static linker keeps
# only the objects each file actually references, so no GL is pulled in.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/tests/*.cpp)

add_executable(headless_checks ${TEST_SOURCES})
target_link_libraries(headless_checks PRIVATE drone_core)

enable_testing()
add_test(NAME headless_checks COMMAND headless_checks)
