#pragma once

#include "../core/Camera.h"

struct GLFWwindow;   // only a pointer is needed here -- keep GLFW out of this header

// Continuous, frame-rate-independent FPS movement of the (one) player camera. Polls
// the currently-held keys via glfwGetKey and applies a dt-scaled update, so it is
// meant to be called once per frame (from a State's handleMovement), not per key event.
//   W/S          move forward/back along the look direction
//   A/D          strafe left/right
//   arrow keys   look (yaw/pitch), pitch clamped near vertical
//   Q / E        raise / lower altitude
void moveCamera(Camera &camera, float terrainSize, GLFWwindow *window, float dt);
