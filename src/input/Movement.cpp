#include "Movement.h"

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

// True while `key` is physically held down this frame. Continuous movement is a
// polling model: instead of reacting to discrete press/repeat events, every frame we
// ask GLFW which keys are down right now and integrate motion over dt.
static bool held(GLFWwindow *window, int key)
{
    return glfwGetKey(window, key) == GLFW_PRESS;
}

void moveCamera(Camera &camera, float terrainSize, GLFWwindow *window, float dt)
{
    // Scale by dt so speed is the same regardless of frame rate. moveSpeed also scales
    // with terrain size so the feel is consistent across differently sized DEMs.
    const float moveSpeed     = terrainSize * 0.05f * dt;
    const float rotationSpeed = 1.05f * dt;

    const bool shift = held(window, GLFW_KEY_LEFT_SHIFT) ||
                       held(window, GLFW_KEY_RIGHT_SHIFT);

    glm::vec3 forward = glm::normalize(camera.target - camera.position);
    glm::vec3 right   = glm::normalize(glm::cross(forward, camera.up));

    // ── Look (rotate the target around the eye) ──────────────────
    // Nudging the target along the up/right vectors and renormalizing below turns it
    // into a yaw/pitch. Pitch is clamped before it reaches straight up/down so forward
    // and up never become parallel (which would make the view matrix degenerate).
    if (held(window, GLFW_KEY_UP)    && forward.y <  0.99f) camera.target += camera.up * rotationSpeed;
    if (held(window, GLFW_KEY_DOWN)  && forward.y > -0.99f) camera.target -= camera.up * rotationSpeed;
    if (held(window, GLFW_KEY_LEFT))                        camera.target -= right     * rotationSpeed;
    if (held(window, GLFW_KEY_RIGHT))                       camera.target += right     * rotationSpeed;

    // ── Translate (free-fly: forward follows pitch) ──────────────
    glm::vec3 delta(0.0f);
    if (held(window, GLFW_KEY_W)) delta += forward * moveSpeed;
    if (held(window, GLFW_KEY_S)) delta -= forward * moveSpeed;
    if (held(window, GLFW_KEY_A)) delta -= right   * moveSpeed;
    if (held(window, GLFW_KEY_D)) delta += right   * moveSpeed;

    // Altitude is independent of look direction (Shift + > / <).
    if (shift && held(window, GLFW_KEY_PERIOD)) delta += camera.up * moveSpeed;
    if (shift && held(window, GLFW_KEY_COMMA))  delta -= camera.up * moveSpeed;

    camera.position += delta;
    camera.target   += delta;

    // Keep the target exactly one unit ahead of the eye, so the rotation nudges above
    // stay angular (constant look distance) rather than drifting the target away.
    camera.target = camera.position + glm::normalize(camera.target - camera.position);
}
