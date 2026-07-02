#include "CameraControls.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>   // glm::rotate (overview orbit)

namespace {
// Interaction tuning, in one place. File-local: implementation detail of these
// verbs, not part of their contract.
constexpr float kZoomStep       = 0.9f;    // eye-target distance multiplier per zoom notch
constexpr float kMinZoomDist    = 1.0f;    // zoom clamp floor  (can't cross the target)
constexpr float kMaxZoomDist    = 1.0e5f;  // zoom clamp ceiling (can't fly to infinity)
constexpr float kDegenerateDist = 1e-4f;   // guard: eye ~= target, view axis undefined
constexpr float kPanScale       = 0.0015f; // pan pixels -> world, times eye-target distance
constexpr float kOrbitRate      = 0.005f;  // radians per drag pixel
constexpr float kOrbitPoleLimit = 0.985f;  // keep the orbit clear of the vertical
constexpr float kMoveFraction   = 0.05f;   // fly speed as a fraction of terrain size, per second
constexpr float kRotateSpeed    = 1.05f;   // fly look speed, per second
constexpr float kPitchLimit     = 0.99f;   // fly pitch clamp: forward.y never reaches vertical
}

void zoom(Camera &cam, double scrollY)
{
    glm::vec3 forward = cam.target - cam.position;
    float dist = glm::length(forward);
    if (dist < kDegenerateDist)
        return;
    forward /= dist;
    dist = glm::clamp(dist * (scrollY > 0 ? kZoomStep : 1.0f / kZoomStep),
                      kMinZoomDist, kMaxZoomDist);
    cam.position = cam.target - forward * dist;
}

void pan(Camera &cam, double dx, double dy)
{
    const glm::vec3 forward = glm::normalize(cam.target - cam.position);
    const glm::vec3 right   = glm::normalize(glm::cross(forward, cam.up));
    const glm::vec3 up      = glm::normalize(glm::cross(right, forward));
    const float scale = glm::length(cam.target - cam.position) * kPanScale;
    const glm::vec3 delta = right * (float)(-dx) * scale + up * (float)(dy) * scale;
    cam.position += delta;
    cam.target   += delta;
}

void orbit(Camera &cam, double dx, double dy)
{
    glm::vec3 offset = cam.position - cam.target;

    // Yaw: rotate the eye offset around world up.
    offset = glm::vec3(glm::rotate(glm::mat4(1.0f), (float)(-dx) * kOrbitRate,
                                   glm::vec3(0.0f, 1.0f, 0.0f)) * glm::vec4(offset, 0.0f));

    // Pitch: rotate around the view's right axis, applied only while it keeps
    // the view clear of the vertical (degenerate with the world-up axis).
    const glm::vec3 forward = glm::normalize(-offset);
    const glm::vec3 right   = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    const glm::vec3 pitched = glm::vec3(glm::rotate(glm::mat4(1.0f), (float)(-dy) * kOrbitRate, right)
                                        * glm::vec4(offset, 0.0f));
    if (glm::abs(glm::normalize(pitched).y) < kOrbitPoleLimit)
        offset = pitched;

    cam.position = cam.target + offset;
}

void fly(Camera &cam, const MovementIntent &in, float terrainSize, float dt)
{
    const float moveSpeed = terrainSize * kMoveFraction * dt;
    const float rotSpeed  = kRotateSpeed * dt;

    const glm::vec3 forward = glm::normalize(cam.target - cam.position);
    const glm::vec3 right   = glm::normalize(glm::cross(forward, cam.up));

    // Look first (rotate the target around the eye). Pitch is clamped before
    // straight up/down so forward and up never become parallel (degenerate
    // view matrix).
    if (in.pitch > 0 && forward.y <  kPitchLimit) cam.target += cam.up * rotSpeed;
    if (in.pitch < 0 && forward.y > -kPitchLimit) cam.target -= cam.up * rotSpeed;
    cam.target += right * (float)in.yaw * rotSpeed;

    // Then translate eye + target together (forward follows the current
    // pitch; vertical is independent of look direction).
    const glm::vec3 delta = forward  * (float)in.forward
                          + right    * (float)in.strafe
                          + cam.up   * (float)in.vertical;
    cam.position += delta * moveSpeed;
    cam.target   += delta * moveSpeed;

    // Keep the target exactly one unit ahead of the eye, so the look nudges
    // above stay angular rather than drifting the target away.
    cam.target = cam.position + glm::normalize(cam.target - cam.position);
}
