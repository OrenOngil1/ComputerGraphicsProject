// Headless sanity checks for zoom/pan/orbit/fly -- no window, no GL context.

#include <cmath>
#include <iostream>

#include <glm/glm.hpp>

#include "check.h"
#include "../src/input/CameraControls.h"

static Camera makeCamera(glm::vec3 position, glm::vec3 target)
{
    Camera cam;
    cam.position = position;
    cam.target   = target;
    cam.up       = glm::vec3(0.0f, 1.0f, 0.0f);
    cam.fov = 45.0f;
    cam.near = 0.1f;
    cam.far  = 1000.0f;
    return cam;
}

static float eyeTargetDist(const Camera &c) { return glm::length(c.target - c.position); }

void testCameraControls()
{
    std::cout << "CameraControls:" << std::endl;

    // zoom in: one notch scales the eye-target distance by 0.9, along the same view
    // axis, target fixed.
    {
        Camera cam = makeCamera({ 0, 0, 100 }, { 0, 0, 0 });
        zoom(cam, +1.0);   // scroll up = zoom in
        check(std::abs(eyeTargetDist(cam) - 90.0f) < 1e-3f &&
              glm::length(cam.target) < 1e-5f &&
              glm::length(cam.position - glm::vec3(0, 0, 90)) < 1e-3f,
              "zoom in: distance x0.9 along the view axis, target fixed");
    }

    // zoom floor: from distance 1 a further zoom-in can't cross the target.
    {
        Camera cam = makeCamera({ 0, 0, 1 }, { 0, 0, 0 });
        zoom(cam, +1.0);
        check(std::abs(eyeTargetDist(cam) - 1.0f) < 1e-4f,
              "zoom in: clamps at the near floor (can't cross the target)");
    }

    // zoom ceiling: from the max distance a zoom-out can't fly to infinity.
    {
        Camera cam = makeCamera({ 0, 0, 1.0e5f }, { 0, 0, 0 });
        zoom(cam, -1.0);
        check(std::abs(eyeTargetDist(cam) - 1.0e5f) < 1.0f,
              "zoom out: clamps at the far ceiling");
    }

    // zoom degenerate guard: eye == target is left untouched (no NaN / divide-by-zero).
    {
        Camera cam = makeCamera({ 5, 5, 5 }, { 5, 5, 5 });
        zoom(cam, +1.0);
        check(glm::length(cam.position - glm::vec3(5, 5, 5)) < 1e-6f,
              "zoom: degenerate eye==target is left unchanged");
    }

    // pan: eye AND target shift by the same nonzero delta (distance preserved -> the
    // map slides under the cursor rather than the camera turning).
    {
        Camera cam = makeCamera({ 0, 0, 100 }, { 0, 0, 0 });
        const glm::vec3 p0 = cam.position, t0 = cam.target;
        pan(cam, 40.0, -25.0);
        const glm::vec3 dp = cam.position - p0, dt = cam.target - t0;
        check(glm::length(dp - dt) < 1e-5f && glm::length(dp) > 1e-4f &&
              std::abs(eyeTargetDist(cam) - 100.0f) < 1e-3f,
              "pan: position and target move by the same nonzero delta");
    }

    // orbit: a pure rotation about the target -> eye-target distance is preserved.
    {
        Camera cam = makeCamera({ 0, 0, 100 }, { 0, 0, 0 });
        orbit(cam, 50.0, 20.0);
        check(std::abs(eyeTargetDist(cam) - 100.0f) < 1e-3f,
              "orbit: eye-target distance preserved (pure rotation)");
    }

    // orbit pole clamp: hammering pitch toward vertical never lets the view reach the
    // pole (|normalized offset.y| stays under the 0.985 limit).
    {
        Camera cam = makeCamera({ 0, 0, 100 }, { 0, 0, 0 });
        for (int i = 0; i < 200; i++)
            orbit(cam, 0.0, -100.0);   // keep pitching the same way
        const glm::vec3 offset = glm::normalize(cam.position - cam.target);
        check(std::abs(offset.y) < 0.985f + 1e-3f,
              "orbit: pitch clamp keeps the view clear of the vertical");
    }

    // fly forward: W slides the eye along the look direction; target stays one unit
    // ahead. moveSpeed = terrainSize * 0.05 * dt = 100 * 0.05 * 0.1 = 0.5.
    {
        Camera cam = makeCamera({ 0, 0, 0 }, { 0, 0, -1 });
        MovementIntent in;
        in.forward = 1;
        fly(cam, in, 100.0f, 0.1f);
        check(glm::length(cam.position - glm::vec3(0, 0, -0.5f)) < 1e-4f &&
              std::abs(eyeTargetDist(cam) - 1.0f) < 1e-4f,
              "fly: W moves along forward; target stays one unit ahead");
    }

    // fly pitch clamp: pitching up when already past the near-vertical limit is refused,
    // so forward.y does not climb further (forward/up never become parallel).
    {
        const glm::vec3 tgt(0.0f, 0.995f, std::sqrt(1.0f - 0.995f * 0.995f)); // forward.y=0.995 > 0.99
        Camera cam = makeCamera({ 0, 0, 0 }, tgt);
        const float beforeY = glm::normalize(cam.target - cam.position).y;
        MovementIntent in;
        in.pitch = 1;   // try to look further up
        fly(cam, in, 100.0f, 0.1f);
        const float afterY = glm::normalize(cam.target - cam.position).y;
        check(afterY <= beforeY + 1e-6f,
              "fly: pitch clamp refuses to look past vertical");
    }
}
