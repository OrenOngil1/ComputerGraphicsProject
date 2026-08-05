// Headless contract check: the renderer's viewProjection (Camera.h) and the
// vision side's pinhole model must describe the same camera, or every 2D-3D
// correspondence in the app silently means two different things. World points
// are projected both ways -- through the GL matrix + viewport transform, and
// through an independently derived square-pixel pinhole -- and must land on
// the same pixel. The viewing-ray mapping (fractionToRay / rayToFraction,
// PICK's resize-proof observation form) is checked against the same standard.

#include <cmath>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "check.h"
#include "../src/core/Camera.h"

namespace {

// The render half is Camera.h's own `rasterize` -- clip space via
// viewProjection, perspective divide, viewport transform -- held here against
// the independent standard below rather than restated.

// The vision half: the square-pixel pinhole (fx == fy from the VERTICAL fov,
// principal point at the center) -- the same outside standard test_pnp.cpp
// validates the solvers against, derived here without viewProjection.
glm::vec2 pinholeProject(const Camera &camera, const Viewport &viewport, const glm::vec3 &p)
{
    // glm::lookAt yields the GL camera frame (x right, y up, looking down -z);
    // the image frame has y down and z forward, so y and z flip sign.
    const glm::mat4 view = glm::lookAt(camera.position, camera.target, camera.up);
    const glm::vec3 pc = glm::vec3(view * glm::vec4(p, 1.0f));
    const float f = viewport.height / (2.0f * std::tan(glm::radians(camera.fov) / 2.0f));
    return { f * (pc.x / -pc.z) + viewport.width  / 2.0f,
             f * (-pc.y / -pc.z) + viewport.height / 2.0f };
}

// A world point's camera-space viewing ray (x/z, y/z) in image convention
// (x right, y down, z forward) -- what PickState stores for a 2D click.
glm::vec2 cameraRay(const Camera &camera, const glm::vec3 &p)
{
    const glm::mat4 view = glm::lookAt(camera.position, camera.target, camera.up);
    const glm::vec3 pc = glm::vec3(view * glm::vec4(p, 1.0f));
    return { pc.x / -pc.z, -pc.y / -pc.z };
}

// Points spread across the frustum, off-center on both axes.
const glm::vec3 kPoints[] = {
    { 0, 0, 0 }, { 6, 2, -4 }, { -5, 3, 2 }, { 3, -2, 6 }, { -2, 5, -3 },
};

bool agreesEverywhere(const Camera &camera, const Viewport &viewport)
{
    for (const glm::vec3 &p : kPoints)
        if (glm::length(rasterize(camera, viewport, p)
                        - pinholeProject(camera, viewport, p)) > 0.1f)
            return false;
    return true;
}

}  // namespace

void testCameraModel()
{
    std::cout << "camera model (viewProjection vs pinhole):" << std::endl;

    Camera camera{ { 12, 9, 15 }, { 1, 2, 0 }, { 0, 1, 0 }, 50.0f, 0.1f, 1000.0f };

    // Both aspect directions: a width/height factor wrongly folded into the
    // focal length (or an x/y swap) shifts the two projections apart.
    check(agreesEverywhere(camera, Viewport{ 0, 0, 960, 640 }),
          "matches the pinhole on a landscape viewport (sub-0.1px)");
    check(agreesEverywhere(camera, Viewport{ 0, 0, 640, 960 }),
          "matches the pinhole on a portrait viewport (sub-0.1px)");

    // Clip planes affect depth only; the pinhole has none, so pixel positions
    // must not move with them.
    camera.near = 5.0f;
    camera.far = 60.0f;
    check(agreesEverywhere(camera, Viewport{ 0, 0, 960, 640 }),
          "pixel positions are independent of the near/far planes");

    // The ray mapping is a bijection at any one aspect.
    const Viewport landscape{ 0, 0, 960, 640 }, portrait{ 0, 0, 640, 960 };
    const glm::vec2 uvs[] = { { 0.5f, 0.5f }, { 0.1f, 0.8f }, { 0.9f, 0.2f }, { 0.0f, 1.0f } };
    bool roundTrip = true;
    for (const glm::vec2 &uv : uvs) {
        const glm::vec2 back = rayToFraction(
            fractionToRay(uv, camera.fov, landscape.aspect()),
            camera.fov, landscape.aspect());
        roundTrip = roundTrip && glm::length(back - uv) < 1e-6f;
    }
    check(roundTrip, "fractionToRay / rayToFraction: exact round trip");

    // The resize-proof property PICK relies on: a point's camera-space ray is
    // viewport-independent, and rayToFraction lands it on the pixel the
    // renderer rasterizes that point to -- at whichever aspect is live.
    bool resizeProof = true;
    for (const glm::vec3 &p : kPoints) {
        const glm::vec2 ray = cameraRay(camera, p);
        for (const Viewport &vp : { landscape, portrait }) {
            const glm::vec2 pixel = rayToFraction(ray, camera.fov, vp.aspect())
                                  * glm::vec2((float)vp.width, (float)vp.height);
            resizeProof = resizeProof
                       && glm::length(pixel - rasterize(camera, vp, p)) < 0.1f;
        }
    }
    check(resizeProof, "a stored ray lands on its point's pixel at both aspects");
}
