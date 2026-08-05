// Headless checks for snapToViewRay -- the correction applied to every
// hand-placed anchor in Mode D. No window, no GL context.
//
// The claim being tested is the reason the correction exists: when the camera
// pose and the observed pixel are both known exactly, the observed point is
// somewhere on that viewing ray, so the across-the-ray part of a human's click
// is error and nothing else. Moving the click onto the ray must therefore land
// it on the ray, leave it reprojecting onto the pixel it came from, and put it
// strictly closer to the truth -- never further.

#include <cmath>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "check.h"
#include "../src/core/Camera.h"

namespace {

const Camera kCamera{ { 40, 60, 120 }, { 0, 5, 0 }, { 0, 1, 0 }, 50.0f, 0.1f, 2000.0f };
const Viewport kViewport{ 0, 0, 400, 600 };

}  // namespace

void testRayAnchor()
{
    std::cout << "snapToViewRay (hand-placed anchor correction):" << std::endl;

    // One suggestion, off-center so a y- or x-sign slip would show. The truth
    // is a point on its ray; the "click" is that truth pushed sideways (across
    // the ray) and a long way along it -- a plausible human miss, since depth
    // is the hard part to judge and sideways is the easy part to get slightly
    // wrong.
    const glm::vec2 ray = fractionToRay(glm::vec2(0.68f, 0.34f), kCamera.fov,
                                        kViewport.aspect());
    const glm::vec3 direction = rayDirection(kCamera, ray);
    const glm::vec3 truth = kCamera.position + direction * 150.0f;

    const glm::vec3 across = glm::normalize(glm::cross(direction, glm::vec3(0, 1, 0)));
    const glm::vec3 click = truth + across * 25.0f + direction * 40.0f;

    const std::optional<glm::vec3> snapped = snapToViewRay(kCamera, ray, click);
    check(snapped.has_value(), "a click in front of the camera resolves");
    if (!snapped)
        return;

    // On the ray: the offset from the camera is parallel to the direction.
    check(glm::length(glm::cross(*snapped - kCamera.position, direction)) < 1e-2f,
          "the snapped anchor lies on the suggestion's viewing ray");

    // Strictly closer to the truth. The click's error was the hypotenuse of a
    // right triangle; what survives is one leg.
    check(glm::length(*snapped - truth) < glm::length(click - truth) - 1e-3f,
          "the snapped anchor is strictly closer to the true 3D point");

    // Only the depth error survives -- the sideways component is gone.
    check(std::fabs(glm::length(*snapped - truth) - 40.0f) < 1e-2f,
          "what remains is exactly the depth part of the miss");

    // The point of the whole exercise: the anchor now reprojects onto the
    // pixel its descriptor was taken from, so the correspondence agrees with
    // itself and RANSAC has no reason to throw it out.
    const glm::vec2 sourcePixel = glm::vec2(0.68f, 0.34f) *
                                  glm::vec2((float)kViewport.width, (float)kViewport.height);
    check(glm::length(rasterize(kCamera, kViewport, *snapped) - sourcePixel) < 0.1f,
          "and it reprojects onto the pixel it was placed for");
    check(glm::length(rasterize(kCamera, kViewport, click) - sourcePixel) > 20.0f,
          "whereas the raw click reprojects tens of pixels away (the outlier)");

    // A click already on the ray is already right: the snap must not move it.
    const std::optional<glm::vec3> exact = snapToViewRay(kCamera, ray, truth);
    check(exact && glm::length(*exact - truth) < 1e-3f,
          "a click already on the ray is left where it is");

    // Behind the camera: nothing the view saw can be there, so it is refused
    // rather than folded to the wrong side of the eye.
    check(!snapToViewRay(kCamera, ray, kCamera.position - direction * 30.0f),
          "a point behind the view is refused");
    check(!snapToViewRay(kCamera, ray, kCamera.position),
          "and so is the camera's own position (zero depth)");
}
