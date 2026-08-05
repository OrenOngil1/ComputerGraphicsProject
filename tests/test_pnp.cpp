// Headless checks for the PnP solvers -- no window, no GL context.

#include <cmath>
#include <iostream>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "check.h"
#include "../src/vision/Pnp.h"

namespace {

const glm::vec3 kEye(10.0f, 8.0f, 20.0f);
const glm::vec3 kTarget(0.0f, 0.0f, 0.0f);
const float kFov = 45.0f;
const int kWidth = 800, kHeight = 600;

// Project a world point through a glm camera into image coordinates (origin
// top-left, y down) -- the convention every mode feeds the solvers.
// Deliberately independent of getCameraIntrinsicMatrix: this is the square-
// pixel pinhole glm::perspective actually renders (fx == fy, aspect carried by
// width != height), so the round-trip validates the solver's intrinsics
// against an outside standard. The viewport is deliberately non-square: a
// width/height factor wrongly folded into fx would shift these projections
// and fail the round-trip.
glm::vec2 projectToPixel(const glm::mat4 &view, const glm::vec3 &p)
{
    // glm::lookAt yields the GL camera frame (x right, y up, looking down -z);
    // the image frame has y down and z forward, so y and z flip sign.
    const glm::vec3 pc = glm::vec3(view * glm::vec4(p, 1.0f));
    const float f = kHeight / (2.0f * std::tan(glm::radians(kFov) / 2.0f));
    return { f * (pc.x / -pc.z) + kWidth  / 2.0f,
             f * (-pc.y / -pc.z) + kHeight / 2.0f };
}

// A solver that round-trips synthetic projections back to the camera that made
// them has its conventions (intrinsics, axis flips, inversion) right.
bool poseMatches(const std::optional<Waypoint> &pose)
{
    if (!pose)
        return false;
    const glm::vec3 trueForward = glm::normalize(kTarget - kEye);
    const float positionError = glm::length(pose->position - kEye);
    const glm::vec3 forward = glm::normalize(pose->target - pose->position);
    return positionError < 0.05f && glm::dot(forward, trueForward) > 0.999f;
}

// Non-coplanar points spread through the camera's field of view.
const std::vector<glm::vec3> kPoints = {
    {  0,  0,  0 }, {  5,  1, -3 }, { -4,  2,  1 }, {  2, -1,  5 },
    { -3,  0, -5 }, {  6,  3,  2 }, { -5,  1,  4 }, {  1,  4, -2 },
};

// Correspondence::imagePos is a [0,1] viewport fraction, hence the divisions
// by frame size below.
std::vector<Correspondence> exactCorrespondences()
{
    const glm::mat4 view = glm::lookAt(kEye, kTarget, glm::vec3(0, 1, 0));
    const glm::vec2 frameSize((float)kWidth, (float)kHeight);
    std::vector<Correspondence> exact;
    for (const glm::vec3 &p : kPoints)
        exact.push_back({ p, projectToPixel(view, p) / frameSize });
    return exact;
}

// The same set with 3 of 11 pairs corrupted far beyond the inlier band (which
// defaults to kHandPlacedReprojFraction of the frame height, ~18 px here) --
// a plain least-squares solve would be dragged off; consensus must not be.
std::vector<Correspondence> correspondencesWithOutliers()
{
    const glm::mat4 view = glm::lookAt(kEye, kTarget, glm::vec3(0, 1, 0));
    const glm::vec2 frameSize((float)kWidth, (float)kHeight);
    std::vector<Correspondence> withOutliers = exactCorrespondences();

    withOutliers.push_back({ { 4, 2, 4 }, projectToPixel(view, { 4, 2, 4 }) / frameSize });
    withOutliers.push_back({ { -2, 3, 3 }, projectToPixel(view, { -2, 3, 3 }) / frameSize });
    withOutliers.push_back({ { 3, 1, -4 }, projectToPixel(view, { 3, 1, -4 }) / frameSize });

    // Corruptions are pixel offsets, normalized like the points themselves.
    withOutliers[2].imagePos += glm::vec2(150.0f, -120.0f) / frameSize;
    withOutliers[6].imagePos += glm::vec2(-200.0f, 90.0f) / frameSize;
    withOutliers[9].imagePos += glm::vec2(80.0f, 160.0f) / frameSize;

    return withOutliers;
}

}  // namespace

void testPnp()
{
    std::cout << "PnP solvers:" << std::endl;

    check(poseMatches(computeCameraPose(exactCorrespondences(), kFov, kWidth, kHeight)),
          "SQPnP: exact correspondences round-trip to the true pose");

    check(poseMatches(computeCameraPoseRansac(correspondencesWithOutliers(), kFov, kWidth, kHeight)),
          "RANSAC: recovers the true pose despite 3 wrong correspondences");

    // The same set has only 8 inliers; a caller demanding 20 must get a
    // refusal, not a confident but under-constrained pose.
    check(!computeCameraPoseRansac(correspondencesWithOutliers(), kFov, kWidth, kHeight, 20).has_value(),
          "RANSAC: refuses when inliers fall short of the caller's minimum");
}
