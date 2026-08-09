#include "PosePlausibility.h"

#include <algorithm>
#include <iostream>
#include <optional>

#include "AnchorVisibility.h"

// How far from the origin-centered map an estimate may sit, in terrain widths.
static bool withinTerrainReach(float terrainSize, const Waypoint &estimate)
{
    constexpr float kPlausibleDistanceWidths = 2.0f;

    const float distance = glm::length(estimate.position);
    const float limit    = kPlausibleDistanceWidths * terrainSize;
    if (distance <= limit)
        return true;

    std::cout << "FEATURES: pose rejected as implausible -- it puts the camera "
              << (int)distance << " units from the terrain (nothing seeing this"
                 " terrain can be past " << (int)limit << ")" << std::endl;
    return false;
}

// Under the map is not a pose a camera can hold. heightAt only answers over the
// grid, so an estimate past the edge is left to the other checks.
static bool sitsAboveSurface(const Mesh &mesh, float terrainSize, const Waypoint &estimate)
{
    const std::optional<float> ground =
        mesh.heightAt(estimate.position.x, estimate.position.z);
    if (!ground || estimate.position.y >= *ground + std::max(1.0f, terrainSize / 100.0f))
        return true;

    std::cout << "FEATURES: pose rejected as implausible -- it puts the camera"
                 " under the terrain" << std::endl;
    return false;
}

// Near the terrain is not the same as looking at it. Probe the lower-third
// centre -- terrain lives in a frame's lower half, so a pitched-up view passes.
static bool looksAtTerrain(const Mesh &mesh, float terrainSize, const Camera &estimated,
                           const Viewport &vp)
{
    const glm::vec3 probe = rayDirection(
        estimated, fractionToRay(glm::vec2(0.5f, 0.75f), estimated.fov, vp.aspect()));
    if (raycastTerrain(mesh, estimated.position, probe, 3.0f * terrainSize,
                       std::max(1.0f, terrainSize / 200.0f)))
        return true;

    std::cout << "FEATURES: pose rejected as implausible -- a camera there, aimed"
                 " that way, would be looking at no terrain at all" << std::endl;
    return false;
}

// The coalition killer: the estimate must SEE its own evidence. The allowance
// scales with the consensus -- refusing a 25-inlier pose over two anchors noise
// buried under a slope threw away good captures, so one hidden in eight is
// placement noise and more is a coalition.
static bool seesItsOwnInliers(const Mesh &mesh, float terrainSize, const Camera &estimated,
                              const Viewport &vp,
                              const std::vector<Correspondence> &inliers)
{
    size_t hidden = 0;
    for (const Correspondence &inlier : inliers)
        if (!anchorVisibleFrom(mesh, terrainSize, estimated, vp, inlier.worldPos))
            hidden++;
    if (hidden <= std::max<size_t>(1, inliers.size() / 8))
        return true;

    std::cout << "FEATURES: pose rejected as implausible -- " << hidden << " of its "
              << inliers.size() << " agreeing anchors would be hidden behind"
                 " ridges from there" << std::endl;
    return false;
}

bool poseIsPlausible(const Mesh &mesh, float terrainSize, const Camera &prototype,
                     const Waypoint &estimate, const Viewport &vp,
                     const std::vector<Correspondence> &inliers)
{
    Camera estimated = prototype;
    estimated.applyPose(estimate);

    return withinTerrainReach(terrainSize, estimate)
        && sitsAboveSurface(mesh, terrainSize, estimate)
        && looksAtTerrain(mesh, terrainSize, estimated, vp)
        && seesItsOwnInliers(mesh, terrainSize, estimated, vp, inliers);
}
