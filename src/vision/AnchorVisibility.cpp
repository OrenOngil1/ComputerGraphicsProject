#include "AnchorVisibility.h"

#include <algorithm>
#include <optional>

bool anchorVisibleFrom(const Mesh &mesh, float terrainSize, const Camera &camera,
                       const Viewport &viewport, const glm::vec3 &anchor)
{
    // Judge the surface feature the anchor DENOTES, not the stored point: the
    // snap can leave a point under the surface, and a buried one reads "hidden"
    // from every camera alike -- true pose and garbage both -- so its verdict
    // carries no information. Only the question is lifted; the 3D never moves.
    glm::vec3 feature = anchor;
    const std::optional<float> ground = mesh.heightAt(anchor.x, anchor.z);
    if (ground && *ground > feature.y)
        feature.y = *ground;

    if (!isInFrame(camera, viewport, feature))
        return false;

    const glm::vec3 toFeature = feature - camera.position;
    const float distance = glm::length(toFeature);

    // Stop short of the feature: a hit at its own distance is the ground it
    // belongs to, not something standing in front of it.
    const float margin = std::max(1.0f, terrainSize / 100.0f);
    const float step   = std::max(0.5f, terrainSize / 300.0f);
    if (distance <= margin)
        return true;

    return !raycastTerrain(mesh, camera.position, toFeature / distance,
                           distance - margin, step);
}
