#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Viewport.h"

// A recorded camera pose (position + look-at target) -- a waypoint along the
// flight path, captured in RECORD mode for later playback.
struct Waypoint {
    glm::vec3 position;   // centered world space
    glm::vec3 target;

    // Exact float equality: two poses match only when one was copied from the
    // other (applyPose / pose()), never when recomputed.
    bool operator==(const Waypoint &other) const
    {
        return position == other.position && target == other.target;
    }
    bool operator!=(const Waypoint &other) const { return !(*this == other); }
};

// An eye's pose and lens -- "what does the world look like from here?". It
// carries no notion of where on screen it is drawn; that is the Viewport's
// job, which lets the renderer take cameras as read-only data.
struct Camera {
    glm::vec3 position;   // centered world space
    glm::vec3 target;     // the point looked at
    glm::vec3 up;

    float fov;    // VERTICAL field of view, degrees (glm::perspective convention)
    float near;   // clip planes, world units
    float far;

    // Put the camera on a recorded pose. A Waypoint records WHERE, not what
    // lens: fov/near/far/up keep their current values. The one definition of
    // this two-field copy -- playback, pick seeding, timestep review, and
    // ghosts must all mean the same thing by it.
    void applyPose(const Waypoint &waypoint)
    {
        position = waypoint.position;
        target   = waypoint.target;
    }

    // The current pose as a Waypoint -- applyPose's outward counterpart.
    Waypoint pose() const { return { position, target }; }
};

// An eye (Camera) paired with the screen rectangle it paints into (Viewport).
// Bundled so a camera can't be drawn into the wrong viewport, and resize has
// one place to update.
struct View {
    Camera   camera;
    Viewport viewport;
};

// The projection * view matrix a (camera, viewport) pair renders with:
// vertical FOV and clip planes from the camera, aspect from the viewport, no
// model part. The one definition of the app's camera model -- the pinhole
// intrinsics in vision/Pnp.cpp describe this same camera, and the headless
// camera-model check holds the two to it.
inline glm::mat4 viewProjection(const Camera &camera, const Viewport &viewport)
{
    glm::mat4 proj = glm::perspective(glm::radians(camera.fov), viewport.aspect(),
                                      camera.near, camera.far);
    glm::mat4 view = glm::lookAt(camera.position, camera.target, camera.up);
    return proj * view;
}
