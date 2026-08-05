#pragma once

#include <cmath>
#include <optional>

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

// viewProjection with the view's translation stripped (rotation-only view):
// the sky pass renders directions, not positions, so the viewer is pinned to
// the origin and the skybox can never be approached or left behind. The
// mat3 round-trip keeps the rotation block and zeroes the translation column.
inline glm::mat4 skyViewProjection(const Camera &camera, const Viewport &viewport)
{
    glm::mat4 proj = glm::perspective(glm::radians(camera.fov), viewport.aspect(),
                                      camera.near, camera.far);
    glm::mat4 view = glm::lookAt(camera.position, camera.target, camera.up);
    return proj * glm::mat4(glm::mat3(view));
}

// Convert between a viewport fraction ([0,1] x [0,1], origin top-left) and the
// camera-space viewing ray (x/z, y/z), for this same camera model at a given
// vertical FOV (degrees) and aspect:
//   ray = ((u - 0.5)*2*tan(fov/2)*aspect, (v - 0.5)*2*tan(fov/2)).
// Only the horizontal term carries the aspect, which is why the ray form
// survives a resize and a stored fraction doesn't (see PickState::Observation).
inline glm::vec2 fractionToRay(glm::vec2 uv, float fovDeg, float aspect)
{
    const float t = std::tan(glm::radians(fovDeg) * 0.5f);
    return { (uv.x - 0.5f) * 2.0f * t * aspect, (uv.y - 0.5f) * 2.0f * t };
}

inline glm::vec2 rayToFraction(glm::vec2 ray, float fovDeg, float aspect)
{
    const float t = std::tan(glm::radians(fovDeg) * 0.5f);
    return { 0.5f + ray.x / (2.0f * t * aspect), 0.5f + ray.y / (2.0f * t) };
}

// The camera's orthonormal frame. `up` is only a hint (as in glm::lookAt): the
// true up is re-derived from forward x right, so a non-perpendicular hint still
// yields the same frame the renderer projects with.
struct CameraBasis {
    glm::vec3 forward;   // toward the target
    glm::vec3 right;
    glm::vec3 up;
};

inline CameraBasis cameraBasis(const Camera &camera)
{
    const glm::vec3 forward = glm::normalize(camera.target - camera.position);
    const glm::vec3 right   = glm::normalize(glm::cross(forward, camera.up));
    return { forward, right, glm::cross(right, forward) };
}

// The world-space direction of a camera-space viewing ray (x/z, y/z) -- the
// inverse of projecting a world point onto a pixel, and the one thing an image
// observation actually determines. The 3D point behind a pixel lies somewhere
// along position + t * rayDirection(...); WHERE along it is the depth this
// camera model cannot recover from one view, which is exactly what PnP solves
// for and what the manual modes ask the user to supply.
//
// The `- up * ray.y` is the image convention (y grows downward, see
// fractionToRay): the top of the frame is +up in the world.
inline glm::vec3 rayDirection(const Camera &camera, glm::vec2 ray)
{
    const CameraBasis basis = cameraBasis(camera);
    return glm::normalize(basis.forward + basis.right * ray.x - basis.up * ray.y);
}

// Is this world point inside what the camera draws into `viewport`?
//
// Tested against the clip volume of the very matrix the renderer projects with,
// so "in frame" here and "on screen" there cannot disagree.
//
// Frustum only: a point behind a ridge is still in frame. Callers that need
// "actually visible" pair this with raycastTerrain (see Scene.h).
inline bool isInFrame(const Camera &camera, const Viewport &viewport, const glm::vec3 &point)
{
    const glm::vec4 clip = viewProjection(camera, viewport) * glm::vec4(point, 1.0f);
    if (clip.w <= 0.0f)
        return false;   // on or behind the eye plane; nothing there is drawn

    return std::fabs(clip.x) <= clip.w && std::fabs(clip.y) <= clip.w &&
           std::fabs(clip.z) <= clip.w;
}

// Where a world point lands on screen, in image pixels (origin top-left, y
// down) -- the convention FramePixels and Correspondence both use, and the
// forward direction of rayDirection below.
//
// Only meaningful for a point isInFrame accepts: behind the eye the perspective
// divide mirrors the result back into the frame rather than reporting an error.
inline glm::vec2 rasterize(const Camera &camera, const Viewport &viewport,
                           const glm::vec3 &point)
{
    const glm::vec4 clip = viewProjection(camera, viewport) * glm::vec4(point, 1.0f);
    const glm::vec3 ndc = glm::vec3(clip) / clip.w;
    return { (ndc.x * 0.5f + 0.5f) * (float)viewport.width,
             (1.0f - (ndc.y * 0.5f + 0.5f)) * (float)viewport.height };
}

// Move a guessed 3D point onto the viewing ray it was guessed for -- the point
// of that ray closest to it.
//
// The use is manual anchoring. When both the camera pose and the observed pixel
// are known exactly, the observed point is on that ray; the only thing a person
// is really being asked for is how FAR along it. So the sideways part of their
// pick is error, not information, and dropping it moves the guess strictly
// closer to the truth -- what remains is one leg of the right triangle their
// error was the hypotenuse of.
//
// It is also the half that matters. Error along the ray reprojects onto the
// same pixel and barely disturbs a pose solve; error across it reprojects
// proportionally to the focal length, which is what turns a correct
// correspondence into a rejected outlier.
//
// nullopt if the point resolves behind the camera, where nothing it saw can be.
inline std::optional<glm::vec3> snapToViewRay(const Camera &camera, glm::vec2 ray,
                                              const glm::vec3 &point)
{
    const glm::vec3 direction = rayDirection(camera, ray);
    const float depth = glm::dot(point - camera.position, direction);
    if (depth <= 0.0f)
        return std::nullopt;
    return camera.position + direction * depth;
}
