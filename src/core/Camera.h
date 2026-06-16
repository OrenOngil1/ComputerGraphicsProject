#pragma once

#include <glm/glm.hpp>

// A camera answers "what does the world look like from here?" -- purely the
// eye's pose and lens. It carries no notion of where on screen it is drawn;
// that is the Viewport's job. Keeping these apart lets the camera be passed
// as read-only (const&) data: the renderer never has to mutate it.
struct Camera {
    glm::vec3 position;
    glm::vec3 target;
    glm::vec3 up;

    float fov;
    float near;
    float far;
};

// A viewport answers "which rectangle of the window do I paint into?" -- pure
// render configuration. Recomputed on resize (see leftHalf/rightHalf below) and
// stored inside the View it belongs to.
struct Viewport {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

// A View pairs an eye (Camera) with the screen rectangle it paints into
// (Viewport). The two always travel together; bundling them into one object
// prevents handing a camera one viewport and accidentally drawing it into
// another, and gives the resize callback a single place to update the layout.
struct View {
    Camera   camera;
    Viewport viewport;
};

// Window-layout helpers: the two viewports are pure functions of the current
// framebuffer size. They live here (next to Viewport) rather than in Renderer.h
// because they're pure layout with no renderer dependency -- the resize callback
// computes layout without pulling in the renderer. `inline` lets the definitions
// sit in this widely-included header without an ODR/multiple-definition link error.
inline Viewport leftHalf(int windowWidth, int windowHeight)
{
    return Viewport{ 0, 0, windowWidth / 2, windowHeight };
}

inline Viewport rightHalf(int windowWidth, int windowHeight)
{
    return Viewport{ windowWidth / 2, 0, windowWidth / 2, windowHeight };
}

// A recorded camera pose (position + look-at target) -- a waypoint along the
// flight path, captured in RECORD mode for later playback.
struct Waypoint {
    glm::vec3 position;
    glm::vec3 target;
};

// Put a camera on a recorded pose. A Waypoint records WHERE, not what lens:
// fov/near/far/up keep their current values. The one place the two-field copy
// lives -- playback, pick seeding, timestep review, ghosts, and the feature
// pre-phase all snap cameras to waypoints, and they must all mean the same
// thing by it.
inline void applyPose(Camera &camera, const Waypoint &waypoint)
{
    camera.position = waypoint.position;
    camera.target   = waypoint.target;
}