#pragma once

#include <glm/glm.hpp>

// A recorded camera pose (position + look-at target) -- a waypoint along the
// flight path, captured in RECORD mode for later playback.
struct Waypoint {
    glm::vec3 position;   // centered world space
    glm::vec3 target;
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
};

// The window rectangle a view paints into. Recomputed on resize (see
// leftHalf/rightHalf below).
struct Viewport {
    int x = 0;        // lower-left corner + size, in framebuffer PIXELS
    int y = 0;
    int width = 0;
    int height = 0;

    // Hit-test a pixel. Half-open on the far edges, so a pixel never counts as
    // inside two abutting viewports (the split shares the column at x + width).
    // Cursor coords arrive as doubles from GLFW, hence the parameter type.
    bool contains(double px, double py) const
    {
        return px >= x && px < x + width &&
               py >= y && py < y + height;
    }

    // Width-to-height ratio -- the factor mapping the vertical FOV to the
    // horizontal one in projections and viewing rays.
    float aspect() const { return (float)width / (float)height; }
};

// An eye (Camera) paired with the screen rectangle it paints into (Viewport).
// Bundled so a camera can't be drawn into the wrong viewport, and resize has
// one place to update.
struct View {
    Camera   camera;
    Viewport viewport;
};

// The split-screen layout as pure functions of the framebuffer size. Here
// (next to Viewport) rather than in the renderer so the resize callback
// computes layout without a renderer dependency; `inline` because this header
// is widely included.
inline Viewport leftHalf(int windowWidth, int windowHeight)
{
    return Viewport{ 0, 0, windowWidth / 2, windowHeight };
}

inline Viewport rightHalf(int windowWidth, int windowHeight)
{
    return Viewport{ windowWidth / 2, 0, windowWidth / 2, windowHeight };
}
