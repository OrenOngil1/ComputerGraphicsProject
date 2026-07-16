#pragma once

#include "../core/Camera.h"

// Pure camera-motion math, decoupled from the input device: everything here is
// window-free and unit-tested headlessly (tests/test_camera_controls.cpp). The
// GLFW glue that gathers input lives in Callbacks.{h,cpp}.

// Device-neutral movement request for the fly (player) camera: signed axes
// summed from opposing keys (+1 / 0 / -1), built by pollMovementIntent.
struct MovementIntent {
    int forward  = 0;   // W / S       -> along the look direction
    int strafe   = 0;   // D / A       -> along the right axis
    int vertical = 0;   // Q / E       -> along world up
    int yaw      = 0;   // RIGHT / LEFT
    int pitch    = 0;   // UP / DOWN
};

// Each verb mutates the camera it is handed. zoom/pan/orbit drive an overview
// (look-at) camera; fly drives the free-look player camera. Interaction
// constants live in the .cpp.

// Scroll-wheel zoom: slide the eye along its view axis, target fixed, clamped
// so it can neither cross the target nor fly off to infinity.
void zoom(Camera &cam, double scrollY);

// Middle-drag pan: shift eye AND target together in the view plane, scaled by
// the eye-target distance so it feels the same at any zoom.
void pan(Camera &cam, double dx, double dy);

// Right-drag orbit: swing the eye around the fixed target (yaw around world
// up, pitch clamped clear of the vertical).
void orbit(Camera &cam, double dx, double dy);

// Free-fly (FPS-style) motion for one frame: look (yaw/pitch, pitch clamped
// near vertical), then translate eye + target together. dt-scaled for
// frame-rate independence; terrainSize scales the move speed across DEMs.
void fly(Camera &cam, const MovementIntent &in, float terrainSize, float dt);

// Owns the middle/right-drag lifecycle for an overview camera, delegating the
// motion to pan/orbit/zoom. Holds only the drag anchors -- no GLFW -- so it
// stays unit-testable. Only one of pan/orbit is active at a time.
struct OrbitController {
    bool   panning  = false;
    bool   rotating = false;
    double lastX = 0.0;   // cursor position at the last drag sample
    double lastY = 0.0;

    void beginPan  (double x, double y) { panning = true;  rotating = false; lastX = x; lastY = y; }
    void beginOrbit(double x, double y) { rotating = true; panning = false; lastX = x; lastY = y; }
    void end() { panning = false; rotating = false; }
    bool dragging() const { return panning || rotating; }

    // Continuous motion, called on every cursor move; no-op unless dragging.
    void drag(Camera &cam, double x, double y)
    {
        if (!dragging())
            return;
        const double dx = x - lastX;
        const double dy = y - lastY;
        if (panning)
            pan(cam, dx, dy);
        else
            orbit(cam, dx, dy);
        lastX = x;
        lastY = y;
    }

    void zoomBy(Camera &cam, double scrollY) { zoom(cam, scrollY); }
};
