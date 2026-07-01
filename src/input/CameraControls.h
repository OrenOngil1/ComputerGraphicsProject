#pragma once

#include "../core/Camera.h"

// Camera controls: the pure transforms that move a camera, decoupled from the input
// device. The GLFW glue that gathers input (pollMovementIntent, the callbacks) lives
// in the input layer (Callbacks.{h,cpp}); everything here is window-free so it can be
// unit-tested headlessly (see tests/headless_checks.cpp).

// Device-neutral "what the user wants" for the fly (player) camera: signed axes summed
// from opposing keys (+1 / 0 / -1). Built by pollMovementIntent (GLFW glue) and consumed
// by fly(); being a plain struct is what lets fly() be tested without a window.
struct MovementIntent {
    int forward  = 0;   // W / S       -> along the look direction
    int strafe   = 0;   // D / A       -> along the right axis
    int vertical = 0;   // Q / E       -> along world up
    int yaw      = 0;   // RIGHT / LEFT
    int pitch    = 0;   // UP / DOWN
};

// ── Pure camera verbs (GLFW-free, unit-testable) ──────────────
// Each mutates the camera it is handed. zoom/pan/orbit drive an overview (look-at)
// camera whose target stays put or is orbited; fly drives the free-look player camera.
// The interaction constants and the eye-target-distance invariants live in the .cpp.

// Scroll-wheel zoom: slide the eye along its view axis, target fixed, clamped so it can
// neither cross the target nor fly off to infinity.
void zoom(Camera &cam, double scrollY);

// Middle-drag pan: shift eye AND target together in the view plane (grab-the-map feel),
// scaled by the eye-target distance so it feels the same at any zoom.
void pan(Camera &cam, double dx, double dy);

// Right-drag orbit: swing the eye around the fixed target (yaw around world up, pitch
// while the view stays clear of the vertical), so the far side of the terrain comes
// into view without moving what the camera aims at.
void orbit(Camera &cam, double dx, double dy);

// Free-fly (FPS-style) motion of the player camera for one frame: look (yaw/pitch,
// pitch clamped near vertical), then translate eye + target together, scaled by dt so
// it is frame-rate independent. terrainSize scales the move speed across DEMs.
void fly(Camera &cam, const MovementIntent &in, float terrainSize, float dt);

// ── OrbitController ───────────────────────────────────────────
// Owns the middle/right-drag lifecycle for an overview camera and delegates the motion
// to pan/orbit/zoom above. Holds only the drag anchors -- no GLFW -- so it stays
// unit-testable: begin a drag, feed cursor positions, assert the camera moved. Lives in
// the input layer (a CallbackContext member), replacing the loose panning/rotating/
// lastDrag fields that used to sit there. Only one of pan/orbit is active at a time.
struct OrbitController {
    bool   panning  = false;
    bool   rotating = false;
    double lastX = 0.0;
    double lastY = 0.0;

    void beginPan  (double x, double y) { panning = true;  rotating = false; lastX = x; lastY = y; }
    void beginOrbit(double x, double y) { rotating = true; panning = false; lastX = x; lastY = y; }
    void end() { panning = false; rotating = false; }
    bool dragging() const { return panning || rotating; }

    // Continuous motion, called on every cursor move; a no-op unless a drag is active.
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
