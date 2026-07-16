#pragma once

#include <glm/glm.hpp>

struct Simulation;
class Renderer;
struct GLFWwindow;   // only a pointer is needed here -- keep GLFW out of this header

// One State object per mode, encapsulating the two things that vary by mode:
// how it reacts to input, and what it draws as an overlay. States hold no
// shared scene data (that arrives via Simulation&), only mode-local scratch.
//
// Deliberately no mode/id() tag -- that would invite a switch(state->id()).
// Transitions are an app-level concern handled in Callbacks.cpp (tryTransition),
// not by the states themselves.
class State {
public:
    virtual ~State() = default;

    // Entry action, run by setState once the state is current (after the
    // transition guard has passed). No onExit: the destructor covers cleanup.
    virtual void onEnter(Simulation &) {}

    // Discrete key events (press/repeat), routed from keyCallback. Gets the
    // Renderer because a mode may trigger a render pass in response (the
    // capture modes render on 'B').
    virtual void handleKey(Simulation &, Renderer &, int /*key*/, int /*mods*/) {}

    // Per-frame step; dt = seconds since the last frame. The moving modes poll
    // held keys here (via fly) to move the player camera.
    virtual void tick(Simulation &, GLFWwindow *, float /*dt*/) {}

    // Discrete mouse-button events, routed from mouseButtonCallback. Gets the
    // Renderer because PICK runs the color-pick pass in response.
    virtual void handleMouseButton(Simulation &, Renderer &, GLFWwindow *,
                                   int /*button*/, int /*action*/) {}

    // Per-view overlays; a mode overrides only the view(s) it decorates. Drawn
    // *through* the Renderer, so the shaders never leave their owner.
    virtual void renderGlobalOverlay(const Simulation &, Renderer &, const glm::mat4 &) const {}
    virtual void renderPlayerOverlay(const Simulation &, Renderer &, const glm::mat4 &) const {}
};
