#pragma once

#include <glm/glm.hpp>

struct Simulation;
class Renderer;
struct GLFWwindow;   // only a pointer is needed here -- keep GLFW out of this header

// One State object per mode. It encapsulates the two things that vary by mode:
// how the mode reacts to input, and what it draws as an overlay. States hold no
// shared scene data -- that is passed in via Simulation& -- they own only
// mode-local scratch (e.g. a future PlaybackState's index).
//
// There is deliberately no mode/id() tag: that would be a type label inviting a
// switch(state->id()), reintroducing exactly what this pattern removes. "Which
// modes exist" is answered by "which State subclasses exist". Transitions are a
// global, app-level concern handled by the context, not this interface (see
// tryTransition in Callbacks.cpp). See docs/state-pattern-refactor.md.
class State {
public:
    virtual ~State() = default;

    // Entry action, run by setState once the state becomes current (after any
    // transition guard has already passed). Lets a mode set up shared state --
    // e.g. RECORD clearing the recording, PLAYBACK snapping to the first waypoint.
    // Default: nothing. (No onExit: a state's destructor covers what it owns.)
    virtual void onEnter(Simulation &) {}

    // Discrete key events (press/repeat), routed from keyCallback. Default: nothing,
    // so a mode with no discrete keys (e.g. NavigationState, now that arrows became
    // continuous "look") need not carry an empty override -- matching the other hooks.
    virtual void handleKey(Simulation &, int /*key*/, int /*mods*/) {}

    // Per-frame step, called once every frame from the main loop with dt = seconds
    // since the last frame (so time-based work is frame-rate independent). The
    // engine-standard update/tick, as opposed to handleKey's discrete events: here the
    // moving modes poll held keys (via moveCamera) to fly the player camera. Default:
    // nothing (e.g. Playback/Pick don't move).
    virtual void tick(Simulation &, GLFWwindow *, float /*dt*/) {}

    // Discrete mouse-button events, routed from mouseButtonCallback. Receives the
    // Renderer because a mode may need to trigger a render pass in response (PICK
    // runs the color-pick pass here). Default: nothing.
    virtual void handleMouseButton(Simulation &, Renderer &, GLFWwindow *,
                                   int /*button*/, int /*action*/) {}

    // Default to drawing nothing; a mode overrides only the view(s) it decorates.
    // The mode draws *through* the Renderer (it is handed a Renderer&, not a Shader),
    // so the scene shader never leaves its owner.
    virtual void renderGlobalOverlay(const Simulation &, Renderer &, const glm::mat4 &) const {}
    virtual void renderPlayerOverlay(const Simulation &, Renderer &, const glm::mat4 &) const {}
};
