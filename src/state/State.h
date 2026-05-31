#pragma once

#include <glm/glm.hpp>

struct AppState;
class Shader;

// One State object per mode. It encapsulates the two things that vary by mode:
// how the mode reacts to input, and what it draws as an overlay. States hold no
// shared scene data -- that is passed in via AppState& -- they own only
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
    // e.g. RECORD clearing the recording, PLAYBACK snapping to the first record.
    // Default: nothing. (No onExit: a state's destructor covers what it owns.)
    virtual void onEnter(AppState &) {}

    virtual void handleKey(AppState &appState, int key, int mods) = 0;

    // Default to drawing nothing; a mode overrides only the view(s) it decorates.
    virtual void renderGlobalOverlay(const AppState &, Shader &, const glm::mat4 &) const {}
    virtual void renderPlayerOverlay(const AppState &, Shader &, const glm::mat4 &) const {}
};
