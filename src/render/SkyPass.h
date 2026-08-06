#pragma once

#include <array>
#include <string>

#include <Shader.h>

#include "GlTexture.h"
#include "GpuMesh.h"
#include "../core/Camera.h"
#include "../core/Lighting.h"

// The sky pass: one cubemap skybox per light preset, drawn behind the terrain.
class SkyPass {
public:
    // Compiles the skybox shader and builds the cube; needs a live GL context.
    // skyboxRoot (with trailing slash) is prepended to the preset's name to
    // find its folder -- injected once, so no other render code composes
    // asset paths.
    explicit SkyPass(std::string skyboxRoot);

    // Non-copyable: owns GPU resources (shader, cube, cubemap cache).
    SkyPass(const SkyPass &) = delete;
    SkyPass &operator=(const SkyPass &) = delete;

    // Draw the preset's skybox over the clear-color background: lazily
    // loads/uploads the cubemap on first use (a failed load warns once and
    // disables that slot). Depth 1.0 + the app's GL_LEQUAL confine it to
    // pixels the terrain left untouched.
    void draw(size_t presetIdx, const Camera &camera, const Viewport &viewport);

private:
    // Lazy per-preset cubemap cache, filled on first draw. tried with an empty
    // texture marks a failed load, so it isn't retried per frame.
    struct SkySlot {
        GlTexture tex;          // the preset's cubemap; empty = none
        bool tried = false;
    };

    Shader      m_shader;   // cubemap sampler behind the terrain
    GpuMesh     m_cube;     // position-only unit cube, built once
    std::string m_root;     // base dir of the per-preset skybox folders
    std::array<SkySlot, kLightPresetCount> m_slots;
};
