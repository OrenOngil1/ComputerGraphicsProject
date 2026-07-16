#pragma once

#include <array>
#include <string>

#include <Shader.h>

#include "GpuMesh.h"
#include "../core/Camera.h"
#include "../core/Lighting.h"

// The sky pass: one cubemap skybox per light preset, drawn behind the terrain.
// The one home of manually managed GL texture lifetime -- everything else in
// src/ sits behind a vendored RAII wrapper.
class SkyPass {
public:
    // Compiles the skybox shader and builds the cube; needs a live GL context.
    // skyboxRoot (with trailing slash) is prepended to the preset's name to
    // find its folder -- injected once, so no other render code composes
    // asset paths.
    explicit SkyPass(std::string skyboxRoot);

    // Non-copyable: owns raw GL texture ids.
    SkyPass(const SkyPass &) = delete;
    SkyPass &operator=(const SkyPass &) = delete;

    // Deletes the cubemap textures. Runs while the GL context is live
    // (SkyPass is a Renderer member; see Renderer's ownership note).
    ~SkyPass();

    // Draw the preset's skybox over the clear-color background: lazily
    // loads/uploads the cubemap on first use (a failed load warns once and
    // disables that slot). Depth 1.0 + the app's GL_LEQUAL confine it to
    // pixels the terrain left untouched.
    void draw(size_t presetIdx, const Camera &camera, const Viewport &viewport);

private:
    // Lazy per-preset cubemap cache: GL texture names, filled on first draw.
    // tried with id still 0 marks a failed load, so it isn't retried per frame.
    struct SkySlot {
        unsigned int id = 0;    // GL cubemap texture name; 0 = none
        bool tried = false;
    };

    Shader      m_shader;   // cubemap sampler behind the terrain
    GpuMesh     m_cube;     // position-only unit cube, built once
    std::string m_root;     // base dir of the per-preset skybox folders
    std::array<SkySlot, kLightPresetCount> m_slots;
};
