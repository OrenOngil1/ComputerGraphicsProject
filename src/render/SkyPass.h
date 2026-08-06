#pragma once

#include <array>
#include <string>

#include <Shader.h>

#include "GpuMesh.h"
#include "../core/Camera.h"
#include "../core/Lighting.h"

// RAII owner of one GL texture name (the vendored toolkit has no texture
// wrapper). Deletes the texture on destruction, which runs while the GL
// context is live -- its only holders sit inside Renderer (see Renderer's
// ownership note).
class GlTexture {
public:
    GlTexture() = default;                              // empty: no texture
    explicit GlTexture(unsigned int id) : m_id(id) {}   // adopts the name

    // Owns a raw GL id -- copying would double-free it.
    GlTexture(const GlTexture &) = delete;
    GlTexture &operator=(const GlTexture &) = delete;
    GlTexture(GlTexture &&other) noexcept;
    GlTexture &operator=(GlTexture &&other) noexcept;
    ~GlTexture();

    unsigned int id() const { return m_id; }   // 0 = none

private:
    unsigned int m_id = 0;
};

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
