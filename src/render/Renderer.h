#pragma once

#include <memory>
#include <string>

#include <Shader.h>
#include <VertexArray.h>
#include <VertexBuffer.h>
#include <IndexBuffer.h>

#include "../core/AppState.h"

// GPU-side terrain buffers. Owned by Renderer and rebuilt whenever the terrain
// is swapped (e.g. the menu loading a different DEM).
struct TerrainGpu {
    std::unique_ptr<VertexArray> va;
    std::unique_ptr<VertexBuffer> vb;
    std::unique_ptr<IndexBuffer> ib;
    unsigned int indexCount = 0;
};

// Window-layout helpers: the two viewports are pure functions of the current
// window size, so we compute them on demand each frame rather than storing them.
// They have no dependency on renderer state, so they stay free functions.
Viewport leftHalf(int windowWidth, int windowHeight);
Viewport rightHalf(int windowWidth, int windowHeight);

// Owns the GPU resources needed to draw the scene (the scene shader plus the
// terrain buffers) and exposes a single per-view draw call.
//
// One concrete class, no virtuals: there is exactly one way to draw the scene;
// only the overlay varies, and it is selected per call. The real win is
// lifetime safety -- as a stack object whose destructor runs while the GL
// context is still alive, a Renderer replaces the hand-managed { } scope that
// used to guard Shader/TerrainGpu in main(). It also gives the upcoming menu a
// cheap terrain swap (loadTerrain) and leaves a home for Mode-2 picking/ghost.
class Renderer {
public:
    explicit Renderer(const Mesh &terrain);   // compile scene shader + upload terrain

    // Owns a Shader (raw GL program id, no safe copy), so the Renderer is
    // non-copyable. It is constructed once on the stack and never duplicated.
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;

    void clear() const;                        // glClear(COLOR | DEPTH)

    // Menu swap: drop the old terrain buffers and upload a new mesh. The already
    // compiled shader is reused, so changing terrains never recompiles GLSL.
    void loadTerrain(const Mesh &terrain);

    // Paint one viewport: draw the terrain, then let the active mode decorate it.
    // Two methods rather than a flag parameter, so the overlay can never be
    // mismatched with the view -- the global view draws the global overlay, the
    // player view draws the player overlay.
    void renderGlobalView(const Camera &camera, const Viewport &viewport, const AppState &appState);
    void renderPlayerView(const Camera &camera, const Viewport &viewport, const AppState &appState);

private:
    // Shared by both views: set the viewport, build the MVP, draw the terrain;
    // returns the MVP so the caller can hand it to the active mode's overlay.
    glm::mat4 renderScene(const Camera &camera, const Viewport &viewport);

    Shader     m_sceneShader;
    TerrainGpu m_terrain;
    // SEAM (Mode 2, not implemented yet): a second m_pickShader + a pickVertex()
    // offscreen pass for color picking, and a ghost-terrain path (uniform
    // color/alpha on m_sceneShader). Both fit here without changing this surface.
};
