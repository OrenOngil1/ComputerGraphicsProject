#pragma once

#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

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
    void renderGlobalView(const View &view, const AppState &appState);
    void renderPlayerView(const View &view, const AppState &appState);

    // Overlay drawing surface: a mode's render*Overlay draws *through* the Renderer
    // (it is handed `*this`, not a Shader), so m_sceneShader never leaves its owner.
    // Throwaway buffers each call -- overlay geometry changes every frame.
    void drawPath(const std::vector<glm::vec3> &pathPoints, const glm::mat4 &mvp);
    void drawWaypoints(const std::vector<Waypoint> &waypoints,
                       const glm::vec3 &cameraPos, const glm::mat4 &mvp);

private:
    // Shared by both views: set the viewport, build the MVP, draw the terrain;
    // returns the MVP so the caller can hand it to the active mode's overlay.
    glm::mat4 renderScene(const Camera &camera, const Viewport &viewport);

    Shader     m_sceneShader;
    TerrainGpu m_terrain;
    // SEAM (Mode 2, not implemented yet): a ghost overlay (drawGhost, uniform
    // color/alpha on m_sceneShader -- drawn through the surface above) and an
    // offscreen color-pick pass (pickAt(x,y) using a private m_pickShader + an
    // FBO + glReadPixels). Neither passes a shader out. The pick pass is a distinct
    // responsibility -- extract it to a ColorPicker/PickPass when it lands, not here.
};
