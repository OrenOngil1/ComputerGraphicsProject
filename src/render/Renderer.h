#pragma once

#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include <Shader.h>
#include <VertexArray.h>
#include <VertexBuffer.h>
#include <IndexBuffer.h>

#include "../core/Simulation.h"

// A GPU-resident indexed mesh (VAO + VBO + IBO). Owned by Renderer: the
// terrain (rebuilt whenever the menu swaps DEMs) and the shared tracker
// sphere (built once) are both instances of this.
struct GpuMesh {
    std::unique_ptr<VertexArray> va;
    std::unique_ptr<VertexBuffer> vb;
    std::unique_ptr<IndexBuffer> ib;
    unsigned int indexCount = 0;    // fed to glDrawElements (GLsizei)
    size_t vertexCount = 0;         // pure count, for validating picked vertex ids
};

// Owns the GPU resources needed to draw the scene (the scene shader plus the
// terrain buffers) and exposes a single per-view draw call.
//
// One concrete class, no virtuals: there is exactly one way to draw the scene;
// only the overlay varies, and it is selected per call. The real win is
// lifetime safety -- owned as an Application member declared after the Window, so
// member-init order destroys it (glDelete*) while the GL context is still alive,
// without the hand-managed { } scope main() used to need. loadTerrain gives the
// menu a cheap in-place terrain swap and leaves a home for Mode-2 picking/ghost.
class Renderer {
public:
    // Compiles the shader programs only -- no terrain yet. Splitting "compile the
    // pipeline" (needs just a live context) from "upload a mesh" (mutable content,
    // via loadTerrain) lets a Renderer be a plain member constructed once, with
    // every terrain arriving through loadTerrain. Render paths assume a terrain has
    // been loaded; the session always calls loadTerrain before drawing.
    Renderer();

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
    void renderGlobalView(const View &view, const Simulation &sim);
    void renderPlayerView(const View &view, const Simulation &sim);

    // Overlay drawing surface: a mode's render*Overlay draws *through* the Renderer
    // (it is handed `*this`, not a Shader), so m_sceneShader never leaves its owner.
    // Throwaway buffers each call -- overlay geometry changes every frame.
    void drawPath(const std::vector<glm::vec3> &pathPoints, const glm::vec3 &color,
                  const glm::mat4 &mvp);
    void drawWaypoints(const std::vector<Waypoint> &waypoints,
                       const glm::vec3 &cameraPos, const glm::mat4 &mvp);

    // PICK mode (Mode 2): render the terrain with per-vertex id-colors (flat) and
    // read back the clicked pixel, returning the vertex id under (mouseX, mouseY) or
    // -1 on a miss. Draws into the back buffer and never swaps, so it isn't shown.
    int pickVertex(int mouseX, int mouseY, const View &playerView);

    // Redraw the terrain from an estimated pose in one translucent color, overlaid
    // without depth on the current view -- the PnP "ghost" for visual comparison.
    void drawGhost(const Camera &estimatedCamera, const Viewport &viewport,
                   const glm::vec3 &color, float alpha, float tintStrength);

    // Colored point markers (picked correspondence points, estimated camera, ...),
    // one color per position. Drawn on top of the scene (depth test off) so a marker
    // sitting on the terrain surface is never hidden behind it.
    void drawPoints(const std::vector<glm::vec3> &positions,
                    const std::vector<glm::vec3> &colors,
                    float size, const glm::mat4 &mvp);

    // TRACKERS mode (Mode 3): draw one tracker as the shared unit sphere, scaled
    // and translated into place, filled flat with the tracker's identifying color.
    // Unlike the markers above this draws WITH depth testing, like the terrain:
    // a tracker behind a hill is hidden -- and therefore absent from the capture
    // read-back, which is exactly the occlusion semantics auto-correspondence
    // wants. viewProj is the scene matrix the overlay was handed (no model part).
    void drawTracker(const Tracker &tracker, const glm::mat4 &viewProj);

    // TRACKERS capture pass: re-render the player view with the terrain flat
    // black and the trackers in their flat colors, and read the viewport back.
    // Any non-black pixel therefore belongs to a tracker, with occlusion already
    // resolved by the depth buffer. Like pickVertex, this draws into the back
    // buffer and never swaps, so the capture is invisible on screen.
    FramePixels captureTrackersFrame(const View &playerView,
                                     const std::vector<Tracker> &trackers);

    // FEATURE MATCH captures (Mode 4). Both render into the back buffer and
    // never swap, like pickVertex, so nothing shows on screen.
    //
    // The lit scene exactly as the player view would draw it, minus overlays:
    // the pixels feature detection runs on. Takes the light explicitly because
    // lighting is part of the Mode 4 experiment -- the same view captured
    // under a different preset must produce different pixels.
    FramePixels captureSceneFrame(const View &view, const DirectionalLight &light);

    // The same view through the pick shader, fully decoded: the terrain vertex
    // id under each pixel (image convention, index y * width + x, matching
    // FramePixels), or -1 where no terrain was hit. pickVertex's full-frame
    // twin -- it answers "which 3D point is this keypoint sitting on?" for
    // every pixel at once.
    std::vector<int> captureVertexIdFrame(const View &view);

private:
    // Shared by both views: set the viewport, build the MVP, draw the terrain
    // lit by the scene's sun; returns the MVP so the caller can hand it to the
    // active mode's overlay.
    glm::mat4 renderScene(const Camera &camera, const Viewport &viewport,
                          const DirectionalLight &light);

    // Read the viewport's back-buffer pixels into image-convention RGB (rows
    // flipped to top-down, packing alignment forced tight). The shared tail of
    // every full-frame capture.
    FramePixels readViewportPixels(const Viewport &viewport);

    Shader     m_sceneShader;
    // Flat per-vertex-id program for the color-pick pass (pickVertex). Kept here
    // for now; if the pick pass grows (FBO, depth read-back), extract it to a
    // dedicated ColorPicker/PickPass rather than expanding the Renderer.
    Shader     m_pickShader;
    // Disc-carving program for GL_POINTS markers (drawPoints / drawWaypoints):
    // discards the sprite corners so points render round, not square -- the
    // Core-profile replacement for the deprecated GL_POINT_SMOOTH.
    Shader     m_pointShader;
    GpuMesh    m_terrain;
    // The one sphere every tracker draw reuses (see drawTracker). Built in the
    // constructor -- it only needs a live GL context, not a loaded terrain.
    GpuMesh    m_sphere;
};
