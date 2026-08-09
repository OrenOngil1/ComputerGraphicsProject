#pragma once

#include <optional>   // renderScene's skyPreset
#include <vector>

#include <glm/glm.hpp>

#include <Shader.h>

#include "GpuMesh.h"
#include "OverlayBatch.h"
#include "SkyPass.h"
#include "../core/Simulation.h"

// Owns all GPU resources (shaders, terrain + tracker-sphere buffers) and every
// draw and read-back pass. Owned as an Application member declared after the
// Window, so it is constructed and destroyed while the GL context is live.
class Renderer {
public:
    // Compiles the shader programs; no terrain yet -- render paths assume
    // loadTerrain has been called first, which the session loop guarantees.
    Renderer();

    // Non-copyable: sole owner of the GPU resources.
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;

    void clear() const;   // glClear(COLOR | DEPTH)

    // Swap the terrain buffers in place; the compiled shaders are reused.
    void loadTerrain(const Mesh &terrain);

    // Paint one viewport: draw the lit terrain, then let the active mode
    // decorate it with its overlay for that view.
    void renderGlobalView(const View &view, const Simulation &sim);
    void renderPlayerView(const View &view, const Simulation &sim);

    // Overlay primitives, called by the modes' render*Overlay methods. All
    // draw through one persistent dynamic buffer (m_overlayBatch) -- the
    // geometry changes every frame, the GPU storage does not.
    void drawPath(const std::vector<glm::vec3> &pathPoints, const glm::vec3 &color,
                  const glm::mat4 &mvp);
    // Waypoint dots: green where cameraPos sits (exact position match), red otherwise.
    void drawWaypoints(const std::vector<Waypoint> &waypoints,
                       const glm::vec3 &cameraPos, const glm::mat4 &mvp);

    // Color-pick pass: render the terrain with per-vertex id colors into the
    // back buffer (never swapped, so invisible) and read back the pixel under
    // (mouseX, mouseY). Returns the vertex id there, or -1 on a miss. Callers
    // pass the GLOBAL view to pick 3D points off the map, or the player view
    // as a terrain hit-test.
    int pickVertex(int mouseX, int mouseY, const View &view);

    // The PnP "ghost": the terrain re-drawn from an estimated pose in one
    // translucent color, without depth, over the current view.
    void drawGhost(const Camera &estimatedCamera, const Viewport &viewport,
                   const glm::vec3 &color, float alpha, float tintStrength);

    // Colored point markers, one color per position, drawn with depth test off
    // so markers on the terrain surface are never hidden behind it.
    void drawPoints(const std::vector<glm::vec3> &positions,
                    const std::vector<glm::vec3> &colors,
                    float size, const glm::mat4 &mvp);

    // Independent line segments: consecutive pairs (0-1, 2-3, ...), one color
    // for the batch, depth test off like the markers. A trailing odd vertex is
    // ignored. The shared primitive under the view aids below.
    void drawLines(const std::vector<glm::vec3> &segments, const glm::vec3 &color,
                   float width, const glm::mat4 &mvp);

    // One segment, staged through the same reused scratch: a caller drawing a
    // single line every frame would otherwise build a two-element vector per
    // frame, which is a heap allocation in the render loop.
    void drawLine(const glm::vec3 &from, const glm::vec3 &to, const glm::vec3 &color,
                  float width, const glm::mat4 &mvp);

    // ── View aids: one view's geometry drawn into another ──────
    // Both take the PLAYER camera and are drawn through the GLOBAL view's mvp,
    // which is what makes them useful: they answer "where is the other pane
    // looking" on the map.

    // The player camera's view volume: apex at the eye, four edges out to
    // `reach`, and the rectangle closing them. `aspect` is the player
    // viewport's -- the cone must match the frame the user is matching against,
    // not the viewport it is drawn into.
    void drawViewCone(const Camera &camera, float aspect, float reach,
                      const glm::vec3 &color, float width, const glm::mat4 &mvp);

    // The lines from the eye along given camera-space viewing rays (x/z, y/z):
    // the locus each observation's 3D point must lie on.
    //
    // Deliberately NOT intersected with the terrain. A pixel determines a
    // direction, not a point; resolving the depth along it is the manual step
    // PICK and FEATURE MATCH exist to have the user perform, so drawing the
    // hit would hand over the correspondence itself.
    void drawSightLines(const Camera &camera, const std::vector<glm::vec2> &imageRays,
                        float reach, const glm::vec3 &color, float width,
                        const glm::mat4 &mvp);

    // Draw the trackers as flat-colored spheres -- the exact palette colors the
    // blob detector classifies, in the visible frame and the detection capture
    // alike. Depth-tested, unlike the markers above: a tracker behind a hill is
    // hidden in both, the occlusion semantics automatic correspondence needs.
    // viewProj carries no model part.
    void drawTrackers(const std::vector<Tracker> &trackers, const glm::mat4 &viewProj);

    // Tracker detection pass: the player view -- terrain lit as the player
    // sees it, trackers in their flat palette colors -- read back. The
    // detector separates the two by color alone, which holds because the
    // palette is disjoint from every color the lit terrain can render (see
    // trackerPalette in TrackersState.cpp). Occlusion comes from the depth
    // buffer. Back buffer only, never swapped.
    FramePixels captureTrackersFrame(const View &playerView,
                                     const std::vector<Tracker> &trackers,
                                     const DirectionalLight &light);

    // Feature-matching capture: the lit scene exactly as the player view draws
    // it, minus overlays -- the pixels SIFT runs on. Takes the light explicitly:
    // the same view under a different preset must produce different pixels
    // (that is the Mode 4 lighting experiment). Back buffer only, never swapped.
    FramePixels captureSceneFrame(const View &view, const DirectionalLight &light);

    // The same capture rendered offscreen at an explicit size, independent of
    // the window. SIFT descriptors shift when the same scene is rasterised at a
    // different resolution, so a database is only ever exact against frames of
    // the size it was built at. The feature-matching run phase captures at the
    // database's recorded size through this, so the window may be anything.
    FramePixels captureSceneFrameAt(int width, int height, const Camera &camera,
                                    const DirectionalLight &light);

private:
    // Bind the scene shader and raise u_Lit with this light's uniforms.
    void applyLighting(const DirectionalLight &light);

    // Lit 3D draw of the mesh's own vertex colors (the terrain), bracketed by
    // the u_Lit raise/drop -- unlit is the shared program's resting state,
    // which every exact-color draw depends on.
    void drawMeshLit(const GpuMesh &gpu, const DirectionalLight &light,
                     const glm::mat4 &mvp);

    // Shared per-view pass: set the viewport, draw the lit terrain, then the
    // sky when a preset index is given -- the visible views pass the active
    // preset; the captures leave it defaulted so their background stays the
    // flat clear color. Returns the MVP for the overlays.
    glm::mat4 renderScene(const Camera &camera, const Viewport &viewport,
                          const DirectionalLight &light,
                          std::optional<size_t> skyPreset = std::nullopt);

    // Read the viewport's back-buffer pixels into image-convention RGB (rows
    // flipped top-down, packing forced tight). Shared tail of every capture.
    FramePixels readViewportPixels(const Viewport &viewport);

    // The id pass behind pickVertex: terrain in flat per-vertex-id colors on a
    // white background (white decodes to a miss).
    void renderPickPass(const Camera &camera, const Viewport &viewport);

    // Draw a mesh through the scene shader's override path (one fill color,
    // or a tint of the vertex colors), restoring the override to off
    // afterward -- every other draw through the shared program depends on it
    // being off. Always unlit: the override path bypasses the shader's lit
    // branch, so these draws land in exact colors.
    void drawMeshFlat(const GpuMesh &gpu, const glm::vec4 &fill, float tintStrength,
                      const glm::mat4 &mvp);

    Shader     m_sceneShader;   // lit terrain + flat-override path (ghost, trackers, captures)
    Shader     m_pickShader;    // per-vertex-id colors for the color-pick pass
    Shader     m_pointShader;   // round GL_POINTS markers (discards sprite corners)
    GpuMesh    m_terrain;       // rebuilt per loadTerrain
    GpuMesh    m_sphere;        // unit sphere shared by all tracker draws, built once
    SkyPass    m_sky;           // per-preset skybox: own shader, cube, cubemap cache

    OverlayBatch m_overlayBatch;   // persistent GPU buffer behind every overlay draw

    // Frame-to-frame staging scratch for the overlay draws, so filling a batch
    // allocates nothing once the capacity has warmed up. Each drawing method
    // clears and refills; none of them nest.
    std::vector<float>     m_overlayVerts;
    std::vector<glm::vec3> m_overlaySegments;
};
