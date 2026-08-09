#pragma once

#include <cstddef>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../core/Camera.h"              // Camera, Viewport, Waypoint
#include "../core/Scene.h"               // Mesh
#include "../vision/FeatureMatching.h"   // FeatureDb, FrameCapture

// Mode D's hand build: walk the recorded views, offer each one's strongest SIFT
// points, and pair the active suggestion's descriptor with a 3D point the user
// picked. Owns the database while placing and hands it over when the views run
// out -- so a half-built database is not merely refused by the save path, it
// does not exist to be saved.
//
// Knows nothing of Renderer, Simulation or GLFW: the caller does the picking
// and the drawing, and passes a Context in. That is what lets a build be driven
// headlessly.
class ManualBuild {
public:
    // Everything a step needs from outside, rebuilt by the caller per call so
    // the build stores no references into the session.
    struct Context {
        const Mesh  &mesh;
        float        terrainSize;
        Camera       prototype;      // the lens each view is posed onto
        size_t       featureCount;   // suggestions offered per view
        FrameCapture render;         // draws one view offscreen
    };

    // A completed build, handed over in one move.
    struct Result {
        FeatureDb          db;
        std::vector<float> placementErrors;   // per anchor, placement order
    };

    // Walks `views` in order, stopping at the first that has something to
    // place. `frame` is the size every capture renders at -- part of the
    // database's identity, since SIFT descriptors shift with resolution.
    ManualBuild(std::vector<Waypoint> views, Viewport frame, const Context &ctx);

    // True once the views run out: the caller takes the result and drops this.
    bool     finished() const { return m_view >= m_views.size(); }
    size_t   viewIndex() const { return m_view; }
    size_t   viewCount() const { return m_views.size(); }
    Waypoint currentPose() const { return m_views[m_view]; }
    Viewport frame() const { return m_frame; }

    // What the overlays draw: [0,1] fractions of `frame`. activeMarker equals
    // markers().size() only in the instant before a step advances the view.
    const std::vector<glm::vec2> &markers() const { return m_markers; }
    size_t                        activeMarker() const { return m_active; }
    const std::vector<size_t>    &anchoredMarkers() const { return m_anchoredAt; }
    const FeatureDb              &database() const { return m_db; }

    // The active suggestion's viewing ray, so the caller's snap and sight-line
    // aids agree with what place() measures against.
    glm::vec2 activeRay(float fov) const;

    // Anchor the active suggestion at `anchor`; `ray` is what activeRay gave.
    void place(const glm::vec3 &anchor, const glm::vec2 &ray, const Context &ctx);
    void skipSuggestion(const Context &ctx);    // X
    void skipRestOfView(const Context &ctx);    // Ctrl+X
    void undo();                                // U: take back this view's last anchor

    Result takeResult();

private:
    // Pose, capture and detect until a view has something to place, or the
    // views run out.
    void advanceToPlaceableView(const Context &ctx);

    // The active suggestion is spent: step to the next, or to the next view.
    void advance(const Context &ctx);

    std::vector<Waypoint> m_views;
    Viewport              m_frame;
    FeatureDb             m_db;

    size_t                 m_view   = 0;
    size_t                 m_active = 0;
    std::vector<glm::vec2> m_markers;       // [0,1] fractions of the suggestions
    cv::Mat                m_descriptors;   // one SIFT descriptor row per marker

    // Markers anchored in THIS view, newest last -- the undo stack. Indices
    // rather than a count because skips leave gaps: stepping m_active back by
    // one would land on a skipped suggestion. Cleared on a new view.
    std::vector<size_t> m_anchoredAt;

    // Distance from each anchor to its ray's true terrain point, whole build,
    // placement order. Measured at placement, disclosed only in the caller's
    // debrief. Negative marks a ray that missed the terrain (kept so undo stays
    // aligned with m_anchoredAt).
    std::vector<float> m_placementErrors;
};
