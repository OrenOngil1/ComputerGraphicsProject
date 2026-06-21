#pragma once

#include <cstddef>
#include <memory>
#include <optional>

#include "PoseComparisonState.h"
#include "../core/Camera.h"   // Waypoint

// Vision-side store (src/vision/FeatureMatching.h) and the pre-phase scratch,
// both kept behind pointers so OpenCV types stay out of the state headers.
struct FeatureDb;
struct BuildScratch;

// Mode 4: pose estimation by 2D feature matching -- MANUAL, like picking. The
// pipeline is unchanged from the automatic version (F enter, G build, B/N/M
// run-phase comparison from PoseComparisonState); only the pre-phase anchoring
// is by hand:
//
//   G (pre-phase) -- interactive build: step through each recorded view; for
//                    each, ORB highlights its strongest N points one at a time
//                    in the player (right) view and the user color-picks each
//                    one's matching 3D spot in the global (left) map. Those
//                    hand-placed (descriptor, 3D) pairs are the database.
//   B (run-phase) -- inherited capture flow: record the true pose, then match
//                    the current view's ORB features against the hand-built
//                    database (+ RANSAC PnP) to estimate the pose.
//
// Lighting (L) feeds the reframed experiment: changing the light changes which
// points ORB proposes (and how the run-phase frame matches), so a database built
// under one light degrades under another. Entered via F; requires recorded
// waypoints (they are the views to anchor).
class FeatureMatchState : public PoseComparisonState {
public:
    // Features suggested per view, public so the F-key prompt (Callbacks.cpp)
    // and this class share one range policy.
    static constexpr size_t kDefaultFeatures = 5;
    static constexpr size_t kMaxFeatures     = 20;

    // The feature count is transition-time configuration, chosen at the F-key
    // prompt and fixed for the mode's lifetime.
    explicit FeatureMatchState(size_t featureCount = kDefaultFeatures);
    ~FeatureMatchState() override;   // out-of-line: m_db/m_build are incomplete here

    // The F-key terminal prompt (1..kMaxFeatures, plain Enter = the default),
    // same shape as TrackersState::promptCount.
    static size_t promptCount();

    void onEnter(Simulation &sim) override;
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;
    void handleMouseButton(Simulation &sim, Renderer &renderer, GLFWwindow *window,
                           int button, int action) override;
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
    void renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;

protected:
    // Run-phase estimate: match the current view against the hand-built database.
    std::optional<Waypoint> computePose(Simulation &sim, Renderer &renderer) override;

private:
    bool building() const { return m_build != nullptr; }

    void startBuild(Simulation &sim, Renderer &renderer);        // G: begin the manual build
    void loadCurrentView(Simulation &sim, Renderer &renderer);   // pose + detect for m_build->waypoint
    void advance(Simulation &sim, Renderer &renderer);           // active suggestion done (anchored/skipped)
    void finishBuild();

    size_t                     m_featureCount;
    std::unique_ptr<FeatureDb> m_db;      // hand-built; empty until a build completes
    std::unique_ptr<BuildScratch> m_build;   // non-null only while building (G in progress)
};
