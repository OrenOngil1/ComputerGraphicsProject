#pragma once

#include <cstddef>
#include <memory>
#include <optional>

#include "PoseComparisonState.h"
#include "../core/Camera.h"   // Waypoint

// Both behind pointers so OpenCV types stay out of the state headers.
struct FeatureDb;
struct BuildScratch;

// Mode D: pose estimation by 2D feature matching, with manual anchoring.
// Two phases share this one State:
//
//   G (pre-phase) -- interactive build: step through each recorded view; ORB
//                    highlights its strongest N points one at a time in the
//                    player view and the user color-picks each one's 3D spot
//                    in the global map. The hand-placed (descriptor, 3D)
//                    pairs are the database.
//   B (run-phase) -- inherited capture flow: record the true pose, match the
//                    live view's ORB features against the database, RANSAC PnP.
//
// Lighting (L) feeds the experiment: a database anchored under one light
// degrades under another. Requires recorded waypoints (the views to anchor).
class FeatureMatchState : public PoseComparisonState {
public:
    static constexpr size_t kDefaultFeatures = 5;
    static constexpr size_t kMaxFeatures     = 20;

    // featureCount (suggestions per view) is fixed for the mode's lifetime.
    explicit FeatureMatchState(size_t featureCount = kDefaultFeatures);
    ~FeatureMatchState() override;   // out-of-line: m_db/m_build are incomplete here

    // The F-key terminal prompt (1..kMaxFeatures, plain Enter = the default).
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
    std::unique_ptr<FeatureDb> m_db;         // hand-built; empty until a build completes
    std::unique_ptr<BuildScratch> m_build;   // non-null only while building (G in progress)
};
