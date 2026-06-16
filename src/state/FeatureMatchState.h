#pragma once

#include <memory>
#include <optional>

#include "PoseComparisonState.h"
#include "../core/Camera.h"   // Waypoint

// Vision-side store (src/vision/FeatureMatching.h), kept behind a pointer so
// OpenCV types stay out of the state headers.
struct FeatureDb;

// Mode 4: pose estimation by 2D feature matching -- no markers in the scene,
// only what the terrain itself looks like. Two phases, both user-driven so
// the lighting experiment is possible:
//
//   G (pre-phase) -- (re)build the feature database: render each recorded
//                    waypoint's view under the CURRENT light, detect ORB
//                    features, anchor them to 3D through the id pass.
//   B (run-phase) -- inherited capture flow: record the true pose, and
//                    computePose estimates it by matching the current view's
//                    features against the database (+ RANSAC PnP).
//
// Changing the light (L) between G and B is the Mode 4 experiment: the
// database keeps the pre-phase appearance while the run-phase frames look
// different, and the match/inlier counts show how the matching degrades.
//
// Flight, B/N/M handling, and both comparison overlays come from
// PoseComparisonState. Entered via F; requires recorded waypoints (they are
// the pre-phase views).
class FeatureMatchState : public PoseComparisonState {
public:
    // Out of line: m_db's unique_ptr needs FeatureDb complete only where
    // these are defined (the incomplete-type pattern Simulation uses for
    // State, see Simulation.cpp).
    FeatureMatchState();
    ~FeatureMatchState() override;

    void onEnter(Simulation &sim) override;
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;

protected:
    std::optional<Waypoint> computePose(Simulation &sim, Renderer &renderer) override;

private:
    void buildDatabase(Simulation &sim, Renderer &renderer);

    std::unique_ptr<FeatureDb> m_db;   // empty until the first G
};
