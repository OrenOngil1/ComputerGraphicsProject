#pragma once

#include <cstddef>
#include <vector>

#include "FeatureMatching.h"   // FeatureDb, FrameCapture
#include "../core/Camera.h"    // Camera, Viewport, Waypoint
#include "../core/Scene.h"     // Mesh

// An automated stand-in for Mode D's manual build (Ctrl+G): fly an already-laid
// path, anchor every suggestion with simulated aim error, hand back the
// database. The human is SIMULATED, not skipped -- true ray-terrain depth plus
// Gaussian error plays the user's EYES, never the estimator's, so the run phase
// still sees only (descriptor, 3D) pairs (docs, "Ctrl+G").

// Aim error sigma, in units along the sight line -- the one dimension a human
// supplies. Keep it modest: at sigma 15 a third of the anchors ended up buried
// past the visibility margin and every pose was (rightly) distrusted. Large
// values distort rather than degrade; the honest setting is your own number
// from the manual build's debrief.
constexpr size_t kDefaultAimErrorUnits = 4;
constexpr size_t kMaxAimErrorUnits     = 40;

constexpr size_t kMaxAutoViews        = 20;
constexpr size_t kDefaultAutoFeatures = 10;   // denser than the manual default:
                                              // clicks are free here

// The canonical frame size an auto build pins its database to.
constexpr int kAutoCaptureWidth  = 1280;
constexpr int kAutoCaptureHeight = 720;

struct AutoBuildSettings {
    size_t features;        // anchors to place per view
    size_t aimErrorUnits;   // sigma of the simulated human's depth error
};

struct AutoBuildResult {
    FeatureDb db;
    float     mapSpacing;   // the spacing the selection aimed for, for the summary
};

// Fly `views` -- laid by the caller -- and anchor as it goes. `prototype`
// supplies the lens, copied per view. An empty db means every suggestion missed
// the terrain. Deterministic: same path and settings, same database.
AutoBuildResult autoBuildDatabase(const Mesh &mesh, float terrainSize,
                                  const std::vector<Waypoint> &views,
                                  const Camera &prototype, const Viewport &vp,
                                  const AutoBuildSettings &settings,
                                  const FrameCapture &capture);
