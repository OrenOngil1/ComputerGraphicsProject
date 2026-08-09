#include "FlightPaths.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

static constexpr float kTau = 6.28318530718f;

// Fixed, so two paths with the same parameters are the same flight.
static constexpr unsigned int kPathSeed = 20260805u;

// Arc: span, orbit radius, altitude (fractions of the terrain's width). The
// build ring reuses the radius and altitude -- that pairing is the calibration.
static constexpr float kArcSpanDegrees      = 120.0f;
static constexpr float kArcRadiusFraction   = 0.30f;
static constexpr float kArcAltitudeFraction = 0.25f;

// High survey circle: orbit radius, altitude, how far past the centre each stop
// aims (fractions of the terrain's width), and the stop count below which the
// azimuth step outgrows SIFT's viewpoint tolerance.
static constexpr float  kCircleRadiusFraction       = 0.22f;
static constexpr float  kCircleAltitudeFraction     = 0.50f;
static constexpr float  kCircleAimOvershootFraction = 0.17f;
static constexpr size_t kCircleComfortViews         = 12;

// Scattered stations: sampling extent and look-ahead (fractions of the
// terrain's width), the altitude band, and how many random candidates compete
// for each spot.
static constexpr float kScatterExtentFraction    = 0.35f;
static constexpr float kScatterLookaheadFraction = 0.45f;
static constexpr float kScatterAltitudeMin       = 0.30f;
static constexpr float kScatterAltitudeMax       = 0.50f;
static constexpr int   kScatterCandidates        = 16;

const AutoPathSpec kAutoPathSpecs[3] = {
    { "an arc", 8,
      "FEATURES: recognition is strongest along the arc's own corridor -- fly"
      " the low sweep around the middle, looking at the centre" },
    { "a high survey circle", kCircleComfortViews,
      "FEATURES: recognition is strongest from viewpoints like the orbit's own"
      " -- fly near the grey ring, high, looking across the middle" },
    { "scattered survey stations", 10,
      "FEATURES: recognition is strongest near the recorded stations -- their"
      " spread trades per-spot depth for coverage of the whole map" },
};

// An ARC, not a full ring -- low, every stop aimed at the centre. A low ring's
// neighbouring views sit ~36 degrees apart, past SIFT's ~15-20 degree tolerance
// on 3D relief, and nothing matches across them; the arc's ~13 degree steps
// reproduce every manual corridor that worked.
static void layOutArc(Simulation &sim, size_t views)
{
    const float arcSpan  = glm::radians(kArcSpanDegrees);
    const float radius   = kArcRadiusFraction   * sim.terrainSize;
    const float altitude = kArcAltitudeFraction * sim.terrainSize;

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < views; i++) {
        const float t     = views > 1 ? (float)i / (float)(views - 1) : 0.5f;
        const float angle = (t - 0.5f) * arcSpan;
        const glm::vec3 eye(radius * std::cos(angle), altitude, radius * std::sin(angle));
        sim.waypoints.push_back({ eye, glm::vec3(0.0f) });
        sim.pathPoints.push_back(eye);
    }
}

// A full circle flown HIGH, each stop aimed PAST the centre. Height is the
// legality condition: from high up an azimuth step is mostly in-plane rotation,
// which SIFT absorbs where a low ring's is fatal (see layOutArc). The
// past-centre aim fans the footprints across the middle.
static void layOutSurveyCircle(Simulation &sim, size_t views)
{
    const float radius    = kCircleRadiusFraction       * sim.terrainSize;
    const float altitude  = kCircleAltitudeFraction     * sim.terrainSize;
    const float overshoot = kCircleAimOvershootFraction * sim.terrainSize;

    if (views < kCircleComfortViews)
        std::cout << "FEATURES: note -- with fewer than ~" << kCircleComfortViews
                  << " stops a circle's azimuth steps outgrow SIFT's viewpoint"
                     " tolerance; expect weaker cross-view collection" << std::endl;

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < views; i++) {
        const float angle = kTau * (float)i / (float)views;
        const glm::vec3 out(std::cos(angle), 0.0f, std::sin(angle));
        sim.waypoints.push_back({ out * radius + glm::vec3(0.0f, altitude, 0.0f),
                                  -out * overshoot });
        sim.pathPoints.push_back(sim.waypoints.back().position);
    }
    sim.pathPoints.push_back(sim.waypoints.front().position);   // close the ring on the map
}

// Best-candidate spacing: take the farthest of kScatterCandidates random spots
// from everything already placed -- spread without a tuning parameter.
static glm::vec2 bestSpacedSpot(std::mt19937 &rng, const std::vector<Waypoint> &placed,
                                const glm::vec2 &extent)
{
    std::uniform_real_distribution<float> coordX(-extent.x, extent.x);
    std::uniform_real_distribution<float> coordZ(-extent.y, extent.y);
    glm::vec2 best(0.0f);
    float bestScore = -1.0f;
    for (int c = 0; c < kScatterCandidates; c++) {
        const glm::vec2 candidate(coordX(rng), coordZ(rng));
        float nearest = std::numeric_limits<float>::max();
        for (const Waypoint &w : placed)
            nearest = std::min(nearest, glm::distance(candidate,
                                                      glm::vec2(w.position.x, w.position.z)));
        if (nearest > bestScore) {
            bestScore = nearest;
            best = candidate;
        }
    }
    return best;
}

// An evenly divided compass, jittered and shuffled: the angular spread is
// guaranteed, the order and exact bearings are not.
static std::vector<float> spreadHeadings(std::mt19937 &rng, size_t views)
{
    std::uniform_real_distribution<float> jitter(-0.5f, 0.5f);
    std::vector<float> headings(views);
    for (size_t i = 0; i < views; i++)
        headings[i] = kTau * ((float)i + jitter(rng)) / (float)views;
    std::shuffle(headings.begin(), headings.end(), rng);
    return headings;
}

// Well-spaced random stations, each at its own altitude and heading. No
// geometry ties neighbouring views together, so cross-view collection finds
// little -- the shape trades per-spot depth for coverage.
static void layOutScatteredStations(Simulation &sim, size_t views)
{
    std::mt19937 rng(kPathSeed);
    // Per-axis, NOT a terrainSize square (that is max(cols, rows)): a non-square
    // DEM would put square-sized stations over empty space. The lookahead
    // follows the short axis so the re-aim below stays on the grid.
    const glm::vec2 grid((float)sim.mesh.cols, (float)sim.mesh.rows);
    const glm::vec2 extent  = kScatterExtentFraction * grid;
    const glm::vec2 mapEdge = 0.5f * grid;
    const float lookahead = kScatterLookaheadFraction * std::min(grid.x, grid.y);
    std::uniform_real_distribution<float> altitude(kScatterAltitudeMin * sim.terrainSize,
                                                   kScatterAltitudeMax * sim.terrainSize);
    const std::vector<float> headings = spreadHeadings(rng, views);

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < views; i++) {
        const glm::vec2 spot = bestSpacedSpot(rng, sim.waypoints, extent);
        glm::vec2 target = spot + glm::vec2(std::cos(headings[i]),
                                            std::sin(headings[i])) * lookahead;
        // A station that would stare off the map looks inward across the centre
        // instead. Always lands on the grid: crossing the centre ends within
        // lookahead of it. Off-map implies |spot| > 0, so the normalize is safe.
        if (std::abs(target.x) > mapEdge.x || std::abs(target.y) > mapEdge.y)
            target = spot - lookahead * glm::normalize(spot);
        sim.waypoints.push_back({ glm::vec3(spot.x, altitude(rng), spot.y),
                                  glm::vec3(target.x, 0.0f, target.y) });
        sim.pathPoints.push_back(sim.waypoints.back().position);
    }
    std::cout << "FEATURES: note -- scattered stations rarely revisit a spot"
                 " within SIFT's viewpoint tolerance; expect weaker cross-view"
                 " collection than the circle's" << std::endl;
}

void layOutAutoPath(Simulation &sim, AutoPathMode mode, size_t views)
{
    switch (mode) {
        case AutoPathMode::Arc:       layOutArc(sim, views);               break;
        case AutoPathMode::Circle:    layOutSurveyCircle(sim, views);      break;
        case AutoPathMode::Scattered: layOutScatteredStations(sim, views); break;
    }
}

// The count IS the calibration: 16 stops step 22.5 degrees -- inside the ~25
// degree range a 21-degree flight demonstrated, and under the 36-degree steps
// of a hand-flown 10-station ring, where collection died (95 pairs missed).
static constexpr size_t kBuildRingViews = 16;

void layOutBuildRing(Simulation &sim)
{
    const float radius   = kArcRadiusFraction   * sim.terrainSize;
    const float altitude = kArcAltitudeFraction * sim.terrainSize;

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < kBuildRingViews; i++) {
        const float angle = kTau * (float)i / (float)kBuildRingViews;
        sim.waypoints.push_back({ glm::vec3(radius * std::cos(angle), altitude,
                                            radius * std::sin(angle)),
                                  glm::vec3(0.0f) });
        sim.pathPoints.push_back(sim.waypoints.back().position);
    }
    sim.pathPoints.push_back(sim.waypoints.front().position);   // close the ring on the map
}
