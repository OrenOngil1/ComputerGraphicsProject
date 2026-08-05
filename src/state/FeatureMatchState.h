#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include <glm/glm.hpp>

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
    // 8 per view is what a pose actually needs from a hand-built database:
    // fewer than that and a single misplaced anchor is a large share of the
    // consensus. They are spread across the frame (see detectSpreadFeatures),
    // so 8 is 8 usable places rather than 8 dots on one corner.
    static constexpr size_t kDefaultFeatures = 8;
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
    void undoAnchor();                                           // U: take back the last placement
    void finishBuild(Simulation &sim, Renderer &renderer);

    // Give every hand-placed point the descriptors of its appearances in the
    // OTHER recorded views. ORB is not viewpoint-invariant, so one appearance
    // per point is what makes free flight fail where a recorded waypoint works;
    // the 3D still comes entirely from the user's click.
    void addOtherViewAppearances(Simulation &sim, Renderer &renderer);

    // Ctrl+S / Ctrl+O: the database on disk. Placing anchors is minutes of
    // human work per build, so it survives the session rather than being redone
    // for every measurement.
    void saveDatabase(const Simulation &sim) const;
    void loadDatabase(Simulation &sim, Renderer &renderer);

    // Build-phase map aids: the active suggestion's sight line, plus a dim one
    // per anchor already placed in this view. Drawn through the global view's
    // mvp; see the .cpp for why they stop short of the terrain.
    void drawSightAids(const Simulation &sim, Renderer &renderer, const glm::mat4 &mvp) const;

    // Cache the database's distinct hand-placed points. Recomputed only when
    // the database itself changes (a finished build, a load), because the rows
    // repeat a point once per appearance and the scan is quadratic.
    void refreshPlaces();

    // Split those points into the ones the player's current view can use and
    // the ones it cannot. Recomputed once per tick and once per capture, never
    // per draw: the occlusion half marches a ray per point.
    void refreshAnchorVisibility(const Simulation &sim);

    // The in-view count on the console, printed only when it changes and at
    // most twice a second -- it churns continuously while flying.
    void reportAnchorCount();

    size_t                     m_featureCount;
    std::unique_ptr<FeatureDb> m_db;         // hand-built; empty until a build completes
    std::unique_ptr<BuildScratch> m_build;   // non-null only while building (G in progress)

    // The database's distinct hand-placed points, and the refreshAnchorVisibility
    // split of them for the overlay to draw.
    std::vector<glm::vec3> m_places;
    std::vector<glm::vec3> m_inView, m_outOfView;

    // Debounce state for reportAnchorCount: the count last printed (npos so the
    // first refresh always reports) and when.
    size_t m_reportedInView  = (size_t)-1;
    double m_lastCountReport = -1.0;
};
