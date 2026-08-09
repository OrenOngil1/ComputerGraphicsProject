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
class ManualBuild;

// How far the heading may turn from a stored view's before SIFT stops
// re-finding its appearances. Measured, not theoretical: 22.5-degree steps
// cross-match between stations, a 36-degree ring collected nothing. Nothing
// gates on it -- the envelope aid below only colors by it.
constexpr float kViewpointToleranceDegrees = 25.0f;

// The nearest recorded view, in the two dimensions recognition depends on:
// distance between the eyes and angle between the headings -- the pilot's live
// "am I inside the envelope" readout. Empty when nothing was recorded.
struct NearestView {
    size_t    index;              // into the waypoint list, for the console line
    glm::vec3 position;           // that view's eye, COPIED: the overlay draws to it
                                  // a frame later, and the waypoint list can be
                                  // replaced (a load, an auto-build) in between
    float     distanceUnits;      // camera eye to the view's eye
    float     headingOffDegrees;  // between the two view directions
};
std::optional<NearestView> nearestRecordedView(const std::vector<Waypoint> &views,
                                               const Camera &camera);

// Mode D: pose estimation by 2D feature matching, with manual anchoring
// (docs/pose-estimation-modes.md, "Mode D"). G builds the database -- SIFT
// suggests each view's strongest points, the user color-picks each one's 3D
// spot, and those hand-placed (descriptor, 3D) pairs ARE the database. B is the
// inherited capture flow. Needs recorded waypoints: they are the views to
// anchor.
class FeatureMatchState : public PoseComparisonState {
public:
    // What a pose needs from a hand-built database: fewer, and one misplaced
    // anchor is a large share of the consensus. They are spread across the
    // frame (see detectSpreadFeatures), so 8 is 8 usable places.
    static constexpr size_t kDefaultFeatures = 8;
    static constexpr size_t kMaxFeatures     = 20;

    // Both fixed for the mode's lifetime. minInliers is the consensus floor a
    // capture must reach; nullopt leaves the choice to the solver's rule.
    explicit FeatureMatchState(size_t featureCount = kDefaultFeatures,
                               std::optional<size_t> minInliers = std::nullopt);
    ~FeatureMatchState() override;   // out-of-line: m_db/m_build are incomplete here

    // The two F-key terminal prompts; plain Enter takes the default in each.
    static size_t promptCount();
    static std::optional<size_t> promptMinInliers();

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

    void startBuild(Simulation &sim, Renderer &renderer);   // G: begin the manual build

    // Pose the player camera at the view being anchored, and take over once the
    // build reports it has no views left. Called after every build step.
    void followBuild(Simulation &sim, Renderer &renderer);

    // Take the finished build's database, report on it, and return to the run
    // phase. Only followBuild calls this.
    void finishBuild(Simulation &sim, Renderer &renderer);

    // Give every hand-placed point the descriptors of its appearances in the
    // OTHER recorded views. No descriptor is viewpoint-invariant, so one
    // appearance per point is what makes free flight fail where a recorded
    // waypoint works; the 3D still comes entirely from the user's click.
    void addOtherViewAppearances(Simulation &sim, Renderer &renderer);

    // Ctrl+S / Ctrl+O: the database on disk. Placing anchors is minutes of
    // human work per build, so it survives the session rather than being redone
    // for every measurement.
    void saveDatabase(const Simulation &sim) const;
    void loadDatabase(Simulation &sim, Renderer &renderer);

    // Ctrl+G: an automated stand-in for the whole G workflow -- lay one of
    // three view paths (arc, high survey circle, or scattered stations),
    // anchor every suggestion with simulated human aim error (depth noise
    // along the sight line), collect appearances, save. Test databases in
    // seconds; the manual build remains the mode. See the .cpp for why the
    // simulation is honest.
    void autoBuild(Simulation &sim, Renderer &renderer);

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

    // The frame size every capture and collection renders at -- part of the
    // DATABASE's identity, not the window's. SIFT descriptors shift when the
    // same scene is rasterised at another resolution, so captures render
    // offscreen at this size and the window may be anything, any session.
    // Adopted from the finished build, from the file on a load, or from the
    // auto-build's canonical size; zero until then, and captureViewport falls
    // back to the live pane. Mid-build the build owns the frame size instead.
    Viewport captureViewport(const Simulation &sim) const;
    int m_captureWidth = 0, m_captureHeight = 0;

    size_t                     m_featureCount;
    std::optional<size_t>      m_minInliers;   // nullopt = the solver's rule

    // Only one of these holds a database at a time: the build owns it while
    // placing and hands it over when the views run out, so there is no
    // half-built database for the save path to reach.
    std::unique_ptr<FeatureDb>   m_db;      // empty until a build completes or a load
    std::unique_ptr<ManualBuild> m_build;   // non-null only while building (G in progress)

    // The database's distinct hand-placed points, and the refreshAnchorVisibility
    // split of them for the overlay to draw.
    std::vector<glm::vec3> m_places;
    std::vector<glm::vec3> m_inView, m_outOfView;

    // Where the camera stands relative to the collected views, refreshed with
    // the visibility split: the console line reads it, and the overlay draws
    // it as the envelope tether.
    std::optional<NearestView> m_nearestView;

    // Debounce state for reportAnchorCount: the values last printed (npos /
    // -1 so the first refresh always reports) and when. The nearest-view pair
    // is bucketed (5 units, 5 degrees) so drifting a hair does not spam.
    size_t m_reportedInView  = (size_t)-1;
    int    m_reportedDistanceBucket = -1;
    int    m_reportedHeadingBucket  = -1;
    double m_lastCountReport = -1.0;

    // The once-per-pin latch for estimatePoseFromFeatures's small-pinned-floor
    // warning. A member, not a static in the solver: the warning is about THIS
    // mode entry's pinned floor, so a re-entry with a new pin must warn again.
    bool m_pinnedFloorNoted = false;

    // Said once, on the first capture of the session: captures are easy to
    // misread as growing the database, since both a capture and an anchor draw
    // as a small dot on the same map. They do not -- only G does that.
    bool m_captureHintShown = false;
};
