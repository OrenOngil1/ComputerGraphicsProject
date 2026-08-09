#include "ManualBuild.h"

#include <algorithm>
#include <iostream>
#include <optional>

// The build's duplicate guard: drop suggestions that already resemble an
// anchored point. Left in, they would be re-anchored as SECOND places --
// identities the matcher cannot tell apart (measured as twin pairs a couple of
// units apart). Re-sightings belong to the appearance pass; the auto-build's
// selectSpacedAnchors is this same guard in map space.
static size_t dropAnchoredLookalikes(const FeatureDb &db,
                                     std::vector<cv::KeyPoint> &kps, cv::Mat &desc)
{
    if (db.empty() || kps.empty())
        return 0;

    std::vector<cv::KeyPoint> kept;
    cv::Mat keptDesc;
    for (int i = 0; i < desc.rows; i++) {
        if (resemblesAnyAnchoredPoint(db, desc.row(i)))
            continue;
        kept.push_back(kps[(size_t)i]);
        keptDesc.push_back(desc.row(i));
    }

    const size_t dropped = kps.size() - kept.size();
    kps  = std::move(kept);
    desc = keptDesc;
    return dropped;
}

// How far the placed anchor sits from where the suggestion's ray actually meets
// the terrain -- the answer the user was estimating. Negative marks a ray that
// missed the terrain within reach (nothing to compare against).
static float placementError(const ManualBuild::Context &ctx, const Camera &camera,
                            const glm::vec2 &ray, const glm::vec3 &anchor)
{
    const glm::vec3 direction = rayDirection(camera, ray);
    const float reach = ctx.terrainSize * 1.5f;
    const float step  = std::max(0.5f, ctx.terrainSize / 300.0f);
    const std::optional<float> depth =
        raycastTerrain(ctx.mesh, camera.position, direction, reach, step);
    if (!depth)
        return -1.0f;
    return glm::distance(anchor, camera.position + direction * *depth);
}

ManualBuild::ManualBuild(std::vector<Waypoint> views, Viewport frame, const Context &ctx)
    : m_views(std::move(views)), m_frame(frame)
{
    advanceToPlaceableView(ctx);
}

void ManualBuild::advanceToPlaceableView(const Context &ctx)
{
    while (m_view < m_views.size()) {
        Camera camera = ctx.prototype;   // throwaway; the caller's camera is its own
        camera.applyPose(m_views[m_view]);

        FramePixels frame = ctx.render(camera, m_frame);
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        detectSpreadFeatures(frame, ctx.featureCount, kps, desc);

        const size_t dropped = dropAnchoredLookalikes(m_db, kps, desc);
        if (dropped)
            std::cout << "FEATURES: view " << (m_view + 1) << " -- " << dropped
                      << " suggestion(s) are re-detections of already-anchored points;"
                         " skipped (the appearance pass collects re-sightings)" << std::endl;

        if (kps.empty()) {
            // Which of the two emptied it matters: no features is the terrain
            // being featureless here, all-dropped is the guard, and calling the
            // second one "no features" hid the guard from the one person who
            // could judge it.
            std::cout << "FEATURES: view " << (m_view + 1)
                      << (dropped ? " has nothing left to place -- every suggestion was"
                                    " already anchored; skipping."
                                  : " has no features -- skipping.") << std::endl;
            m_view++;
            continue;
        }

        m_markers.clear();
        m_markers.reserve(kps.size());
        for (const cv::KeyPoint &kp : kps)
            m_markers.push_back(glm::vec2(kp.pt.x / frame.width, kp.pt.y / frame.height));
        m_descriptors = desc;
        m_active = 0;
        m_anchoredAt.clear();   // undo is scoped to the current view
        std::cout << "FEATURES: view " << (m_view + 1) << "/" << m_views.size()
                  << " -- place " << kps.size() << " features on the map. ("
                  << m_db.anchors.size() << " anchored so far)" << std::endl;
        return;
    }
}

void ManualBuild::advance(const Context &ctx)
{
    m_active++;
    if (m_active < m_markers.size())
        return;
    m_view++;
    advanceToPlaceableView(ctx);
}

glm::vec2 ManualBuild::activeRay(float fov) const
{
    return fractionToRay(m_markers[m_active], fov, m_frame.aspect());
}

void ManualBuild::place(const glm::vec3 &anchor, const glm::vec2 &ray, const Context &ctx)
{
    Camera camera = ctx.prototype;
    camera.applyPose(m_views[m_view]);   // measure before advance moves the view on

    // The four halves of an anchor are pushed together and popped together by
    // undo(): m_db.anchors[i] belongs to descriptor row i, and both indices run
    // parallel to m_anchoredAt and m_placementErrors.
    m_db.descriptors.push_back(m_descriptors.row((int)m_active));
    m_db.anchors.push_back(anchor);
    m_anchoredAt.push_back(m_active);
    m_placementErrors.push_back(placementError(ctx, camera, ray, anchor));

    advance(ctx);
}

void ManualBuild::skipSuggestion(const Context &ctx)
{
    std::cout << "FEATURES: skipped a suggestion." << std::endl;
    advance(ctx);
}

// Ctrl+X: skip the REST of the current view in one press. The dense-ring
// workflow anchors in a few spread views and lets the appearance pass harvest
// the others -- walking each unanchored 8-suggestion view out with X was the
// friction that made recording more views expensive. Anchors already placed in
// the view stay placed.
void ManualBuild::skipRestOfView(const Context &ctx)
{
    const size_t unplaced = m_markers.size() - m_active;
    std::cout << "FEATURES: skipped the rest of view " << (m_view + 1) << " -- "
              << unplaced << " suggestion(s) left unplaced." << std::endl;
    m_view++;
    advanceToPlaceableView(ctx);
}

// Scoped to this view: crossing a view boundary would mean re-posing and
// re-running SIFT to rebuild the markers, and the mistake this exists for -- a
// misclick noticed right away -- never needs it.
void ManualBuild::undo()
{
    if (m_anchoredAt.empty()) {
        std::cout << "FEATURES: nothing to undo in this view." << std::endl;
        return;
    }

    m_active = m_anchoredAt.back();
    m_anchoredAt.pop_back();
    m_placementErrors.pop_back();
    m_db.anchors.pop_back();
    m_db.descriptors = m_db.descriptors.rowRange(0, m_db.descriptors.rows - 1).clone();

    std::cout << "FEATURES: undid the last anchor -- place feature "
              << (m_active + 1) << " again." << std::endl;
}

ManualBuild::Result ManualBuild::takeResult()
{
    return { std::move(m_db), std::move(m_placementErrors) };
}
