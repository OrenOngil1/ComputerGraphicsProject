#include "FeatureMatching.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "Pnp.h"   // computeCameraPoseRansac

// The detector works on intensity; collapse the captured RGB to grayscale. The
// wrap constructor shares frame's bytes and cvtColor writes a fresh Mat, so the
// frame is never modified -- the const_cast is only because cv::Mat's wrap
// constructor has no const overload.
static cv::Mat toGray(const FramePixels &frame)
{
    cv::Mat rgb(frame.height, frame.width, CV_8UC3,
                const_cast<unsigned char *>(frame.rgb.data()));
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    return gray;
}

// The one detection entry for every phase: routing all detection through here
// makes pre- and run-phase descriptors comparable structurally, not by
// convention. SIFT because this terrain is textureless shading: separating
// "the same place again" from a lookalike ridge needs the gradient-orientation
// histograms, and binary brightness tests cannot do it at any threshold
// (docs/pose-estimation-modes.md, "Run-phase (B)").
void detectAllFeatures(const FramePixels &frame,
                       std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    static const cv::Ptr<cv::SIFT> sift = cv::SIFT::create(1000);
    sift->detectAndCompute(toGray(frame), cv::noArray(), keypoints, descriptors);
}

std::vector<size_t> rankByResponse(const std::vector<cv::KeyPoint> &keypoints)
{
    std::vector<size_t> order(keypoints.size());
    std::iota(order.begin(), order.end(), size_t(0));
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return keypoints[a].response > keypoints[b].response;
    });
    return order;
}

// Take a keypoint only when it clears the current spacing from everything
// already taken -- strongest first, so the spread is paid for with the weaker
// of any two crowded keypoints. Spacing starts at what maxCount points would
// have if they tiled the frame evenly, pulled in so the quota still fills.
static std::vector<size_t> pickSpreadKeypoints(const std::vector<cv::KeyPoint> &kps,
                                               size_t maxCount, int width, int height)
{
    float radius = 0.8f * std::sqrt((float)width * (float)height / (float)maxCount);
    const std::vector<size_t> order = rankByResponse(kps);

    std::vector<size_t> chosen;
    for (size_t attempt = 0; attempt < kSpacingRetries && chosen.size() < maxCount; attempt++, radius *= 0.5f) {
        chosen.clear();
        for (size_t i : order) {
            const cv::Point2f &pt = kps[i].pt;
            if (pt.x < kRimMarginPx || pt.y < kRimMarginPx ||
                pt.x > (float)width - kRimMarginPx ||
                pt.y > (float)height - kRimMarginPx)
                continue;

            const bool crowded = std::any_of(chosen.begin(), chosen.end(),
                                             [&](size_t j) {
                                                 return cv::norm(kps[j].pt - pt) < radius;
                                             });
            if (crowded)
                continue;

            chosen.push_back(i);
            if (chosen.size() == maxCount)
                break;
        }
    }
    return chosen;
}

void detectSpreadFeatures(const FramePixels &frame, size_t maxCount,
                          std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    std::vector<cv::KeyPoint> allKps;
    cv::Mat allDesc;
    detectAllFeatures(frame, allKps, allDesc);

    keypoints.clear();
    descriptors.release();
    if (allKps.empty() || maxCount == 0)
        return;

    keypoints.reserve(maxCount);
    for (size_t i : pickSpreadKeypoints(allKps, maxCount, frame.width, frame.height)) {
        keypoints.push_back(allKps[i]);
        descriptors.push_back(allDesc.row((int)i));   // copies the row
    }
}

double nearestAppearanceDistance(const FeatureDb &db, const glm::vec3 &place,
                                 const cv::Mat &descriptor)
{
    double nearest = 1e30;   // sentinel: farther than any real SIFT pair
    for (size_t i = 0; i < db.anchors.size(); i++) {
        if (db.anchors[i] != place)
            continue;
        nearest = std::min(nearest, cv::norm(db.descriptors.row((int)i),
                                             descriptor, cv::NORM_L2));
    }
    return nearest;
}

bool hasAppearanceWithin(const FeatureDb &db, const glm::vec3 &place,
                         const cv::Mat &descriptor, double maxDistance)
{
    return nearestAppearanceDistance(db, place, descriptor) <= maxDistance;
}

bool resemblesAnyAnchoredPoint(const FeatureDb &db, const cv::Mat &descriptor)
{
    for (int i = 0; i < db.descriptors.rows; i++)
        if (cv::norm(db.descriptors.row(i), descriptor) <= kDuplicateSuggestionDistance)
            return true;
    return false;
}

// The audit's expensive half: place pairs whose closest rows sit under the
// bar. `rows[p]` lists the descriptor rows owned by place p.
static size_t countConfusablePairs(const FeatureDb &db,
                                   const std::vector<std::vector<int>> &rows,
                                   const std::vector<glm::vec3> &places,
                                   float &worstApart)
{
    size_t confusable = 0;
    for (size_t i = 0; i < places.size(); i++)
        for (size_t j = i + 1; j < places.size(); j++) {
            double best = 1e30;
            for (int a : rows[i])
                for (int b : rows[j])
                    best = std::min(best, cv::norm(db.descriptors.row(a),
                                                   db.descriptors.row(b), cv::NORM_L2));
            if (best <= kMaxDescriptorDistance) {
                confusable++;
                worstApart = std::max(worstApart, glm::distance(places[i], places[j]));
            }
        }
    return confusable;
}

void reportDbAudit(const FeatureDb &db)
{
    if (db.empty())
        return;

    const std::vector<glm::vec3> places = db.places();
    std::vector<std::vector<int>> rows(places.size());
    for (int r = 0; r < (int)db.anchors.size(); r++)
        for (size_t p = 0; p < places.size(); p++)
            if (db.anchors[(size_t)r] == places[p]) {
                rows[p].push_back(r);
                break;
            }

    std::vector<size_t> perPlace;
    perPlace.reserve(rows.size());
    for (const std::vector<int> &list : rows)
        perPlace.push_back(list.size());
    std::sort(perPlace.begin(), perPlace.end());

    float worstApart = 0.0f;
    const size_t confusable = countConfusablePairs(db, rows, places, worstApart);

    std::cout << "FEATURES: audit -- " << places.size() << " places, "
              << db.anchors.size() << " appearance rows (per place "
              << perPlace.front() << "/" << perPlace[perPlace.size() / 2] << "/"
              << perPlace.back() << " min/med/max); ";
    if (confusable == 0)
        std::cout << "no confusable place pairs" << std::endl;
    else
        std::cout << confusable << " confusable place pair(s) under the "
                  << (int)kMaxDescriptorDistance << " bar (worst "
                  << (int)worstApart << " units apart)" << std::endl;
}

// One correspondence per PLACE -- the closest match wins. `anchors` repeats a
// point once per appearance, so several rows can answer for the same place,
// and keeping two would put one place at two pixels -- a contradiction PnP
// would quietly average. bestDistance stays parallel to correspondences; both
// are appended to in place.
static void keepBestMatchPerPlace(const std::vector<cv::DMatch> &matches,
                                  const FeatureDb &db,
                                  const std::vector<cv::KeyPoint> &keypoints,
                                  const FramePixels &frame,
                                  std::vector<Correspondence> &correspondences,
                                  std::vector<float> &bestDistance)
{
    for (const cv::DMatch &match : matches) {
        // A cross-checked match is still only a nearest neighbour.
        if (match.distance > kMaxDescriptorDistance)
            continue;

        const glm::vec3 &place = db.anchors[match.queryIdx];
        const cv::Point2f &pt = keypoints[match.trainIdx].pt;
        const Correspondence pair{ place,   // keypoint pixel -> the [0,1] fraction
                                   glm::vec2(pt.x / frame.width, pt.y / frame.height) };

        // This place's slot, if it has one.
        size_t existing = correspondences.size();   // one past the end = no slot yet
        for (size_t i = 0; i < correspondences.size(); i++)
            if (correspondences[i].worldPos == place) {
                existing = i;
                break;
            }

        if (existing == correspondences.size()) {
            correspondences.push_back(pair);
            bestDistance.push_back(match.distance);
        } else if (match.distance < bestDistance[existing]) {
            correspondences[existing] = pair;
            bestDistance[existing]    = match.distance;
        }
    }
}

// The calibration readout for the cap above: a healthy capture reads tight, a
// median hugging the cap means the list is best-of-noise. `distances` arrives
// by value -- the report sorts its own copy.
static void reportAcceptedMatches(size_t keypointCount,
                                  const std::vector<Correspondence> &correspondences,
                                  std::vector<float> distances)
{
    if (correspondences.empty()) {
        std::cout << "FEATURES: 0 anchors matched (from " << keypointCount
                  << " keypoints in the frame)" << std::endl;
        return;
    }
    std::sort(distances.begin(), distances.end());
    std::cout << "FEATURES: " << correspondences.size() << " anchors matched (from "
              << keypointCount << " keypoints; accepted distances "
              << (int)distances.front() << "-" << (int)distances.back()
              << ", median " << (int)distances[distances.size() / 2] << ")" << std::endl;
}

// Where the in-frame anchors went: reportAcceptedMatches shows what survived,
// this shows what did not, split by which filter took it. A place the cross-
// check paired with nothing is an appearance-coverage question (read against
// the "in frame" count printed just before the capture); a place whose every
// mutual match sat above the bar is the bar's, and its nearest miss says how
// close it came. Silent when every place matched.
static void reportMatchLosses(const std::vector<cv::DMatch> &matches, const FeatureDb &db)
{
    // Per-place best mutual distance, at any quality -- the pre-bar view.
    std::vector<glm::vec3> mutualPlaces;
    std::vector<float> best;
    for (const cv::DMatch &match : matches) {
        const glm::vec3 &place = db.anchors[match.queryIdx];
        size_t i = 0;
        while (i < mutualPlaces.size() && mutualPlaces[i] != place)
            i++;
        if (i == mutualPlaces.size()) {
            mutualPlaces.push_back(place);
            best.push_back(match.distance);
        } else if (match.distance < best[i]) {
            best[i] = match.distance;
        }
    }

    std::vector<float> nearMisses;   // places the bar alone cost
    for (float d : best)
        if (d > kMaxDescriptorDistance)
            nearMisses.push_back(d);

    const size_t total = db.places().size();
    const size_t noMutual = total - mutualPlaces.size();
    if (noMutual == 0 && nearMisses.empty())
        return;

    std::sort(nearMisses.begin(), nearMisses.end());
    std::cout << "        (losses: " << noMutual << " of " << total
              << " places had no mutual match in this frame";
    if (!nearMisses.empty()) {
        std::cout << "; " << nearMisses.size() << " more matched only above the "
                  << (int)kMaxDescriptorDistance << " bar -- nearest misses";
        for (size_t i = 0; i < nearMisses.size() && i < 3; i++)
            std::cout << (i ? ", " : " ") << (int)nearMisses[i];
    }
    std::cout << ")" << std::endl;
}

std::vector<Correspondence> matchFeaturesToDb(const FeatureDb &db, const FramePixels &frame)
{
    std::vector<Correspondence> correspondences;
    if (db.empty())
        return correspondences;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detectAllFeatures(frame, keypoints, descriptors);
    if (descriptors.empty()) {
        std::cout << "FEATURES: no keypoints in the current view" << std::endl;
        return correspondences;
    }

    // DATABASE -> FRAME, cross-checked; the header explains why that direction.
    //
    // Cross-checking rather than Lowe's ratio test: the ratio test only passes
    // where a feature is locally unique, and on shading-driven terrain every
    // ridge shoulder resembles every other, so it discarded most of the anchors
    // genuinely in frame. Mutual agreement leaves outlier rejection to RANSAC.
    cv::BFMatcher matcher(cv::NORM_L2, /*crossCheck=*/true);
    std::vector<cv::DMatch> matches;
    matcher.match(db.descriptors, descriptors, matches);

    std::vector<float> bestDistance;   // parallel to correspondences
    keepBestMatchPerPlace(matches, db, keypoints, frame, correspondences, bestDistance);
    reportAcceptedMatches(keypoints.size(), correspondences, bestDistance);
    reportMatchLosses(matches, db);
    return correspondences;
}

std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight,
                                                 std::optional<size_t> minInliers,
                                                 std::vector<Correspondence> *inliersOut,
                                                 bool *pinnedFloorNoted)
{
    const std::vector<Correspondence> correspondences = matchFeaturesToDb(db, frame);

    // Scales with what this frame MATCHED, not with the database: a frame sees
    // only the anchors of nearby views, so any fraction of the total goes
    // unmeetable once the database outgrows one frame. A sixth, not a quarter,
    // because the pool is polluted -- lookalikes roughly double it, and a
    // quarter then demanded 7-11 where genuine consensus measured 6-9. The
    // clamp's kMinConsensus holds the safety line (every accepted-wrong pose
    // measured had exactly 5 inliers).
    const size_t consensus = std::clamp(minInliers.value_or(correspondences.size() / 6),
                                        kMinConsensus, kMaxConsensus);
    if (correspondences.size() < consensus) {
        std::cout << "FEATURES: not enough to solve -- " << consensus
                  << " agreeing matches are needed; move toward the anchored views"
                  << std::endl;
        return std::nullopt;
    }

    // A pinned floor does not scale with the pool, and coalitions do: measured
    // once at a floor of 5 against 30-match pools, five confident poses came
    // out 60-436 units wrong. Warn once per PIN -- the first sighting teaches,
    // thirty repeats bury the log -- and the pinned value still rules. The
    // caller owns the flag, so a new mode entry with a new pin warns again.
    if (pinnedFloorNoted && !*pinnedFloorNoted && minInliers &&
        correspondences.size() > 4 * consensus) {
        *pinnedFloorNoted = true;
        std::cout << "FEATURES: note -- " << correspondences.size()
                  << " matches against a pinned floor of " << consensus
                  << ": lookalike coalitions can seat a floor this small; Enter at"
                     " the F prompt scales it to a sixth of the pool" << std::endl;
    }

    std::vector<int> inlierIndices;
    const std::optional<Waypoint> pose =
        computeCameraPoseRansac(correspondences, fov, viewportWidth, viewportHeight,
                                (int)consensus, 0.0f,
                                inliersOut ? &inlierIndices : nullptr);
    if (pose && inliersOut) {
        inliersOut->clear();
        inliersOut->reserve(inlierIndices.size());
        for (int i : inlierIndices)
            inliersOut->push_back(correspondences[(size_t)i]);
    }
    return pose;
}
