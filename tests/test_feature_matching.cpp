// Headless checks for matchFeaturesToDb (Mode D's run-phase match step) --
// pure OpenCV on a synthetic frame, no window, no GL context.
//
// This pins a bug that was invisible from the outside. A six-anchor
// hand-built database, matched against a live frame, reported "140 confident
// matches" and RANSAC then found "14 inliers" among them -- with six anchors,
// at least eight of those inliers had to be wrong, so the consensus PnP
// trusted was provably contaminated and the pose came out hundreds of units
// off. The cause was direction: asking "does any anchor look like this pixel?"
// once per keypoint lets one anchor be claimed to be in dozens of places.
//
// The invariant below makes that impossible to reintroduce: an anchor is a
// place, and a place appears in a frame at most once.

#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "check.h"
#include "../src/vision/FeatureMatching.h"

namespace {

// Deterministic per-pixel noise: hundreds of keypoints, and no two of them
// alike, which is the population Lowe's ratio test is supposed to be judged
// against. A fixed seed keeps the check reproducible.
FramePixels makeTexturedFrame(int width, int height)
{
    FramePixels frame;
    frame.width = width;
    frame.height = height;
    frame.rgb.resize((size_t)width * height * 3);

    unsigned int seed = 12345u;
    for (size_t i = 0; i < (size_t)width * height; i++) {
        seed = seed * 1664525u + 1013904223u;
        const unsigned char v = (unsigned char)((seed >> 16) & 0xFF);
        frame.rgb[i * 3] = frame.rgb[i * 3 + 1] = frame.rgb[i * 3 + 2] = v;
    }
    return frame;
}

}  // namespace

void testFeatureMatching()
{
    std::cout << "matchFeaturesToDb:" << std::endl;

    const FramePixels frame = makeTexturedFrame(400, 600);

    // A database the size a human actually builds in one view: six anchors,
    // each a SIFT descriptor from this very frame paired with a distinct 3D
    // point. Matching it against the frame it came from is the friendliest
    // possible case -- and the one that used to produce 140 matches.
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detectSpreadFeatures(frame, 6, keypoints, descriptors);
    check(descriptors.rows == 6, "six descriptors were taken from the frame");

    FeatureDb db;
    db.descriptors = descriptors;
    for (int i = 0; i < descriptors.rows; i++)
        db.anchors.push_back(glm::vec3((float)i * 10.0f, 1.0f, (float)i * 7.0f));

    const std::vector<Correspondence> matches = matchFeaturesToDb(db, frame);

    check(matches.size() <= db.anchors.size(),
          "a six-anchor database can never yield more than six matches");

    bool distinct = true;
    for (size_t i = 0; i < matches.size(); i++)
        for (size_t j = i + 1; j < matches.size(); j++)
            distinct = distinct && matches[i].worldPos != matches[j].worldPos;
    check(distinct, "no anchor is claimed to be in two places at once");

    // The frame is the one the descriptors were taken from, so each anchor
    // that matches must be found at its own keypoint. This is what catches a
    // query/train index swap, which would otherwise pair every anchor with a
    // plausible-looking but arbitrary pixel.
    check(!matches.empty(), "matching a frame against its own descriptors finds anchors");
    bool atOrigin = true;
    for (const Correspondence &c : matches) {
        const size_t index = (size_t)(c.worldPos.x / 10.0f + 0.5f);
        atOrigin = atOrigin && index < keypoints.size();
        if (!atOrigin)
            break;
        const glm::vec2 expected(keypoints[index].pt.x / frame.width,
                                 keypoints[index].pt.y / frame.height);
        atOrigin = atOrigin && glm::length(c.imagePos - expected) < 1e-4f;
    }
    check(atOrigin, "each matched anchor lands on the keypoint it was built from");

    // ── several appearances of one place ──────────────────────
    // A place is stored once per recorded view it can be seen in, so the same
    // 3D point owns several descriptor rows. Two of its rows can win two
    // different keypoints, and PnP would quietly average the contradiction --
    // the result must collapse them back to one claim per place.
    FeatureDb multi;
    multi.descriptors = descriptors.clone();
    multi.anchors     = db.anchors;
    for (int i = 0; i < 3; i++) {                       // a second appearance for three places
        multi.descriptors.push_back(descriptors.row(i));
        multi.anchors.push_back(db.anchors[i]);
    }
    check(multi.anchors.size() == 9 && multi.places().size() == 6,
          "nine appearances of six places");

    const std::vector<Correspondence> collapsed = matchFeaturesToDb(multi, frame);
    check(collapsed.size() <= multi.places().size(),
          "extra appearances add no extra correspondences");
    bool stillDistinct = true;
    for (size_t i = 0; i < collapsed.size(); i++)
        for (size_t j = i + 1; j < collapsed.size(); j++)
            stillDistinct = stillDistinct && collapsed[i].worldPos != collapsed[j].worldPos;
    check(stillDistinct, "and no place is claimed to be in two spots at once");

    // ── resemblesAnchoredPoint ────────────────────────────────
    // The gate on collecting a place's appearance from another view: it must
    // still look like what the user anchored, or that view is showing something
    // standing in front of it.
    check(resemblesAnchoredPoint(db, db.anchors[2], descriptors.row(2)),
          "a place's own descriptor resembles it");
    check(!resemblesAnchoredPoint(db, db.anchors[2], descriptors.row(5)),
          "an unrelated feature's descriptor does not");

    // ── degenerate inputs ─────────────────────────────────────
    check(matchFeaturesToDb(FeatureDb{}, frame).empty(), "an empty database matches nothing");
    check(FeatureDb{}.places().empty(), "an empty database has no places");
}
