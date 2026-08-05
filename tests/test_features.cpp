// Headless sanity checks for detectSpreadFeatures (Mode D's suggestion step) --
// pure OpenCV on synthetic frames, no window, no GL context. Two invariants:
//
//   row alignment  descriptor row i must belong to keypoint i, or every
//                  hand-placed anchor pairs with the wrong 3D point
//   spreading      the suggestions must land in different PLACES, not on the
//                  same corner several times over -- a human cannot tell
//                  clustered dots apart on the map, and clustered points barely
//                  constrain a pose however well they are placed

#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "check.h"
#include "../src/vision/FeatureMatching.h"

namespace {

// Paint a black square of half-width `half` centered on (cx, cy).
void paintSquare(FramePixels &frame, int cx, int cy, int half)
{
    for (int y = cy - half; y <= cy + half; y++)
        for (int x = cx - half; x <= cx + half; x++) {
            unsigned char *px = &frame.rgb[((size_t)y * frame.width + x) * 3];
            px[0] = px[1] = px[2] = 0;
        }
}

FramePixels whiteFrame(int width, int height)
{
    FramePixels frame;
    frame.width = width;
    frame.height = height;
    frame.rgb.assign((size_t)width * height * 3, 255);
    return frame;
}

// Soften the painted squares. SIFT is a blob-and-gradient detector: on
// razor-edged binary squares it collapses each square into a stack of
// duplicate keypoints (or a phantom blob BETWEEN squares), while a small blur
// turns every corner into a clean, distinct detection. The blur also makes
// the fixture honest -- the production input is a smooth-shaded terrain
// render, which never contains a binary step edge.
void soften(FramePixels &frame)
{
    cv::Mat rgb(frame.height, frame.width, CV_8UC3, frame.rgb.data());
    cv::GaussianBlur(rgb, rgb, cv::Size(), 2.0);
}

// Four well-separated squares: contrast corners for the detector to find,
// kept well clear of the border (descriptors need a patch around each).
FramePixels makeFrame()
{
    FramePixels frame = whiteFrame(256, 256);
    const int centers[][2] = { { 70, 70 }, { 180, 60 }, { 90, 170 }, { 190, 180 } };
    for (const auto &c : centers)
        paintSquare(frame, c[0], c[1], 12);
    soften(frame);
    return frame;
}

// Six squares crowded into one corner of the frame plus three far apart. By
// corner strength alone the crowd wins -- they are the same shape as the rest,
// and there are twice as many of them -- so a top-N-by-response pick lands
// several suggestions inside one 60-pixel patch. This is the frame that tells
// the two behaviours apart.
FramePixels makeClusteredFrame()
{
    FramePixels frame = whiteFrame(400, 400);
    const int cluster[][2] = { { 60, 60 }, { 85, 60 }, { 110, 60 },
                               { 60, 85 }, { 85, 85 }, { 110, 85 } };
    for (const auto &c : cluster)
        paintSquare(frame, c[0], c[1], 5);

    const int isolated[][2] = { { 330, 80 }, { 80, 330 }, { 330, 330 } };
    for (const auto &c : isolated)
        paintSquare(frame, c[0], c[1], 12);
    soften(frame);
    return frame;
}

cv::Mat grayOf(const FramePixels &frame)
{
    cv::Mat rgb(frame.height, frame.width, CV_8UC3,
                const_cast<unsigned char *>(frame.rgb.data()));
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    return gray;
}

bool inBox(const cv::Point2f &p, float x0, float y0, float x1, float y1)
{
    return p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1;
}

}  // namespace

void testFeatures()
{
    std::cout << "detectSpreadFeatures:" << std::endl;

    const FramePixels frame = makeFrame();

    // Uncapped detection: four squares offer plenty of corners.
    std::vector<cv::KeyPoint> all;
    cv::Mat allDesc;
    detectSpreadFeatures(frame, 1000, all, allDesc);
    check(all.size() >= 4, "synthetic corners: keypoints are found");
    check((int)all.size() == allDesc.rows, "one descriptor row per keypoint");

    bool ordered = true;
    for (size_t i = 1; i < all.size(); i++)
        ordered = ordered && all[i - 1].response >= all[i].response;
    check(ordered, "keypoints are ordered by response, strongest first");

    // Row alignment, against an independent oracle: ask SIFT to recompute
    // descriptors for exactly the keypoints that came back, and require each
    // returned row to equal the recomputation for ITS keypoint. Checking it
    // this way rather than against the head of the ranking is what spreading
    // forces -- the result is a subset of the ranking, not a prefix of it --
    // and it pins the failure that matters: a descriptor filed under someone
    // else's keypoint, which anchors a database row to the wrong 3D point
    // without anything looking wrong. (SIFT may hand the keypoints back in a
    // different order, so the pairing is matched up by keypoint.)
    std::vector<cv::KeyPoint> top;
    cv::Mat topDesc;
    detectSpreadFeatures(frame, 3, top, topDesc);

    std::vector<cv::KeyPoint> recomputedKps = top;
    cv::Mat recomputedDesc;
    cv::SIFT::create(1000)->compute(grayOf(frame), recomputedKps, recomputedDesc);

    bool aligned = top.size() == 3 && topDesc.rows == 3 &&
                   recomputedKps.size() == top.size() &&
                   recomputedDesc.rows == topDesc.rows;
    // Matched by position, octave AND angle: SIFT emits one keypoint per
    // dominant orientation, so a location can host several twins whose
    // descriptors legitimately differ. And near-equality rather than bit
    // equality: OpenCV's provided-keypoints path rebuilds a slightly
    // shallower pyramid, so a recomputed descriptor lands ~10 (L2) from the
    // original on a descriptor of norm 512. A row misfiled under someone
    // else's keypoint would sit hundreds away, so the invariant that
    // actually pins alignment is nearest-recomputation-wins, with an
    // absolute bound as the sanity belt.
    for (size_t i = 0; aligned && i < top.size(); i++) {
        int match = -1;
        for (size_t j = 0; j < recomputedKps.size(); j++)
            if (recomputedKps[j].pt == top[i].pt && recomputedKps[j].octave == top[i].octave &&
                recomputedKps[j].angle == top[i].angle)
                match = (int)j;
        if (match < 0) {
            aligned = false;
            break;
        }
        const double own = cv::norm(recomputedDesc.row(match), topDesc.row((int)i),
                                    cv::NORM_L2);
        aligned = own < 50.0;
        for (int j = 0; aligned && j < recomputedDesc.rows; j++)
            if (j != match)
                aligned = own < cv::norm(recomputedDesc.row(j), topDesc.row((int)i),
                                         cv::NORM_L2);
    }
    check(aligned, "each descriptor row is SIFT's own descriptor for its own keypoint");

    // ── spreading ─────────────────────────────────────────────
    const FramePixels clustered = makeClusteredFrame();

    std::vector<cv::KeyPoint> spread;
    cv::Mat spreadDesc;
    detectSpreadFeatures(clustered, 4, spread, spreadDesc);
    check(spread.size() == 4 && spreadDesc.rows == 4,
          "clustered frame: four suggestions are still found");

    // The crowd holds six squares -- twenty-four corners -- inside one small
    // patch. At most one suggestion may come from there; the rest have to go
    // to the three isolated squares.
    int inCluster = 0;
    for (const cv::KeyPoint &kp : spread)
        if (inBox(kp.pt, 40.0f, 40.0f, 130.0f, 105.0f))
            inCluster++;
    check(inCluster <= 1, "at most one suggestion comes from the crowded patch");

    float closest = 1e9f;
    for (size_t i = 0; i < spread.size(); i++)
        for (size_t j = i + 1; j < spread.size(); j++)
            closest = std::min(closest, (float)cv::norm(spread[i].pt - spread[j].pt));
    check(closest >= 100.0f, "the suggestions are spread across the frame, not stacked");

    bool insideMargin = true;
    for (const cv::KeyPoint &kp : spread)
        insideMargin = insideMargin && inBox(kp.pt, 16.0f, 16.0f, 384.0f, 384.0f);
    check(insideMargin, "no suggestion sits on the frame's rim");

    // ── degenerate inputs ─────────────────────────────────────
    detectSpreadFeatures(frame, 0, top, topDesc);
    check(top.empty() && topDesc.empty(), "maxCount = 0: empty result");

    FramePixels blank = whiteFrame(64, 64);
    blank.rgb.assign(blank.rgb.size(), 128);
    detectSpreadFeatures(blank, 5, top, topDesc);
    check(top.empty() && topDesc.empty(), "featureless frame: empty result");
}
