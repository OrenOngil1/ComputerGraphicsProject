// Headless sanity checks for detectTopFeatures (Mode D's suggestion step) --
// pure OpenCV on a synthetic frame, no window, no GL context. The load-bearing
// invariant is row alignment: descriptor row i must belong to keypoint i, or
// every hand-placed anchor in the feature database pairs with the wrong point.

#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "check.h"
#include "../src/vision/FeatureMatching.h"

// A white frame with black squares: maximal-contrast corners for ORB to find,
// kept well clear of the border (descriptors need a 31-pixel patch around
// each keypoint).
static FramePixels makeFrame()
{
    FramePixels frame;
    frame.width = 256;
    frame.height = 256;
    frame.rgb.assign((size_t)frame.width * frame.height * 3, 255);

    const int centers[][2] = { { 70, 70 }, { 180, 60 }, { 90, 170 }, { 190, 180 } };
    for (const auto &c : centers)
        for (int y = c[1] - 12; y <= c[1] + 12; y++)
            for (int x = c[0] - 12; x <= c[0] + 12; x++) {
                unsigned char *px = &frame.rgb[((size_t)y * frame.width + x) * 3];
                px[0] = px[1] = px[2] = 0;
            }
    return frame;
}

void testFeatures()
{
    std::cout << "detectTopFeatures:" << std::endl;

    const FramePixels frame = makeFrame();

    // Uncapped detection: four squares offer plenty of corners.
    std::vector<cv::KeyPoint> all;
    cv::Mat allDesc;
    detectTopFeatures(frame, 1000, all, allDesc);
    check(all.size() >= 4, "synthetic corners: keypoints are found");
    check((int)all.size() == allDesc.rows, "one descriptor row per keypoint");

    bool ordered = true;
    for (size_t i = 1; i < all.size(); i++)
        ordered = ordered && all[i - 1].response >= all[i].response;
    check(ordered, "keypoints are ordered by response, strongest first");

    // Capped detection must return the head of the same ranking, each keypoint
    // still paired with its own descriptor row.
    std::vector<cv::KeyPoint> top;
    cv::Mat topDesc;
    detectTopFeatures(frame, 3, top, topDesc);
    bool aligned = top.size() == 3 && topDesc.rows == 3;
    for (size_t i = 0; aligned && i < top.size(); i++)
        aligned = top[i].pt == all[i].pt &&
                  cv::norm(topDesc.row((int)i), allDesc.row((int)i), cv::NORM_HAMMING) == 0;
    check(aligned, "maxCount = 3: the top 3 keypoints with their aligned descriptors");

    detectTopFeatures(frame, 0, top, topDesc);
    check(top.empty() && topDesc.empty(), "maxCount = 0: empty result");

    FramePixels blank;
    blank.width = 64;
    blank.height = 64;
    blank.rgb.assign((size_t)blank.width * blank.height * 3, 128);
    detectTopFeatures(blank, 5, top, topDesc);
    check(top.empty() && topDesc.empty(), "featureless frame: empty result");
}
