// Headless sanity checks for findTrackerCentroids -- no window, no GL context.

#include <iostream>
#include <vector>

#include "check.h"
#include "../src/vision/TrackerDetection.h"

static void paintBlock(FramePixels &frame, int cx, int cy, int halfSize,
                       unsigned char r, unsigned char g, unsigned char b)
{
    for (int y = cy - halfSize; y <= cy + halfSize; y++)
        for (int x = cx - halfSize; x <= cx + halfSize; x++) {
            unsigned char *px = &frame.rgb[((size_t)y * frame.width + x) * 3];
            px[0] = r; px[1] = g; px[2] = b;
        }
}

void testCentroids()
{
    std::cout << "findTrackerCentroids:" << std::endl;

    // A blank 100x80 frame. The real capture holds lit terrain here; both are
    // background as far as the palette scan is concerned.
    FramePixels frame;
    frame.width = 100;
    frame.height = 80;
    frame.rgb.assign((size_t)frame.width * frame.height * 3, 0);

    // Paint exactly n contiguous pixels of one color in a row -- for exercising
    // the size floor at the boundary, where a square block can't hit odd counts.
    auto paintN = [&](int x0, int y0, int n, unsigned char r, unsigned char g, unsigned char b) {
        for (int k = 0; k < n; k++) {
            unsigned char *px = &frame.rgb[((size_t)y0 * frame.width + (x0 + k)) * 3];
            px[0] = r; px[1] = g; px[2] = b;
        }
    };

    // Red: a 5x5 block centered on (30, 40) -- centroid lands on the center.
    // Blue: a 2x2 block (4 px), under the 6-pixel minimum -- too small to trust.
    // Green: never painted -- not visible. Yellow: exactly 6 px (the floor) --
    // must be KEPT; Magenta: 5 px (one under) -- must be dropped. The yellow /
    // magenta pair pins the >=/> boundary of minBlobPixels, which a square-only
    // test (25 vs 4) leaves unexercised.
    paintBlock(frame, 30, 40, 2, 255, 0, 0);
    paintBlock(frame, 70, 20, 0, 0, 0, 255);
    paintBlock(frame, 71, 20, 0, 0, 0, 255);
    paintBlock(frame, 70, 21, 0, 0, 0, 255);
    paintBlock(frame, 71, 21, 0, 0, 0, 255);
    paintN(10, 60, 6, 255, 255, 0);   // yellow, exactly the floor
    paintN(10, 70, 5, 255, 0, 255);   // magenta, one under

    const std::vector<Tracker> trackers = {
        { glm::vec3(0), 1.0f, glm::vec3(1, 0, 0) },   // red
        { glm::vec3(0), 1.0f, glm::vec3(0, 1, 0) },   // green
        { glm::vec3(0), 1.0f, glm::vec3(0, 0, 1) },   // blue
        { glm::vec3(0), 1.0f, glm::vec3(1, 1, 0) },   // yellow
        { glm::vec3(0), 1.0f, glm::vec3(1, 0, 1) },   // magenta
    };
    std::vector<std::optional<glm::vec2>> centroids =
        findTrackerCentroids(frame, trackers);

    check(centroids[0] && glm::length(*centroids[0] - glm::vec2(30, 40)) < 1e-4f,
          "5x5 red blob: centroid at its center");
    check(!centroids[1], "absent green tracker: reported not visible");
    check(!centroids[2], "4-pixel blue blob: under the size floor, dropped");
    check(centroids[3] && glm::length(*centroids[3] - glm::vec2(12.5f, 60.0f)) < 1e-4f,
          "6-pixel yellow blob: exactly at the size floor, kept");
    check(!centroids[4], "5-pixel magenta blob: one under the floor, dropped");
}
