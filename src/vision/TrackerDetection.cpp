#include "TrackerDetection.h"

#include <cstdlib>   // abs

// Per-channel distance within which a pixel matches a tracker's palette color.
// The capture pass draws flat colors, so values should match exactly; the
// slack only absorbs rounding when the driver quantizes the float color to 8 bits.
static const int channelTolerance = 3;

// Blobs smaller than this are treated as "not visible": a sphere reduced to a
// couple of pixels yields a centroid too noisy to feed PnP.
static const int minBlobPixels = 6;

std::vector<std::optional<glm::vec2>>
findTrackerCentroids(const FramePixels &frame, const std::vector<Tracker> &trackers)
{
    // One accumulator per tracker. The centroid is the mean position of the
    // blob's pixels -- for a rendered sphere that is (to sub-pixel accuracy)
    // the projection of its center, the 3D point the correspondence pairs it with.
    struct Blob { long count = 0; double sumX = 0.0, sumY = 0.0; };
    std::vector<Blob> blobs(trackers.size());

    // 8-bit quantization of each palette color, computed once.
    std::vector<glm::ivec3> palette;
    palette.reserve(trackers.size());
    for (const Tracker &tracker : trackers)
        palette.push_back(glm::ivec3(tracker.color * 255.0f + 0.5f));

    // Single pass over the frame, classifying each pixel against the palette.
    // The black background fails every test: each palette color has at least
    // one full-intensity channel.
    for (int y = 0; y < frame.height; y++) {
        for (int x = 0; x < frame.width; x++) {
            const unsigned char *px = frame.at(x, y);
            for (size_t i = 0; i < trackers.size(); i++) {
                if (std::abs(px[0] - palette[i].r) <= channelTolerance &&
                    std::abs(px[1] - palette[i].g) <= channelTolerance &&
                    std::abs(px[2] - palette[i].b) <= channelTolerance) {
                    blobs[i].count++;
                    blobs[i].sumX += x;
                    blobs[i].sumY += y;
                    break;   // palette colors are far apart: one pixel, one tracker
                }
            }
        }
    }

    std::vector<std::optional<glm::vec2>> centroids(trackers.size());
    for (size_t i = 0; i < trackers.size(); i++) {
        if (blobs[i].count >= minBlobPixels)
            centroids[i] = glm::vec2(blobs[i].sumX / blobs[i].count,
                                     blobs[i].sumY / blobs[i].count);
    }
    return centroids;
}
