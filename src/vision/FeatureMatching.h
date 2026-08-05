#pragma once

#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../core/Scene.h"    // FramePixels
#include "../core/Camera.h"   // Waypoint

// The feature-matching mode's pre-phase product: SIFT descriptors, each anchored
// to a 3D terrain point the user hand-placed on the map. anchors[i] is the
// world-space point behind descriptor row i.
//
// A row is one APPEARANCE, not one place: the same hand-placed point is stored
// once per recorded view it can be seen in, because on this shading-driven
// terrain a feature's appearance moves with the viewpoint and a single
// appearance often fails to match from anywhere else. So `anchors` repeats
// positions, and its size is the appearance count rather than the number of
// points the user placed.
struct FeatureDb {
    cv::Mat descriptors;              // CV_32F, one 128-float SIFT descriptor per row
    std::vector<glm::vec3> anchors;   // centered world space, like Correspondence::worldPos

    bool empty() const { return anchors.empty(); }

    // The distinct places behind those appearances, in first-seen order --
    // what the user actually placed, for counting and for drawing on the map.
    std::vector<glm::vec3> places() const
    {
        std::vector<glm::vec3> distinct;
        for (const glm::vec3 &anchor : anchors) {
            bool seen = false;
            for (const glm::vec3 &place : distinct)
                seen = seen || place == anchor;   // exact: appearances copy one vec3
            if (!seen)
                distinct.push_back(anchor);
        }
        return distinct;
    }
};

// Pre-phase suggestion step: detect SIFT on the frame and return up to maxCount
// strong keypoints, SPREAD ACROSS THE FRAME, with their aligned descriptor rows.
// keypoints/descriptors are cleared first; descriptors has one row per keypoint.
//
// Strength alone is not enough here, because the suggestions are what a human
// is asked to anchor by hand. The strongest responses cluster -- several fire
// on one corner, a few pixels apart -- and a clustered set is bad twice over:
// the user cannot tell the dots apart on the map, and points crowded into one
// spot barely constrain a pose no matter how accurately they are placed.
// Spreading trades a little corner strength for the geometry PnP needs.
void detectSpreadFeatures(const FramePixels &frame, int maxCount,
                          std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

// Every SIFT feature in a frame -- the full population the run phase matches
// against, and the same detection the suggestions are drawn from, so pre-phase
// and run-phase descriptors are always comparable. Exposed because the build
// also needs it: once a point has been anchored, its appearance in the other
// recorded views is collected from their keypoints.
void detectAllFeatures(const FramePixels &frame,
                       std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

// Does `descriptor` look like the point already anchored at `place`? True when
// it is close enough to ANY appearance stored for that place.
//
// The gate on collecting a point's appearance from a second view: the geometry
// says the point should be at that pixel, and this says the view actually shows
// it there rather than something standing in front of it.
bool resemblesAnchoredPoint(const FeatureDb &db, const glm::vec3 &place,
                            const cv::Mat &descriptor);

// The run-phase match step alone: each database anchor this frame appears to
// contain, paired with the pixel it was found at (as a [0,1] fraction).
//
// Separate from the solve because this is where the mode's least visible
// failure lives. The match runs database -> frame, which makes two things true
// by construction: there is at most one correspondence per anchor, and no
// anchor is claimed to be in two places at once. Asked the other way round --
// once per keypoint -- a 20-anchor database can answer "yes" 140 times, and
// PnP then builds its consensus out of contradictions. See the .cpp.
std::vector<Correspondence> matchFeaturesToDb(const FeatureDb &db, const FramePixels &frame);

// Run-phase: match the frame against the database and solve the surviving
// 2D-3D pairs with RANSAC PnP. nullopt when there are too few confident
// matches or no consensus pose.
std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight);
