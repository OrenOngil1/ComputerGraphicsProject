#pragma once

#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "../core/Scene.h"   // FramePixels, Tracker

// Find each tracker's color blob in a captured detection frame (see
// Renderer::captureTrackersFrame) and return its centroid in image coords
// (origin top-left, y down -- the same convention as PnP's image points).
// Result index i corresponds to trackers[i]; empty means not visible in this
// frame (occluded, off-screen, or its blob is too small to trust). Pure pixel
// math -- no GL, no OpenCV -- so it is headlessly testable.
std::vector<std::optional<glm::vec2>>
findTrackerCentroids(const FramePixels &frame, const std::vector<Tracker> &trackers);
