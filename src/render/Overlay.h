#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "../core/Camera.h"

void renderPath(const std::vector<glm::vec3> &pathPoints);

void renderCameraRecords(const std::vector<CameraRecord> &cameraRecords, size_t playbackIndex);