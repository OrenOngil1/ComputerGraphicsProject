#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "Camera.h"

void recordPathPoint(std::vector<glm::vec3> &pathPoints, const glm::vec3 &position);

void recordCamera(std::vector<CameraRecord> &cameraRecords, const Camera &camera);