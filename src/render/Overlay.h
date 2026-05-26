#pragma once

#include <vector>
#include <glm/glm.hpp>

#include <Shader.h>

#include "../core/Camera.h"

void renderPath(const std::vector<glm::vec3> &pathPoints, Shader &shader, const glm::mat4 &mvp);

void renderCameraRecords(const std::vector<CameraRecord> &cameraRecords, size_t playbackIndex, Shader &shader, const glm::mat4 &mvp);
