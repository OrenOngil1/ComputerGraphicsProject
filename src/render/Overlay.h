#pragma once

#include <vector>
#include <glm/glm.hpp>

#include <Shader.h>

#include "../core/Camera.h"

void renderPath(const std::vector<glm::vec3> &pathPoints, Shader &shader, const glm::mat4 &mvp);

// Draws recorded camera positions as dots: green for the record the player
// camera is on (position match), red for the rest. State-independent -- the
// highlight always tracks the camera, with no separate cursor to keep in sync.
void renderCameraRecords(const std::vector<CameraRecord> &cameraRecords, const glm::vec3 &cameraPos, Shader &shader, const glm::mat4 &mvp);
