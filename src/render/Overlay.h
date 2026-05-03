#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "../core/Scene.h"
#include "../core/Camera.h"

void renderPath(const std::vector<glm::vec3> &pathPoints);

void renderCameraRecords(const glm::vec3 &playerPosition, const std::vector<CameraRecord> &cameraRecords);

void renderPickedPointGlobal(const std::vector<glm::vec3> &pickedPoints, const Mesh &mesh);

void renderPickedPointPlayer(const std::vector<glm::vec3> &pickedPoints, const Mesh &mesh);