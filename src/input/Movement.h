#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "../core/Camera.h"

void move(const bool keys[], float deltaTime, Camera &camera, std::vector<glm::vec3> &pathPoints, float terrainSize);