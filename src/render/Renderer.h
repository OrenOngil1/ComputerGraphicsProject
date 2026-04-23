#pragma once

#include <iostream>
#include <memory>

#include "../core/AppState.h"

void renderGlobalOverlay(const AppState &appState);

void renderPlayerOverlay(const AppState &appState);

void setupCamera(Camera &camera);

void renderTerrainByColor(const Mesh &mesh, const std::vector<glm::vec3> &colorCodes);

void renderTerrain(const Mesh &mesh);