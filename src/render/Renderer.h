#pragma once

#include <iostream>
#include <memory>

#include "../core/AppState.h"

void renderGlobalOverlay(const AppState &appState);

void renderPlayerOverlay(const AppState &appState);

void setupCamera(Camera &camera);

void renderTerrain(const Mesh &mesh);