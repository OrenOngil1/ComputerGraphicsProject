#pragma once

#include <vector>
#include <glm/glm.hpp>

typedef struct {
    glm::vec3 position;
    glm::vec3 color;
} Vertex;

typedef  struct {
    int width, height;
    std::vector<Vertex> vertices;
} Mesh;