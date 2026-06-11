#pragma once

#include <vector>
#include <glm/glm.hpp>

struct Vertex {
    glm::vec3 position;
    glm::vec3 color;
};

struct PickedPoint {
    glm::vec3 worldPos;
    glm::vec2 imagePos;
};

// Grid dimensions are VERTEX COUNTS (cols along x, rows along z), not world
// sizes -- vertices[z * cols + x] is the vertex at grid cell (x, z).
struct Mesh {
    int cols, rows;
    std::vector<Vertex> vertices;
};