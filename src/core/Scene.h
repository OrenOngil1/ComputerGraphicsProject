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

struct Mesh {
    int width, height;
    std::vector<Vertex> vertices;
};