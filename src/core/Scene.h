#pragma once

#include <vector>
#include <glm/glm.hpp>

struct Vertex {
    glm::vec3 position;
    glm::vec3 color;
    // Surface normal for lighting, computed by the loader from the height
    // grid. Defaulted to straight up so partially built vertices (the loader
    // fills positions first, normals in a later pass) are never garbage.
    glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
};

struct PickedPoint {
    glm::vec3 worldPos;
    glm::vec2 imagePos;
};

// A TRACKERS-mode landmark (Mode 3): a sphere resting on the terrain surface,
// identified by a unique flat color. The color doubles as the correspondence
// key -- the blob detector finds it in the rendered frame, and the center is
// the matching 3D point. center is in centered world space (the space the
// cameras live in), like PickedPoint::worldPos.
struct Tracker {
    glm::vec3 center;
    float radius;
    glm::vec3 color;
};

// Grid dimensions are VERTEX COUNTS (cols along x, rows along z), not world
// sizes -- vertices[z * cols + x] is the vertex at grid cell (x, z).
struct Mesh {
    int cols, rows;
    std::vector<Vertex> vertices;

    // The rendered world is this mesh recentered on its middle: the renderer
    // bakes the -center() translation into the uploaded vertices, so every
    // consumer mapping a stored vertex into the world the cameras live in
    // (picking, trackers, feature anchors) must subtract the SAME center.
    // One authority here, so the render and vision sides can't drift apart.
    glm::vec3 center() const { return { cols / 2.0f, 0.0f, rows / 2.0f }; }
    glm::vec3 worldPos(size_t vertexId) const
    {
        return vertices[vertexId].position - center();
    }
};

// A CPU-side copy of a rendered viewport, the renderer's hand-off to the
// vision pipeline. Rows are stored TOP-DOWN (row 0 = top scanline), tightly
// packed RGB -- image convention, matching cursor coordinates. The renderer
// flips OpenGL's bottom-up read-back on the way out, so consumers (blob
// detection, OpenCV wrapping) never re-flip.
struct FramePixels {
    int width = 0, height = 0;
    std::vector<unsigned char> rgb;   // width * height * 3, row-major from the top

    // The RGB triple at image coords (x, y) -- y grows downward.
    const unsigned char *at(int x, int y) const { return &rgb[((size_t)y * width + x) * 3]; }
};