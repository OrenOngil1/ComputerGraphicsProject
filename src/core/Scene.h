#pragma once

#include <cassert>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// One terrain grid point.
struct Vertex {
    glm::vec3 position;   // uncentered grid coords: x = col, z = row, y = height
    glm::vec3 color;      // terrain color from the elevation ramp
    // Defaulted to +Y: the loader fills positions first, normals in a later pass.
    glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
};

// One 2D-3D correspondence for PnP.
struct Correspondence {
    glm::vec3 worldPos;   // the 3D point, centered world space (see Mesh::center)

    // The 2D observation: a fraction of the viewport, [0,1] x [0,1], origin
    // top-left -- NOT pixels; denormalize via imagePixels() at the OpenCV
    // boundary so points and intrinsics agree. A fraction is only faithful when
    // captured and solved at the same aspect ratio (a resize re-scales its
    // horizontal half): fine for TRACKERS / FEATURE MATCHING, which do both in
    // one frame; PICK spans resizes, so it stores an aspect-invariant viewing
    // ray instead (see PickState::Observation).
    glm::vec2 imagePos;

    // imagePos as pixels in a viewport of the given size.
    glm::vec2 imagePixels(int viewportWidth, int viewportHeight) const
    {
        return imagePos * glm::vec2((float)viewportWidth, (float)viewportHeight);
    }
};

// A TRACKERS-mode landmark: a colored sphere resting on the terrain.
struct Tracker {
    glm::vec3 center;   // centered world space -- the 3D half of the correspondence
    float radius;       // world units, scaled to the terrain at placement
    glm::vec3 color;    // unique flat color -- the key the blob detector matches

    // Places the renderer's shared unit sphere: scale to radius, move to center.
    glm::mat4 modelMatrix() const
    {
        return glm::scale(glm::translate(glm::mat4(1.0f), center), glm::vec3(radius));
    }
};

// The terrain height grid, as loaded (uncentered).
struct Mesh {
    int cols, rows;                 // VERTEX counts along x resp. z, not world sizes
    std::vector<Vertex> vertices;   // vertices[z * cols + x] = vertex at cell (x, z)

    // The centering offset the renderer bakes into the uploaded vertices; every
    // consumer mapping a stored vertex into the rendered world must subtract it.
    glm::vec3 center() const { return { cols / 2.0f, 0.0f, rows / 2.0f }; }

    // A stored vertex's position in the rendered (centered) world.
    // Precondition: vertexId < vertices.size() (callers validate picked ids).
    glm::vec3 worldPos(size_t vertexId) const
    {
        assert(vertexId < vertices.size());
        return vertices[vertexId].position - center();
    }
};

// A CPU-side copy of a rendered viewport -- the renderer's hand-off to vision.
// Rows are stored TOP-DOWN (row 0 = top scanline), matching cursor coords: the
// renderer flips OpenGL's bottom-up read-back on the way out, consumers never re-flip.
struct FramePixels {
    int width = 0, height = 0;        // in pixels
    std::vector<unsigned char> rgb;   // tightly packed, width * height * 3, row-major from the top

    // The RGB triple at image coords (x, y) -- y grows downward.
    const unsigned char *at(int x, int y) const { return &rgb[((size_t)y * width + x) * 3]; }
};
