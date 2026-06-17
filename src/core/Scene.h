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

// One 2D-3D correspondence for PnP: a 3D world point and the 2D image point it was
// observed at. Renamed from PickedPoint when imagePos became normalized -- the name
// now reads at every call site as a reminder that imagePos is a fraction, not pixels.
struct Correspondence {
    glm::vec3 worldPos;
    // The 2D observation as a fraction of the viewport, [0,1] x [0,1], origin
    // top-left -- NOT raw pixels. A pixel coordinate only means something against a
    // specific viewport size, so storing it normalized keeps a correspondence valid
    // across a resize: the viewport (and the camera intrinsics derived from it) can
    // change between when the point is captured and when PnP solves. Consumers
    // denormalize via imagePixels() against the CURRENT viewport at the OpenCV
    // boundary (see toCvPoints in Pnp.cpp) so the points and the intrinsics agree.
    glm::vec2 imagePos;

    // The one authority for denormalizing: the stored fraction back to pixels in a
    // viewport of the given size. Keeping it here means no consumer re-derives the
    // conversion and risks drifting from how producers normalized (pixel / size).
    glm::vec2 imagePixels(int viewportWidth, int viewportHeight) const
    {
        return imagePos * glm::vec2((float)viewportWidth, (float)viewportHeight);
    }
};

// A TRACKERS-mode landmark (Mode 3): a sphere resting on the terrain surface,
// identified by a unique flat color. The color doubles as the correspondence
// key -- the blob detector finds it in the rendered frame, and the center is
// the matching 3D point. center is in centered world space (the space the
// cameras live in), like Correspondence::worldPos.
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