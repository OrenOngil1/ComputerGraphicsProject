#pragma once

#include <algorithm>
#include <cassert>
#include <optional>
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

    // Model matrix placing the renderer's shared unit-sphere mesh at this tracker.
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

    // The surface height above a point in the rendered (centered) world's
    // horizontal plane, bilinearly interpolated across the cell it falls in.
    // Empty outside the grid -- there is no surface there to be above or below.
    // Goes through center() like worldPos, so it cannot drift from the
    // uploaded geometry.
    std::optional<float> heightAt(float worldX, float worldZ) const
    {
        if (cols < 2 || rows < 2 || vertices.size() < (size_t)cols * rows)
            return std::nullopt;

        const glm::vec3 offset = center();
        const float gx = worldX + offset.x;   // grid coords: vertex (x, z) sits at (x, z)
        const float gz = worldZ + offset.z;

        // Written as a positive test so NaN (which fails every comparison)
        // falls out as "outside" rather than indexing on garbage.
        if (!(gx >= 0.0f && gz >= 0.0f && gx <= (float)(cols - 1) && gz <= (float)(rows - 1)))
            return std::nullopt;

        const int x0 = std::min((int)gx, cols - 2);
        const int z0 = std::min((int)gz, rows - 2);
        const float fx = gx - (float)x0;
        const float fz = gz - (float)z0;

        auto h = [&](int x, int z) { return vertices[(size_t)z * cols + x].position.y; };
        const float front = glm::mix(h(x0, z0),     h(x0 + 1, z0),     fx);
        const float back  = glm::mix(h(x0, z0 + 1), h(x0 + 1, z0 + 1), fx);
        return glm::mix(front, back, fz);
    }
};

// March a ray until it passes below the terrain surface; returns the distance
// along it, or empty if it never does within maxDistance (aimed at the sky, or
// off the edge of the grid). `dir` must be normalized.
//
// Fixed-step sampling then bisection, not an analytic intersection: a
// heightfield has no closed form, and the callers here (the view cone's reach)
// want a good indicator, not sub-texel precision. A ridge thinner than `step`
// can be stepped over -- harmless at that use, but worth knowing before reusing
// this for anything that must not miss.
inline std::optional<float> raycastTerrain(const Mesh &mesh, const glm::vec3 &origin,
                                           const glm::vec3 &dir, float maxDistance,
                                           float step)
{
    if (step <= 0.0f || maxDistance <= 0.0f)
        return std::nullopt;

    // Is the ray below the surface at distance t? Empty where it is outside
    // the grid, which is neither above nor below.
    auto below = [&](float t) -> std::optional<bool> {
        const glm::vec3 p = origin + dir * t;
        if (const std::optional<float> surface = mesh.heightAt(p.x, p.z))
            return p.y <= *surface;
        return std::nullopt;
    };

    const int steps = (int)(maxDistance / step);
    float lastAbove = 0.0f;

    for (int i = 1; i <= steps; i++) {
        const float t = step * (float)i;
        const std::optional<bool> sample = below(t);
        if (!sample)
            continue;          // off the grid: keep going, the ray may re-enter
        if (!*sample) {
            lastAbove = t;
            continue;
        }

        // Crossed the surface somewhere in (lastAbove, t]; bisect to refine.
        float lo = lastAbove, hi = t;
        for (int j = 0; j < 16; j++) {
            const float mid = 0.5f * (lo + hi);
            const std::optional<bool> midSample = below(mid);
            if (midSample && *midSample)
                hi = mid;
            else
                lo = mid;
        }
        return hi;
    }
    return std::nullopt;
}

// A CPU-side copy of a rendered viewport -- the renderer's hand-off to vision.
// Rows are stored TOP-DOWN (row 0 = top scanline), matching cursor coords: the
// renderer flips OpenGL's bottom-up read-back on the way out, consumers never re-flip.
struct FramePixels {
    int width = 0, height = 0;        // in pixels
    std::vector<unsigned char> rgb;   // tightly packed, width * height * 3, row-major from the top

    // The RGB triple at image coords (x, y) -- y grows downward.
    const unsigned char *at(int x, int y) const { return &rgb[((size_t)y * width + x) * 3]; }
};
