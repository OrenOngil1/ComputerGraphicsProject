// Headless sanity checks for computeNormals -- no window, no GL context.

#include <iostream>

#include <glm/glm.hpp>

#include "check.h"
#include "../src/loader/TerrainLoader.h"

// A cols x rows grid with height h(x, z), matching the loader's layout:
// vertices[z * cols + x] sits at world (x, h, z).
template <typename HeightFn>
static Mesh makeGrid(int cols, int rows, HeightFn h)
{
    Mesh mesh;
    mesh.cols = cols;
    mesh.rows = rows;
    mesh.vertices.resize((size_t)cols * rows);
    for (int z = 0; z < rows; z++)
        for (int x = 0; x < cols; x++)
            mesh.vertices[(size_t)z * cols + x].position =
                glm::vec3((float)x, h(x, z), (float)z);
    return mesh;
}

void testNormals()
{
    std::cout << "computeNormals:" << std::endl;

    // Flat ground: every normal is straight up, including the borders.
    Mesh flat = makeGrid(5, 5, [](int, int) { return 0.0f; });
    computeNormals(flat);
    bool allUp = true;
    for (const Vertex &v : flat.vertices)
        allUp = allUp && glm::length(v.normal - glm::vec3(0, 1, 0)) < 1e-5f;
    check(allUp, "flat grid: all normals point straight up");

    // Tilted plane y = x/2: the surface tangents are (1, 1/2, 0) and (0, 0, 1),
    // whose cross product normalizes to (-1, 2, 0) / sqrt(5). Central
    // differences are exact on a plane, so interior vertices must match it.
    Mesh tilted = makeGrid(5, 5, [](int x, int) { return 0.5f * x; });
    computeNormals(tilted);
    const glm::vec3 expected = glm::normalize(glm::vec3(-1.0f, 2.0f, 0.0f));
    const glm::vec3 got = tilted.vertices[2 * 5 + 2].normal;   // center vertex
    check(glm::length(got - expected) < 1e-5f,
          "tilted plane: interior normal matches the analytic one");
}
