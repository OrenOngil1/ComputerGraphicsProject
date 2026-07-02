#include "GpuMesh.h"

#include <cmath>
#include <vector>

#include <glm/gtc/constants.hpp>

#include <VertexBufferLayout.h>

#include "PickEncoding.h"

// Upload interleaved vertices + triangle indices and record the layout in a
// VAO. attribSizes lists the float count of each attribute in push order,
// which maps to shader attribute locations: 0 = position, 1 = color,
// 2 = normal, 3 = pick-id color, where present.
static GpuMesh uploadBuffers(const std::vector<float> &verts,
                             const std::vector<unsigned int> &indices,
                             const std::vector<int> &attribSizes)
{
    int floatsPerVertex = 0;
    for (int size : attribSizes)
        floatsPerVertex += size;

    GpuMesh gpu;
    gpu.va = std::make_unique<VertexArray>();
    gpu.vb = std::make_unique<VertexBuffer>(verts.data(), verts.size() * sizeof(float));
    gpu.ib = std::make_unique<IndexBuffer>(indices.data(), indices.size() * sizeof(unsigned int));
    gpu.indexCount = (unsigned int)indices.size();
    gpu.vertexCount = verts.size() / floatsPerVertex;

    VertexBufferLayout layout;
    for (int size : attribSizes)
        layout.Push<float>(size);
    gpu.va->AddBuffer(*gpu.vb, layout);

    // Unbind so later binds can't accidentally modify these objects.
    gpu.va->Unbind();
    gpu.vb->Unbind();
    gpu.ib->Unbind();
    return gpu;
}

// Triangulate a rows x cols vertex grid (row-major storage): two triangles
// per cell, sharing the cell's diagonal:
//
//   i00 --- i01        first  triangle: i00 -> i10 -> i01
//    |  \    |         second triangle: i01 -> i10 -> i11
//   i10 --- i11
static std::vector<unsigned int> gridIndices(int rows, int cols)
{
    std::vector<unsigned int> indices;
    indices.reserve((size_t)(rows - 1) * (cols - 1) * 6);
    for (int r = 0; r < rows - 1; r++) {
        for (int c = 0; c < cols - 1; c++) {
            unsigned int i00 = r       * cols + c;
            unsigned int i10 = (r + 1) * cols + c;
            unsigned int i01 = i00 + 1;
            unsigned int i11 = i10 + 1;
            indices.push_back(i00); indices.push_back(i10); indices.push_back(i01);
            indices.push_back(i01); indices.push_back(i10); indices.push_back(i11);
        }
    }
    return indices;
}

// The centering translation is baked into the positions (Mesh::center is the
// shared definition every vertex->world consumer uses). Normals pass through
// untransformed -- translation doesn't change them.
GpuMesh uploadTerrain(const Mesh &mesh)
{
    const glm::vec3 center = mesh.center();

    std::vector<float> verts;
    verts.reserve(mesh.vertices.size() * 12);
    for (size_t i = 0; i < mesh.vertices.size(); i++) {
        const Vertex &v = mesh.vertices[i];
        verts.push_back(v.position.x - center.x);
        verts.push_back(v.position.y);
        verts.push_back(v.position.z - center.z);
        verts.push_back(v.color.r);
        verts.push_back(v.color.g);
        verts.push_back(v.color.b);
        verts.push_back(v.normal.x);
        verts.push_back(v.normal.y);
        verts.push_back(v.normal.z);
        // The vertex's pick-pass identity, baked here so the pick shader
        // carries no packing rule of its own (see PickEncoding.h).
        const glm::vec3 idColor = encodeVertexId((unsigned int)i);
        verts.push_back(idColor.r);
        verts.push_back(idColor.g);
        verts.push_back(idColor.b);
    }

    return uploadBuffers(verts, gridIndices(mesh.rows, mesh.cols),
                         { 3, 3, 3, 3 });   // position, color, normal, pick id
}

// The white vertex color satisfies the layout -- tracker draws always override
// it. On a unit sphere at the origin the outward normal IS the vertex
// position, and the per-tracker uniform scale + translation leave its
// direction intact, so it doubles as the world normal.
GpuMesh buildSphereMesh(int stacks, int sectors)
{
    std::vector<float> verts;
    verts.reserve((stacks + 1) * (sectors + 1) * 9);
    for (int i = 0; i <= stacks; i++) {
        // phi sweeps pole to pole, theta around the equator; the seam column
        // (j == sectors) duplicates j == 0 so the index grid can wrap simply.
        float phi = glm::pi<float>() * i / stacks;
        for (int j = 0; j <= sectors; j++) {
            float theta = 2.0f * glm::pi<float>() * j / sectors;
            float x = std::sin(phi) * std::cos(theta);
            float y = std::cos(phi);
            float z = std::sin(phi) * std::sin(theta);
            verts.push_back(x); verts.push_back(y); verts.push_back(z);
            verts.push_back(1.0f); verts.push_back(1.0f); verts.push_back(1.0f);
            verts.push_back(x); verts.push_back(y); verts.push_back(z);
        }
    }

    // The seam column makes the sphere a plain (stacks + 1) x (sectors + 1)
    // vertex grid, so gridIndices applies.
    return uploadBuffers(verts, gridIndices(stacks + 1, sectors + 1),
                         { 3, 3, 3 });   // position, color, normal
}
