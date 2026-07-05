#pragma once

#include <memory>

#include <VertexArray.h>
#include <VertexBuffer.h>
#include <IndexBuffer.h>

#include "../core/Scene.h"

// A GPU-resident indexed mesh (VAO + VBO + IBO).
struct GpuMesh {
    std::unique_ptr<VertexArray> va;
    std::unique_ptr<VertexBuffer> vb;
    std::unique_ptr<IndexBuffer> ib;
    unsigned int indexCount = 0;    // fed to glDrawElements (GLsizei)
    size_t vertexCount = 0;         // for validating picked vertex ids
};

// Mesh construction only -- drawing these lives in Renderer. Both builders
// need a live GL context (they create GL objects through the vendored wrappers).

// The terrain's GPU buffers: interleaved position / color / normal / pick-id
// color, with the centering translation baked into the positions.
GpuMesh uploadTerrain(const Mesh &mesh);

// The unit-radius UV sphere at the origin that every tracker draw scales and
// translates into place.
GpuMesh buildSphereMesh(int stacks, int sectors);

// The unit cube the skybox is sampled through: position-only vertices, drawn
// with the viewer pinned at its center (the sky pass strips the view's
// translation), so its world size never matters.
GpuMesh buildSkyboxCube();
