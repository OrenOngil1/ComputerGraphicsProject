#pragma once

#include <vector>

#include <glm/glm.hpp>

class Shader;

// One persistent dynamic GPU buffer shared by all overlay geometry (paths,
// markers, view aids). Overlay contents change every frame, but the buffer
// does not: draw() re-uploads into the same VBO, growing it only when a batch
// outsizes every previous one -- so steady-state frames allocate nothing on
// the GPU. Layout is fixed to the overlay convention: interleaved position
// (3 floats) + color (3 floats), shader locations 0 and 1.
class OverlayBatch {
public:
    OverlayBatch();   // creates the VAO/VBO; needs a live GL context

    // Owns raw GL ids -- copying would double-free them.
    OverlayBatch(const OverlayBatch &) = delete;
    OverlayBatch &operator=(const OverlayBatch &) = delete;
    OverlayBatch(OverlayBatch &&other) noexcept;
    OverlayBatch &operator=(OverlayBatch &&other) noexcept;

    // Deletes the VAO/VBO. Runs while the GL context is live (the batch is a
    // Renderer member; see Renderer's ownership note).
    ~OverlayBatch();

    // Upload the vertices and draw them as `primitive` (a GLenum; unsigned int
    // here to keep glad out of this header). Binds, draws, and unbinds -- no GL
    // state is left behind.
    void draw(const std::vector<float> &verts, unsigned int primitive,
              Shader &shader, const glm::mat4 &mvp);

private:
    unsigned int m_vao = 0;
    unsigned int m_vbo = 0;
    size_t m_capacityBytes = 0;   // current glBufferData size; grows, never shrinks
};
