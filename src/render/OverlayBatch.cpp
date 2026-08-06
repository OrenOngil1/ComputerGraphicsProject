#include "OverlayBatch.h"

#include <utility>

#include <glad/glad.h>

#include <Debugger.h>   // GLCall
#include <Shader.h>

static const int kFloatsPerVertex = 6;   // xyz + rgb
static const GLsizei kStrideBytes = kFloatsPerVertex * sizeof(float);

OverlayBatch::OverlayBatch()
{
    GLCall(glGenVertexArrays(1, &m_vao));
    GLCall(glGenBuffers(1, &m_vbo));

    // Record the attribute layout in the VAO once; every draw reuses it.
    GLCall(glBindVertexArray(m_vao));
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_vbo));
    GLCall(glEnableVertexAttribArray(0));   // position
    GLCall(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kStrideBytes, (const void *)0));
    GLCall(glEnableVertexAttribArray(1));   // color
    GLCall(glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, kStrideBytes,
                                 (const void *)(3 * sizeof(float))));
    GLCall(glBindVertexArray(0));
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
}

OverlayBatch::OverlayBatch(OverlayBatch &&other) noexcept
    : m_vao(std::exchange(other.m_vao, 0u)),
      m_vbo(std::exchange(other.m_vbo, 0u)),
      m_capacityBytes(std::exchange(other.m_capacityBytes, size_t(0)))
{
}

OverlayBatch &OverlayBatch::operator=(OverlayBatch &&other) noexcept
{
    if (this != &other) {
        // Braced: GLCall is a multi-statement macro, unsafe in a bare if.
        if (m_vbo) { GLCall(glDeleteBuffers(1, &m_vbo)); }
        if (m_vao) { GLCall(glDeleteVertexArrays(1, &m_vao)); }
        m_vao = std::exchange(other.m_vao, 0u);
        m_vbo = std::exchange(other.m_vbo, 0u);
        m_capacityBytes = std::exchange(other.m_capacityBytes, size_t(0));
    }
    return *this;
}

OverlayBatch::~OverlayBatch()
{
    // Braced: GLCall is a multi-statement macro, unsafe in a bare if.
    if (m_vbo) { GLCall(glDeleteBuffers(1, &m_vbo)); }
    if (m_vao) { GLCall(glDeleteVertexArrays(1, &m_vao)); }
}

void OverlayBatch::draw(const std::vector<float> &verts, unsigned int primitive,
                        Shader &shader, const glm::mat4 &mvp)
{
    if (verts.empty())
        return;
    const GLsizeiptr bytes = (GLsizeiptr)(verts.size() * sizeof(float));

    shader.Bind();
    shader.SetUniformMat4f("u_MVP", mvp);

    GLCall(glBindVertexArray(m_vao));
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, m_vbo));
    if ((size_t)bytes > m_capacityBytes) {
        // First batch of this size: (re)allocate the store. Later frames of
        // equal or smaller batches take the orphan + glBufferSubData path.
        GLCall(glBufferData(GL_ARRAY_BUFFER, bytes, verts.data(), GL_DYNAMIC_DRAW));
        m_capacityBytes = (size_t)bytes;
    } else {
        // Orphan before writing: earlier draws this frame may still be
        // sourcing the old store, and re-specifying at the same capacity gives
        // the driver a fresh one instead of a sync point.
        GLCall(glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)m_capacityBytes, nullptr,
                            GL_DYNAMIC_DRAW));
        GLCall(glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, verts.data()));
    }
    GLCall(glDrawArrays(primitive, 0, (GLsizei)(verts.size() / kFloatsPerVertex)));

    GLCall(glBindVertexArray(0));
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
    shader.Unbind();
}
