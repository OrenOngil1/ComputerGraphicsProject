#include "GlTexture.h"

#include <glad/glad.h>

#include <Debugger.h>   // GLCall

GlTexture::GlTexture(GlTexture &&other) noexcept
    : m_id(other.m_id)
{
    other.m_id = 0;
}

GlTexture &GlTexture::operator=(GlTexture &&other) noexcept
{
    if (this != &other) {
        // Braced: GLCall is a multi-statement macro, unsafe in a bare if.
        if (m_id != 0) { GLCall(glDeleteTextures(1, &m_id)); }
        m_id = other.m_id;
        other.m_id = 0;
    }
    return *this;
}

GlTexture::~GlTexture()
{
    // Braced: GLCall is a multi-statement macro, unsafe in a bare if.
    if (m_id != 0) { GLCall(glDeleteTextures(1, &m_id)); }
}
