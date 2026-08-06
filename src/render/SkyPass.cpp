#include "SkyPass.h"

#include <glad/glad.h>

#include <Debugger.h>   // GLCall

#include "../loader/SkyboxLoader.h"

// Create a GL cubemap from six decoded faces. GL_BGR consumes the loader's
// OpenCV byte order directly -- no channel swap. Unpack alignment 1 matches
// the loader's tightly packed rows (set-and-leave, like GL_PACK_ALIGNMENT in
// Renderer's readViewportPixels). CLAMP_TO_EDGE on all three axes prevents
// visible seams where two faces meet.
static GlTexture uploadCubemap(const CubemapFaces &faces)
{
    GLuint id = 0;
    GLCall(glGenTextures(1, &id));
    GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, id));
    GLCall(glPixelStorei(GL_UNPACK_ALIGNMENT, 1));

    for (int i = 0; i < 6; i++) {
        GLCall(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB8,
                            faces.size, faces.size, 0,
                            GL_BGR, GL_UNSIGNED_BYTE, faces.pixels[i].data()));
    }

    GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE));
    GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));
    return GlTexture(id);
}

SkyPass::SkyPass(std::string skyboxRoot)
    : m_shader("assets/shaders/skyboxShader.glsl"),
      m_root(std::move(skyboxRoot))
{
    m_cube = buildSkyboxCube();
}

void SkyPass::draw(size_t presetIdx, const Camera &camera, const Viewport &viewport)
{
    SkySlot &slot = m_slots[presetIdx];
    if (!slot.tried) {
        slot.tried = true;   // set first: a failed load must not retry per frame
        if (auto faces = loadSkybox(m_root + kLightPresets[presetIdx].name))
            slot.tex = uploadCubemap(*faces);
        // On failure loadSkybox already warned; tex stays empty -> clear-color sky.
    }
    if (slot.tex.id() == 0)
        return;

    m_shader.Bind();
    m_shader.SetUniformMat4f("u_ViewProj", skyViewProjection(camera, viewport));
    m_shader.SetUniform1i("u_Skybox", 0);
    GLCall(glActiveTexture(GL_TEXTURE0));
    GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, slot.tex.id()));
    m_cube.va->Bind();
    m_cube.ib->Bind();
    GLCall(glDrawElements(GL_TRIANGLES, m_cube.indexCount, GL_UNSIGNED_INT, nullptr));

    // Leave nothing bound: the overlays drawn after the sky bind their own
    // program and buffers, and must not inherit the cubemap or cube VAO. The
    // VAO unbind covers the index buffer too -- its binding is VAO state, and
    // touching GL_ELEMENT_ARRAY_BUFFER with no VAO bound is invalid in the
    // core profile.
    m_cube.va->Unbind();
    GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, 0));
    m_shader.Unbind();
}
