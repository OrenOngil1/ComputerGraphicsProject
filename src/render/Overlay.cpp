#include "Overlay.h"

#include <vector>

#include <glad/glad.h>

#include <VertexArray.h>
#include <VertexBuffer.h>
#include <VertexBufferLayout.h>
#include <Debugger.h>

static void drawPoints(const std::vector<float> &verts, GLenum primitive, Shader &shader, const glm::mat4 &mvp)
{
    VertexArray va;
    VertexBuffer vb(verts.data(), verts.size() * sizeof(float));

    VertexBufferLayout layout;
    layout.Push<float>(3);  // position
    layout.Push<float>(3);  // color
    va.AddBuffer(vb, layout);

    shader.Bind();
    shader.SetUniformMat4f("u_MVP", mvp);
    va.Bind();
    GLCall(glDrawArrays(primitive, 0, (GLsizei)(verts.size() / 6)));
}

void renderPath(const std::vector<glm::vec3> &pathPoints, Shader &shader, const glm::mat4 &mvp)
{
    if (pathPoints.empty()) return;

    std::vector<float> verts;
    verts.reserve(pathPoints.size() * 6);
    for (const glm::vec3 &p : pathPoints) {
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(1.0f); // blue
    }

    GLCall(glLineWidth(3.0f));
    drawPoints(verts, GL_LINE_STRIP, shader, mvp);
}

void renderCameraRecords(const std::vector<CameraRecord> &cameraRecords, size_t playbackIndex, Shader &shader, const glm::mat4 &mvp)
{
    if (cameraRecords.empty()) return;

    std::vector<float> verts;
    verts.reserve(cameraRecords.size() * 6);
    for (size_t i = 0; i < cameraRecords.size(); i++) {
        const glm::vec3 &p = cameraRecords[i].position;
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        if (i == playbackIndex) {
            verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f); // green
        } else {
            verts.push_back(1.0f); verts.push_back(0.0f); verts.push_back(0.0f); // red
        }
    }

    GLCall(glPointSize(5.0f));
    drawPoints(verts, GL_POINTS, shader, mvp);
}
