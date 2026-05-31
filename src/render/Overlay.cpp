#include "Overlay.h"

#include <vector>

#include <glad/glad.h>

#include <VertexArray.h>
#include <VertexBuffer.h>
#include <VertexBufferLayout.h>
#include <Debugger.h>

// Overlays differ from terrain in one important way: their contents CHANGE
// every frame (the recorded path grows, the playback marker moves). So instead
// of building a long-lived VAO/VBO like the terrain does, we build a throwaway
// pair each call -- upload the latest data, draw it, and let the destructors
// free the GPU buffers when the function returns. Cheap for hundreds of points.
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
    // verts.size() / 6 because each vertex is 6 floats (xyz + rgb).
    GLCall(glDrawArrays(primitive, 0, (GLsizei)(verts.size() / 6)));
}

// Flight path: a blue line strip connecting all recorded positions in order.
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

// Camera waypoints: green for the record the player camera is on (position
// match), red for the rest.
void renderCameraRecords(const std::vector<CameraRecord> &cameraRecords, const glm::vec3 &cameraPos, Shader &shader, const glm::mat4 &mvp)
{
    if (cameraRecords.empty()) return;

    std::vector<float> verts;
    verts.reserve(cameraRecords.size() * 6);
    for (const CameraRecord &record : cameraRecords) {
        const glm::vec3 &p = record.position;
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        if (p == cameraPos) {
            verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f); // green
        } else {
            verts.push_back(1.0f); verts.push_back(0.0f); verts.push_back(0.0f); // red
        }
    }

    GLCall(glPointSize(5.0f));
    drawPoints(verts, GL_POINTS, shader, mvp);
}
