#include "Overlay.h"

#include <GL/gl.h>

void renderPath(const std::vector<glm::vec3> &pathPoints)
{
    glColor3f(0.0f, 0.0f, 1.0f);
    glLineWidth(3.0f);
    glBegin(GL_LINE_STRIP);
    for (const glm::vec3 &point : pathPoints) {
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
}

void renderCameraRecords(const std::vector<CameraRecord> &cameraRecords, size_t playbackIndex)
{
    if(cameraRecords.empty()) return;

    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < cameraRecords.size(); i++) {

        if (i == playbackIndex) {
            glColor3f(0.0f, 1.0f, 0.0f); // Green for current position
        } else {
            glColor3f(1.0f, 0.0f, 0.0f); // Red for past positions
        }

        const CameraRecord &record = cameraRecords[i];
        glVertex3f(record.position.x, record.position.y, record.position.z);
    }
    glEnd();
}
