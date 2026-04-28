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

void renderCameraRecords(const glm::vec3 &playerPosition, const std::vector<CameraRecord> &cameraRecords)
{
    if(cameraRecords.empty()) return;

    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for (const auto &record : cameraRecords) {

        if (record.position == playerPosition) {
            glColor3f(0.0f, 1.0f, 0.0f); // Green for current position
        } else {
            glColor3f(1.0f, 0.0f, 0.0f); // Red for past positions
        }

        glVertex3f(record.position.x, record.position.y, record.position.z);
    }
    glEnd();
}

glm::vec3 getPickedPointColor(size_t index)
{
    static const glm::vec3 palette[] = {
        glm::vec3(1.0f, 0.5f, 0.0f),  // orange
        glm::vec3(1.0f, 1.0f, 0.0f),  // yellow
        glm::vec3(1.0f, 0.0f, 1.0f),  // magenta
        glm::vec3(0.0f, 1.0f, 1.0f),  // cyan
        glm::vec3(0.6f, 0.2f, 0.8f),  // purple
        glm::vec3(1.0f, 0.4f, 0.7f),  // pink
        glm::vec3(1.0f, 0.7f, 0.3f),  // peach
    };
    return palette[index % (sizeof(palette) / sizeof(palette[0]))];
}

void renderPickedPoints(const std::vector<glm::vec3> &pickedPoints, const Mesh &mesh)
{
    if(pickedPoints.empty()) return;

    // Center the points on the terrain
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(-mesh.width / 2.0f, 0.0f, -mesh.height / 2.0f);

    glPointSize(20.0f);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < pickedPoints.size(); i++) {

        const glm::vec3 &color = getPickedPointColor(i);
        const glm::vec3 &point = pickedPoints[i];

        glColor3f(color.r, color.g, color.b);
        glVertex3f(point.x, point.y * 0.75f, point.z);
    }   
    glEnd();

    glPopMatrix();
}
