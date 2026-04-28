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

void renderPickedPoints(const std::vector<glm::vec3> &pickedPoints, const Mesh &mesh)
{
    if(pickedPoints.empty()) return;

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(-mesh.width / 2.0f, 0.0f, -mesh.height / 2.0f);
    glPointSize(50.0f);
    glColor3f(1.0f, 0.5f, 0.0f); // Orange for picked points
    glBegin(GL_POINTS);
    for (const glm::vec3 &point : pickedPoints) {
        glVertex3f(point.x, point.y, point.z);
    }   
    glEnd();
    glPopMatrix();
}
