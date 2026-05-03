#include "Overlay.h"

#include <GL/gl.h>

#define POINT_SIZE 10.0f

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

    glPointSize(POINT_SIZE);
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

void renderPickedPoints(const std::vector<PickedPoint> &pickedPoints, const Mesh &mesh, float pointSize)
{
    if(pickedPoints.empty()) return;

    // Center the points on the terrain
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(-mesh.width / 2.0f, 0.0f, -mesh.height / 2.0f);

    // Ensure points are visible on top of terrain
    glDisable(GL_DEPTH_TEST);

    glPointSize(pointSize);
    glBegin(GL_POINTS);
    for(size_t i = 0; i < pickedPoints.size(); i++) {

        const glm::vec3 &color = getPickedPointColor(i);
        const PickedPoint &point = pickedPoints[i];

        glColor3f(color.r, color.g, color.b);
        glVertex3f(point.worldPos.x, point.worldPos.y, point.worldPos.z);
    }   
    glEnd();

    glEnable(GL_DEPTH_TEST);

    glPopMatrix();
}

void renderPickedPointGlobal(const std::vector<PickedPoint> &pickedPoints, const Mesh &mesh)
{
    renderPickedPoints(pickedPoints, mesh, POINT_SIZE);
}

void renderPickedPointPlayer(const std::vector<PickedPoint> &pickedPoints, const Mesh &mesh)
{
    renderPickedPoints(pickedPoints, mesh, POINT_SIZE * 1.5f);
}

void renderPnPCameraDiffGlobal(const glm::vec3 &computedCameraPosition)
{
    glColor3f(1.0f, 0.0f, 0.0f); // Red for computed camera
    glPointSize(POINT_SIZE);
    glBegin(GL_POINTS);
    glVertex3f(computedCameraPosition.x, computedCameraPosition.y, computedCameraPosition.z);
    glEnd();
}

void setupGhostEffect()
{
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
}

void cleanupGhostEffect()
{
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
}