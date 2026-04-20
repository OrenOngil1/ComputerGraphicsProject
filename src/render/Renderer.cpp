#include "Renderer.h"

#include <GLFW/glfw3.h>
#include <GL/glu.h>

#include "Overlay.h"

// prototypes for helper functions
void updateAndSetupCameraViewport(Camera &camera);
void setupProjection(const Camera &camera);
void setupModelView(const Camera &camera);

void renderGlobalOverlay(const AppState &appState)
{
    switch(appState.mode) {
        case Mode::NONE:
            return;
        case Mode::RECORD:
        case Mode::PLAYBACK:
            renderPath(appState.pathPoints);
            renderCameraRecords(appState.cameraRecords, appState.playbackIndex);
            break;
    }
}

void renderPlayerOverlay(const AppState &appState)
{
    return; // No player overlay for now
}

void setupCamera(Camera &camera)
{
    updateAndSetupCameraViewport(camera);

    setupProjection(camera);

    setupModelView(camera);
}

void renderTerrain(const Mesh &mesh)
{   
    // Center the terrain
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(-mesh.width / 2.0f, 0.0f, -mesh.height / 2.0f);

    for (int z = 0; z < mesh.height - 1; z++) {

        // Draw the terrain using triangle strips for better performance
        // each inner loop draws a strip between two rows of vertices
        glBegin(GL_TRIANGLE_STRIP);

        for (int x = 0; x < mesh.width; x++) {

            const Vertex &vertex = mesh.vertices[z * mesh.width + x];
            glColor3f(vertex.color.r, vertex.color.g, vertex.color.b);
            glVertex3f(vertex.position.x, vertex.position.y, vertex.position.z);

            const Vertex &nextVertex = mesh.vertices[(z + 1) * mesh.width + x];
            glColor3f(nextVertex.color.r, nextVertex.color.g, nextVertex.color.b);
            glVertex3f(nextVertex.position.x, nextVertex.position.y, nextVertex.position.z);

        }

        glEnd();
    }

    // Restore the modelview matrix after drawing the terrain
    glPopMatrix();
}

void updateAndSetupCameraViewport(Camera &camera)
{
    // Update viewport size in case the window was resized
    glfwGetWindowSize(glfwGetCurrentContext(), &camera.width, &camera.height);

    // Use half the window width for each camera
    // Left camera starts at x=0, right camera starts at x=width/2
    camera.width /= 2.0f;
    camera.x = camera.x == 0 ? 0 : camera.width;

    // Set the viewport to match the camera's dimensions
    glViewport(camera.x, camera.y, camera.width, camera.height);
}

void setupProjection(const Camera &camera)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(camera.fov, (float)camera.width / camera.height, camera.near, camera.far);
}

void setupModelView(const Camera &camera)
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(camera.position.x, camera.position.y, camera.position.z,
              camera.target.x, camera.target.y, camera.target.z,
              camera.up.x, camera.up.y, camera.up.z);
}
