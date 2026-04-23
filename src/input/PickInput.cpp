#include "PickInput.h"
#include "../render/Renderer.h"

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <iostream> // for debugging

glm::vec3 encodeId(size_t id)
{
    int r = (id & 0x000000FF) >> 0;
    int g = (id & 0x0000FF00) >> 8;
    int b = (id & 0x00FF0000) >> 16;

    return glm::vec3(r,g,b) / 255.0f;
}

size_t decodeColor(unsigned char color[3])
{
    return color[0] | (color[1] << 8) | (color[2] << 16);
}

std::vector<glm::vec3> initColorCodes(size_t num)
{
    std::vector<glm::vec3> colorCodes;

    for(size_t i = 0; i < num; i++)
        colorCodes.push_back(encodeId(i));

    return colorCodes;
}

int pickVertex(float x, float y, const Mesh &mesh, Camera &camera, const std::vector<glm::vec3> &colorCodes)
{
    setupCamera(camera);

    // Clear the screen with white color for picking
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the terrain with unique colors for each vertex
    renderTerrainByColor(mesh, colorCodes);
    unsigned char color[3];
    glReadPixels((int)x, camera.height - (int)y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, color);

    // Reset the screen after picking
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    size_t id = decodeColor(color);
    if(id < colorCodes.size())
        return id;

    return -1; // No vertex picked
}

void handlePickMouseButton(AppState &appstate, double x, double y)
{
    static std::vector<glm::vec3> colorCodes = initColorCodes(appState.mesh.vertices.size());

    int pickedIndex = pickVertex(x, y, appState.mesh, appstate.playerCamera, colorCodes);
    if(pickedIndex != -1) {
        appstate.pickedVertices.push_back(appstate.mesh.vertices[pickedIndex]);
        const glm::vec3 &pickedPos = appstate.mesh.vertices[pickedIndex].position;
        std::cout << "Picked vertex position: (" << pickedPos.x << ", " << pickedPos.y << ", " << pickedPos.z << ")" << std::endl;
    }
}