#include "Movement.h"

#include <GLFW/glfw3.h>
#include "../core/Recording.h"
// #include <iostream> // For debugging

void move(const bool keys[], float deltaTime, Camera &camera, std::vector<glm::vec3> &pathPoints, float terrainSize)
{
    bool isShift = keys[GLFW_KEY_LEFT_SHIFT] || keys[GLFW_KEY_RIGHT_SHIFT];

    // Movement speed scales with terrain size and deltaTime for smooth movement
    float rotationSpeed = 1.05f * deltaTime;
    float moveSpeed = 0.05f * terrainSize * deltaTime;

    glm::vec3 delta(0.0f);
    glm::vec3 forward = glm::normalize(camera.target - camera.position);
    glm::vec3 right = glm::normalize(glm::cross(forward, camera.up));

    if(keys[GLFW_KEY_W])
        delta += forward * moveSpeed;
    if(keys[GLFW_KEY_S])
        delta -= forward * moveSpeed;
    if(keys[GLFW_KEY_A])
        delta -= right * moveSpeed;
    if(keys[GLFW_KEY_D])
        delta += right * moveSpeed;
    if(keys[GLFW_KEY_UP] && forward.y <= 0.99f)
        camera.target += camera.up * rotationSpeed;
    if(keys[GLFW_KEY_DOWN] && forward.y >= -0.99f)
        camera.target -= camera.up * rotationSpeed;
    if(keys[GLFW_KEY_LEFT])
        camera.target -= right * rotationSpeed;
    if(keys[GLFW_KEY_RIGHT])
        camera.target += right * rotationSpeed;
    if(isShift && keys[GLFW_KEY_PERIOD]) // >
        delta += camera.up * moveSpeed;
    if(isShift && keys[GLFW_KEY_COMMA]) // <
        delta -= camera.up * moveSpeed; 

    camera.position += delta;
    camera.target += delta;

    forward = glm::normalize(camera.target - camera.position);
    camera.target = camera.position + forward; // Ensure target is always in front of the camera   

    recordPathPoint(pathPoints, camera.position);
}