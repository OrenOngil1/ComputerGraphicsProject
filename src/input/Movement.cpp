#include "Movement.h"

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
// #include <iostream> // For debugging

void handleMovement(Camera &camera, float terrainSize, int key, int mods)
{
    // Movement speed scales with terrain size
    float rotationSpeed = 0.005f;
    float moveSpeed = terrainSize * 0.01f;

    glm::vec3 delta(0.0f);
    glm::vec3 forward = glm::normalize(camera.target - camera.position);
    glm::vec3 right = glm::normalize(glm::cross(forward, camera.up));

    switch (key) {
        case GLFW_KEY_W:
            delta += forward * moveSpeed;
            break;
        case GLFW_KEY_S:
            delta -= forward * moveSpeed;
            break;
        case GLFW_KEY_A:
            delta -= right * moveSpeed;
            break;
        case GLFW_KEY_D:
            delta += right * moveSpeed;
            break;
        case GLFW_KEY_UP:
            if(forward.y > 0.99f) return;
            camera.target += camera.up * rotationSpeed;
            break;
        case GLFW_KEY_DOWN:
            if(forward.y < -0.99f) return;
            camera.target -= camera.up * rotationSpeed;
            break;
        case GLFW_KEY_LEFT:
            camera.target -= right * rotationSpeed;
            break;
        case GLFW_KEY_RIGHT:
            camera.target += right * rotationSpeed;
            break;
        case GLFW_KEY_PERIOD:
            if(mods & GLFW_MOD_SHIFT) // >
                delta += camera.up * moveSpeed;
            break;
        case GLFW_KEY_COMMA:
            if(mods & GLFW_MOD_SHIFT) // <
                delta -= camera.up * moveSpeed;
            break;
    }

    camera.position += delta;
    camera.target += delta;

    forward = glm::normalize(camera.target - camera.position);
    camera.target = camera.position + forward; // Ensure target is always in front of the camera   
}