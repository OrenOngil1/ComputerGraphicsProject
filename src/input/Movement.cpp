#include "Movement.h"

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
// #include <iostream> // For debugging

void handleMovement(Camera &camera, float terrainSize, int key, int mods)
{
    // Movement speed scales with terrain size
    float moveSpeed = terrainSize * 0.01f;
    glm::vec3 delta(0.0f);

    switch (key) {
        case GLFW_KEY_UP:
            delta += glm::normalize(camera.target - camera.position) * moveSpeed;
            delta.y = 0;
            break;
        case GLFW_KEY_DOWN:
            delta -= glm::normalize(camera.target - camera.position) * moveSpeed;
            delta.y = 0;
            break;
        case GLFW_KEY_LEFT:
            delta -= glm::normalize(glm::cross(camera.target - camera.position, camera.up)) * moveSpeed;
            delta.y = 0;
            break;
        case GLFW_KEY_RIGHT:
            delta += glm::normalize(glm::cross(camera.target - camera.position, camera.up)) * moveSpeed;
            delta.y = 0;
            break;
        case GLFW_KEY_PERIOD:
            if(mods & GLFW_MOD_SHIFT) // >
                delta.y += moveSpeed;
            break;
        case GLFW_KEY_COMMA:
            if(mods & GLFW_MOD_SHIFT) // <
                delta.y -= moveSpeed;
            break;
    }

    camera.position += delta;
    camera.target += delta;
}