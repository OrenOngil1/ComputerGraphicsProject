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
            delta += glm::vec3(0.0f, 0.0f, -1.0f) * moveSpeed;
            break;
        case GLFW_KEY_DOWN:
            delta -= glm::vec3(0.0f, 0.0f, -1.0f) * moveSpeed;
            break;
        case GLFW_KEY_LEFT:
            delta -= glm::vec3(1.0f, 0.0f, 0.0f) * moveSpeed;
            delta.y = 0;
            break;
        case GLFW_KEY_RIGHT:
            delta += glm::vec3(1.0f, 0.0f, 0.0f) * moveSpeed;
            delta.y = 0;
            break;
        case GLFW_KEY_PERIOD:
            if(mods & GLFW_MOD_SHIFT) // >
                delta += glm::vec3(0.0f, 1.0f, 0.0f) * moveSpeed;
            break;
        case GLFW_KEY_COMMA:
            if(mods & GLFW_MOD_SHIFT) // <
                delta -= glm::vec3(0.0f, 1.0f, 0.0f) * moveSpeed;
            break;
    }

    camera.position += delta;
    camera.target += delta;
}