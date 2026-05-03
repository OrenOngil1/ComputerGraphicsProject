#include "PickInput.h"
#include "../render/Renderer.h"
#include "../core/Utils.h"

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <opencv2/calib3d.hpp>
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

    // Use flat shading to ensure solid colors for picking
    glShadeModel(GL_FLAT);

    // Render the terrain with unique colors for each vertex
    renderTerrain(mesh, &colorCodes);

    // Read the pixel color at the mouse position
    unsigned char color[3];
    glReadPixels(int(x), camera.height - (int)y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, color);

    // Reset the screen after picking
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Restore smooth shading
    glShadeModel(GL_SMOOTH);

    size_t id = decodeColor(color);
    if(id < colorCodes.size())
        return id;

    return -1; // No vertex picked
}

std::unique_ptr<CameraRecord> computeCameraFromPickedPoints(const std::vector<PickedPoint> &pickedPoints, float fov, int width, int height)
{
    if(pickedPoints.size() < 4) {
        std::cerr << "Not enough points picked to compute camera. Need at least 4 points." << std::endl;
        return nullptr;
    }

    cv::Mat rvec, tvec;

    bool computed = cv::solvePnP(
        map(pickedPoints, [&](const PickedPoint &p) { return cv::Point3f(p.worldPos.x - width / 2.0f, p.worldPos.y, p.worldPos.z - height / 2.0f); }),
        map(pickedPoints, [](const PickedPoint &p) { return cv::Point2f(p.imagePos.x, p.imagePos.y); }),
        getCameraIntrinsicMatrix(fov, width, height),
        cv::Mat::zeros(4, 1, CV_64F),
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_EPNP
    );

    if(!computed) {
        std::cerr << "Failed to compute camera from picked points" << std::endl;
        return nullptr;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat cvCameraPos = -R.t() * tvec;
    cv::Mat cvForward = R.t() * (cv::Mat_<double>(3,1) << 0, 0, 1);
    cv::Mat cvTarget = cvCameraPos + cvForward;

    glm::vec3 cameraPos(cvCameraPos.at<double>(0), cvCameraPos.at<double>(1), cvCameraPos.at<double>(2));
    glm::vec3 target(cvTarget.at<double>(0), cvTarget.at<double>(1), cvTarget.at<double>(2));
    std::cout << "Computed camera position: (" << cameraPos.x << ", " << cameraPos.y << ", " << cameraPos.z << ")" << std::endl;
    return std::make_unique<CameraRecord>(CameraRecord{cameraPos, target});
}

void handlePickKeyButton(AppState &appState, int key)
{
    if(key == GLFW_KEY_C) {
        std::cout << "Computing camera from picked points..." << std::endl;
        appState.computedCameraFromPicking = computeCameraFromPickedPoints(
            appState.pickedPoints,
            appState.playerCamera.fov,
            appState.mesh.width,
            appState.mesh.height
        );
    }
}

void handlePickMouseButton(AppState &appState, double x, double y)
{
    static std::vector<glm::vec3> colorCodes = initColorCodes(appState.mesh.vertices.size());

    int pickedIndex = pickVertex(x, y, appState.mesh, appState.playerCamera, colorCodes);
    if(pickedIndex != -1) {
        glm::vec3 worldPos = appState.mesh.vertices[pickedIndex].position;
        glm::vec2 imagePos(x - appState.playerCamera.x, y - appState.playerCamera.y);
        appState.pickedPoints.push_back(PickedPoint{worldPos, imagePos});
        std::cout << "Picked world: ("
            << worldPos.x << ", " << worldPos.y << ", " << worldPos.z
            << "), image: ("
            << imagePos.x << ", " << imagePos.y << ")"
            << std::endl;
    }
}