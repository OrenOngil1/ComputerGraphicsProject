#include "Recording.h"

void recordPathPoint(std::vector<glm::vec3> &pathPoints, const glm::vec3 &position)
{
    pathPoints.push_back(position);
}

void recordCamera(std::vector<CameraRecord> &cameraRecords, const Camera &camera)
{
    cameraRecords.push_back({ camera.position, camera.target });
}