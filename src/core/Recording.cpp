#include "Recording.h"

void recordPathPoint(std::vector<glm::vec3> &pathPoints, const glm::vec3 &position)
{
    // Only record if moved more than 1.0 units
    if(pathPoints.empty() || glm::distance(pathPoints.back(), position) > 1.0f)
        pathPoints.push_back(position);
}

void recordCamera(std::vector<CameraRecord> &cameraRecords, const Camera &camera)
{
    cameraRecords.push_back({ camera.position, camera.target });
}