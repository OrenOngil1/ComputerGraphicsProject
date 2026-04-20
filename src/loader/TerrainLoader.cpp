#include "TerrainLoader.h"

#include <iostream>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

bool vertexByYComp(const Vertex& a, const Vertex& b) {
    return a.position.y < b.position.y;
}

// Simple color mapping function: maps height to a color gradient
glm::vec3 getColor(float height, float minH, float maxH)
{

    if(minH == maxH) {
        minH = 0.0f;
        maxH = 1.0f;
    }

    float n = (height - minH) / (maxH - minH);

    // color stops
    const glm::vec3 deepWater    = glm::vec3(0.0f,  0.15f, 0.5f);
    const glm::vec3 shallowWater = glm::vec3(0.0f,  0.3f,  0.6f);
    const glm::vec3 sand         = glm::vec3(0.76f, 0.70f, 0.50f);
    const glm::vec3 grass        = glm::vec3(0.25f, 0.50f, 0.20f);
    const glm::vec3 forest       = glm::vec3(0.15f, 0.35f, 0.15f);
    const glm::vec3 rock         = glm::vec3(0.45f, 0.38f, 0.28f);
    const glm::vec3 snow         = glm::vec3(0.95f, 0.95f, 1.00f);

    // blend between stops
    if (n < 0.15f)
        return glm::mix(deepWater,   shallowWater, n / 0.15f);
    else if (n < 0.25f)
        return glm::mix(shallowWater, sand,        (n - 0.15f) / 0.10f);
    else if (n < 0.35f)
        return glm::mix(sand,        grass,        (n - 0.25f) / 0.10f);
    else if (n < 0.60f)
        return glm::mix(grass,       forest,       (n - 0.35f) / 0.25f);
    else if (n < 0.75f)
        return glm::mix(forest,      rock,         (n - 0.60f) / 0.15f);
    else if (n < 0.90f)
        return glm::mix(rock,        snow,         (n - 0.75f) / 0.15f);
    else
        return snow;
}

void getColors(Mesh &mesh)
{
    float minH = std::min_element(
        mesh.vertices.begin(), mesh.vertices.end(), vertexByYComp)->position.y;
    float maxH = std::max_element(
        mesh.vertices.begin(), mesh.vertices.end(), vertexByYComp)->position.y;

    for(Vertex &vertex : mesh.vertices) {
        vertex.color = getColor(vertex.position.y, minH, maxH);
    }
}

// Sample the height at a corner by averaging the surrounding pixels, clamping to image bounds
float sampleHeight(cv::Mat& img, int i, int j) {

    // i,j is a corner
    // clamp to image bounds
    int x0 = std::max(i-1, 0),        x1 = std::min(i, img.cols-1);
    int y0 = std::max(j-1, 0),        y1 = std::min(j, img.rows-1);

    // OpenCV uses row-major order
    // so j is the y-coordinate and i is the x-coordinate
    float sum = (img.at<uchar>(y0, x0) + img.at<uchar>(y0, x1)
                + img.at<uchar>(y1, x0) + img.at<uchar>(y1, x1)) / 4.0f;

    return sum / 4.0f;
}

Mesh readTerrain(const std::string& filename)
{

    cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Failed to load image: " << filename << std::endl;
        return {0, 0, std::vector<Vertex>()};
    }

    Mesh mesh = { image.cols + 1, image.rows + 1, std::vector<Vertex>() };
    float heightScale = std::max(image.rows, image.cols) * 0.2f;

    for(int j = 0; j <= image.rows; j++) {
        for(int i = 0; i <= image.cols; i++) {

            // normalize height to range [-0.5, 0.5] and scale it
            float h = heightScale * (sampleHeight(image, i, j) / 255.0f - 0.5f);

            // Default color value; will be set later based on height
            mesh.vertices.push_back({ glm::vec3(i, h, j), glm::vec3(-1.0f) });
        }
    }

    // After all vertices are created, assign colors based on height
    getColors(mesh);

    // Check if the mesh is valid
    if(mesh.vertices.empty() || mesh.vertices[0].color == glm::vec3(-1.0f)) {
        std::cerr << "Failed to create valid mesh from image: " << filename << std::endl;
        return {0, 0, std::vector<Vertex>()};
    }

    return mesh;
}