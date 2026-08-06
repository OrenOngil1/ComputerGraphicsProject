#include "TerrainLoader.h"

#include <iostream>
#include <algorithm>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

// Map a height to a terrain color by blending through a fixed elevation ramp
// (deep water -> shallow -> sand -> grass -> forest -> rock -> snow). minH/maxH
// normalize the height to [0,1], so the ramp spans whatever relief the DEM has.
static glm::vec3 getColor(float height, float minH, float maxH)
{
    if (minH == maxH) {   // flat DEM: avoid dividing by zero
        minH = 0.0f;
        maxH = 1.0f;
    }

    float n = (height - minH) / (maxH - minH);

    const glm::vec3 deepWater    = glm::vec3(0.0f,  0.15f, 0.5f);
    const glm::vec3 shallowWater = glm::vec3(0.0f,  0.3f,  0.6f);
    const glm::vec3 sand         = glm::vec3(0.76f, 0.70f, 0.50f);
    const glm::vec3 grass        = glm::vec3(0.25f, 0.50f, 0.20f);
    const glm::vec3 forest       = glm::vec3(0.15f, 0.35f, 0.15f);
    const glm::vec3 rock         = glm::vec3(0.45f, 0.38f, 0.28f);
    const glm::vec3 snow         = glm::vec3(0.95f, 0.95f, 1.00f);

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

static void applyElevationColors(Mesh &mesh, float minH, float maxH)
{
    for (Vertex &vertex : mesh.vertices)
        vertex.color = getColor(vertex.position.y, minH, maxH);
}

void computeNormals(Mesh &mesh)
{
    // Height lookup with clamped indices: stepping off the grid re-samples the
    // border vertex, turning the central difference into a one-sided one there.
    auto heightAt = [&mesh](int x, int z) {
        x = std::clamp(x, 0, mesh.cols - 1);
        z = std::clamp(z, 0, mesh.rows - 1);
        return mesh.vertices[z * mesh.cols + x].position.y;
    };

    for (int z = 0; z < mesh.rows; z++) {
        for (int x = 0; x < mesh.cols; x++) {
            // Cross product of the grid tangents (1, dh/dx, 0) x (0, dh/dz, 1),
            // flipped to point up; normalize() eats the central-difference
            // factor of 2.
            float dydx = heightAt(x + 1, z) - heightAt(x - 1, z);
            float dydz = heightAt(x, z + 1) - heightAt(x, z - 1);
            mesh.vertices[z * mesh.cols + x].normal = glm::normalize(glm::vec3(-dydx, 2.0f, -dydz));
        }
    }
}

// Height at a corner vertex: the mean of the (up to 4) surrounding pixels.
// Near an edge the clamps make some of the four coincide, which the mean absorbs.
static float sampleHeight(const cv::Mat &img, int i, int j)
{
    // OpenCV is row-major: j (rows) is y, i (cols) is x.
    int x0 = std::max(i - 1, 0), x1 = std::min(i, img.cols - 1);
    int y0 = std::max(j - 1, 0), y1 = std::min(j, img.rows - 1);

    int sum = img.at<uchar>(y0, x0) + img.at<uchar>(y0, x1)
            + img.at<uchar>(y1, x0) + img.at<uchar>(y1, x1);
    return sum / 4.0f;
}

std::optional<Mesh> readTerrain(const std::string &filename)
{
    cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Failed to load image: " << filename << std::endl;
        return std::nullopt;
    }

    // Corner vertices sit between pixels: one more vertex than pixels per axis.
    Mesh mesh = { image.cols + 1, image.rows + 1, std::vector<Vertex>() };
    mesh.vertices.reserve(mesh.rows * mesh.cols);

    // The one amplitude knob: peak-to-trough relief as a fraction of the
    // terrain's width (lower = flatter). The height is normalized to
    // [-0.5, 0.5] -- centered about y = 0 -- then scaled.
    const float reliefFraction = 0.1f;
    const float heightScale = std::max(image.rows, image.cols) * reliefFraction;

    // Track min and max height for the color mapping afterward.
    float minH = std::numeric_limits<float>::max();
    float maxH = std::numeric_limits<float>::lowest();
    for (int j = 0; j <= image.rows; j++) {
        for (int i = 0; i <= image.cols; i++) {
            float h = heightScale * (sampleHeight(image, i, j) / 255.0f - 0.5f);

            minH = std::min(minH, h);
            maxH = std::max(maxH, h);

            mesh.vertices.push_back({ glm::vec3(i, h, j), glm::vec3(-1.0f) });
        }
    }

    // Colors and normals both need the finished height field, so they run
    // after every vertex is in place.
    applyElevationColors(mesh, minH, maxH);
    computeNormals(mesh);

    return mesh;
}
