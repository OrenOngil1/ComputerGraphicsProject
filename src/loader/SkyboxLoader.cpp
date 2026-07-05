#include "SkyboxLoader.h"

#include <cstring>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

// File names by GL face order (+X, -X, +Y, -Y, +Z, -Z) -- see CubemapFaces.
static const char *kFaceNames[6] = { "posx", "negx", "posy", "negy", "posz", "negz" };

std::optional<CubemapFaces> loadSkybox(const std::string &dir)
{
    CubemapFaces faces;
    for (int i = 0; i < 6; i++) {
        const std::string path = dir + "/" + kFaceNames[i] + ".png";

        // IMREAD_COLOR pins the decode to 3-channel BGR whatever the file
        // holds (grayscale, alpha), so the byte layout below is guaranteed.
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "Skybox face missing or unreadable: " << path << std::endl;
            return std::nullopt;
        }
        if (img.rows != img.cols) {
            std::cerr << "Skybox face not square (" << img.cols << "x" << img.rows
                      << "): " << path << std::endl;
            return std::nullopt;
        }
        if (i == 0)
            faces.size = img.rows;
        else if (img.rows != faces.size) {
            std::cerr << "Skybox face size mismatch (" << img.rows << " vs "
                      << faces.size << "): " << path << std::endl;
            return std::nullopt;
        }

        // OpenCV may pad row ends; compact first so one memcpy of
        // size*size*3 bytes can't shear the rows.
        if (!img.isContinuous())
            img = img.clone();
        const size_t bytes = (size_t)faces.size * faces.size * 3;
        faces.pixels[i].resize(bytes);
        std::memcpy(faces.pixels[i].data(), img.data, bytes);
    }
    return faces;
}
