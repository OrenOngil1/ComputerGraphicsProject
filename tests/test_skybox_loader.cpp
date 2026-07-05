// Headless checks for loadSkybox -- real PNGs written to a temp dir, no GL.

#include <cstdint>
#include <filesystem>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "check.h"
#include "../src/loader/SkyboxLoader.h"

namespace fs = std::filesystem;

static const char *kFaces[6] = { "posx", "negx", "posy", "negy", "posz", "negz" };

// Write all six faces as size x size PNGs of one solid BGR color.
static void writeFaces(const fs::path &dir, int size, const cv::Scalar &bgr)
{
    for (const char *face : kFaces)
        cv::imwrite((dir / (std::string(face) + ".png")).string(),
                    cv::Mat(size, size, CV_8UC3, bgr));
}

void testSkyboxLoader()
{
    std::cout << "loadSkybox:" << std::endl;

    const fs::path dir = fs::temp_directory_path() / "drone_sim_skybox_test";
    fs::create_directories(dir);

    // Happy path: solid blue (BGR 255,0,0) -- checks count, size, and that
    // the bytes really are BGR order as the header promises.
    writeFaces(dir, 8, cv::Scalar(255, 0, 0));
    auto faces = loadSkybox(dir.string());
    check(faces.has_value(), "six valid faces load");
    if (faces) {
        check(faces->size == 8, "face size reported correctly");
        bool allSized = true;
        for (const auto &px : faces->pixels)
            allSized = allSized && px.size() == 8u * 8u * 3u;
        check(allSized, "every face holds size*size*3 bytes");
        check(faces->pixels[0][0] == 255 && faces->pixels[0][1] == 0
                  && faces->pixels[0][2] == 0,
              "pixel bytes arrive in BGR order");
    }

    // Missing face: remove one file.
    fs::remove(dir / "negy.png");
    check(!loadSkybox(dir.string()).has_value(), "missing face rejected");

    // Non-square face.
    writeFaces(dir, 8, cv::Scalar(255, 0, 0));
    cv::imwrite((dir / "negy.png").string(), cv::Mat(4, 8, CV_8UC3, cv::Scalar(0, 0, 0)));
    check(!loadSkybox(dir.string()).has_value(), "non-square face rejected");

    // Size mismatch between (square) faces.
    writeFaces(dir, 8, cv::Scalar(255, 0, 0));
    cv::imwrite((dir / "negz.png").string(), cv::Mat(4, 4, CV_8UC3, cv::Scalar(0, 0, 0)));
    check(!loadSkybox(dir.string()).has_value(), "face size mismatch rejected");

    fs::remove_all(dir);
}
