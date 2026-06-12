// Headless sanity checks for the GL-free math: the PnP solvers, the tracker
// blob centroids, and the terrain normals, each on synthetic inputs with a
// known correct answer. No window, no GL context -- run anywhere with
// `make check`. Exit code = number of failed checks, so CI-style use works.

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "../src/loader/TerrainLoader.h"     // computeNormals
#include "../src/vision/Pnp.h"               // computeCameraPose(+Ransac)
#include "../src/vision/TrackerDetection.h"  // findTrackerCentroids

static int failures = 0;

static void check(bool ok, const std::string &name)
{
    std::cout << (ok ? "  PASS  " : "  FAIL  ") << name << std::endl;
    if (!ok)
        failures++;
}

// ── computeNormals ────────────────────────────────────────────

// A cols x rows grid with height h(x, z), matching the loader's layout:
// vertices[z * cols + x] sits at world (x, h, z).
template <typename HeightFn>
static Mesh makeGrid(int cols, int rows, HeightFn h)
{
    Mesh mesh;
    mesh.cols = cols;
    mesh.rows = rows;
    mesh.vertices.resize((size_t)cols * rows);
    for (int z = 0; z < rows; z++)
        for (int x = 0; x < cols; x++)
            mesh.vertices[(size_t)z * cols + x].position =
                glm::vec3((float)x, h(x, z), (float)z);
    return mesh;
}

static void testNormals()
{
    std::cout << "computeNormals:" << std::endl;

    // Flat ground: every normal is straight up, including the borders.
    Mesh flat = makeGrid(5, 5, [](int, int) { return 0.0f; });
    computeNormals(flat);
    bool allUp = true;
    for (const Vertex &v : flat.vertices)
        allUp = allUp && glm::length(v.normal - glm::vec3(0, 1, 0)) < 1e-5f;
    check(allUp, "flat grid: all normals point straight up");

    // Tilted plane y = x/2: the surface tangents are (1, 1/2, 0) and (0, 0, 1),
    // whose cross product normalizes to (-1, 2, 0) / sqrt(5). Central
    // differences are exact on a plane, so interior vertices must match it.
    Mesh tilted = makeGrid(5, 5, [](int x, int) { return 0.5f * x; });
    computeNormals(tilted);
    const glm::vec3 expected = glm::normalize(glm::vec3(-1.0f, 2.0f, 0.0f));
    const glm::vec3 got = tilted.vertices[2 * 5 + 2].normal;   // center vertex
    check(glm::length(got - expected) < 1e-5f,
          "tilted plane: interior normal matches the analytic one");
}

// ── findTrackerCentroids ──────────────────────────────────────

static void paintBlock(FramePixels &frame, int cx, int cy, int halfSize,
                       unsigned char r, unsigned char g, unsigned char b)
{
    for (int y = cy - halfSize; y <= cy + halfSize; y++)
        for (int x = cx - halfSize; x <= cx + halfSize; x++) {
            unsigned char *px = &frame.rgb[((size_t)y * frame.width + x) * 3];
            px[0] = r; px[1] = g; px[2] = b;
        }
}

static void testCentroids()
{
    std::cout << "findTrackerCentroids:" << std::endl;

    // A black 100x80 detection frame -- as captureTrackersFrame would produce.
    FramePixels frame;
    frame.width = 100;
    frame.height = 80;
    frame.rgb.assign((size_t)frame.width * frame.height * 3, 0);

    // Red: a 5x5 block centered on (30, 40) -- centroid lands on the center.
    // Blue: a 2x2 block (4 px), under the 6-pixel minimum -- too small to
    // trust, must come back empty. Green: never painted -- not visible.
    paintBlock(frame, 30, 40, 2, 255, 0, 0);
    paintBlock(frame, 70, 20, 0, 0, 0, 255);
    paintBlock(frame, 71, 20, 0, 0, 0, 255);
    paintBlock(frame, 70, 21, 0, 0, 0, 255);
    paintBlock(frame, 71, 21, 0, 0, 0, 255);

    const std::vector<Tracker> trackers = {
        { glm::vec3(0), 1.0f, glm::vec3(1, 0, 0) },   // red
        { glm::vec3(0), 1.0f, glm::vec3(0, 1, 0) },   // green
        { glm::vec3(0), 1.0f, glm::vec3(0, 0, 1) },   // blue
    };
    std::vector<std::optional<glm::vec2>> centroids =
        findTrackerCentroids(frame, trackers);

    check(centroids[0] && glm::length(*centroids[0] - glm::vec2(30, 40)) < 1e-4f,
          "5x5 red blob: centroid at its center");
    check(!centroids[1], "absent green tracker: reported not visible");
    check(!centroids[2], "4-pixel blue blob: under the size floor, dropped");
}

// ── the PnP solvers ───────────────────────────────────────────

// Project a world point through a glm camera into image coordinates (origin
// top-left, y down) -- the convention every mode feeds the solvers, using the
// same intrinsics construction (vertical FOV onto the viewport height).
// glm::lookAt yields the OpenGL camera frame (x right, y up, looking down -z);
// the image frame has y down and z forward, so y and z flip sign.
static glm::vec2 projectToPixel(const glm::mat4 &view, const glm::vec3 &p,
                                float fov, int width, int height)
{
    const glm::vec3 pc = glm::vec3(view * glm::vec4(p, 1.0f));
    const float fy = height / (2.0f * std::tan(glm::radians(fov) / 2.0f));
    const float fx = fy * ((float)width / (float)height);
    return { fx * (pc.x / -pc.z) + width  / 2.0f,
             fy * (-pc.y / -pc.z) + height / 2.0f };
}

static void testPnp()
{
    std::cout << "PnP solvers:" << std::endl;

    const glm::vec3 eye(10.0f, 8.0f, 20.0f);
    const glm::vec3 target(0.0f, 0.0f, 0.0f);
    const float fov = 45.0f;
    const int width = 800, height = 600;
    const glm::mat4 view = glm::lookAt(eye, target, glm::vec3(0, 1, 0));

    // Non-coplanar points spread through the camera's field of view.
    const std::vector<glm::vec3> points = {
        {  0,  0,  0 }, {  5,  1, -3 }, { -4,  2,  1 }, {  2, -1,  5 },
        { -3,  0, -5 }, {  6,  3,  2 }, { -5,  1,  4 }, {  1,  4, -2 },
    };

    std::vector<PickedPoint> exact;
    for (const glm::vec3 &p : points)
        exact.push_back({ p, projectToPixel(view, p, fov, width, height) });

    // A solver that round-trips synthetic projections back to the camera that
    // made them has its conventions (intrinsics, axis flips, inversion) right.
    const glm::vec3 trueForward = glm::normalize(target - eye);
    auto poseMatches = [&](const std::optional<Waypoint> &pose) {
        if (!pose)
            return false;
        const float positionError = glm::length(pose->position - eye);
        const glm::vec3 forward = glm::normalize(pose->target - pose->position);
        return positionError < 0.05f && glm::dot(forward, trueForward) > 0.999f;
    };

    check(poseMatches(computeCameraPose(exact, fov, width, height)),
          "SQPnP: exact correspondences round-trip to the true pose");

    // RANSAC: corrupt three of eleven pairs far beyond the 8px inlier band --
    // a plain least-squares solve would be dragged off; consensus must not be.
    std::vector<PickedPoint> withOutliers = exact;
    withOutliers.push_back({ { 4, 2, 4 },
                             projectToPixel(view, { 4, 2, 4 }, fov, width, height) });
    withOutliers.push_back({ { -2, 3, 3 },
                             projectToPixel(view, { -2, 3, 3 }, fov, width, height) });
    withOutliers.push_back({ { 3, 1, -4 },
                             projectToPixel(view, { 3, 1, -4 }, fov, width, height) });
    withOutliers[2].imagePos += glm::vec2(150.0f, -120.0f);
    withOutliers[6].imagePos += glm::vec2(-200.0f, 90.0f);
    withOutliers[9].imagePos += glm::vec2(80.0f, 160.0f);

    check(poseMatches(computeCameraPoseRansac(withOutliers, fov, width, height)),
          "RANSAC: recovers the true pose despite 3 wrong correspondences");
}

int main()
{
    testNormals();
    testCentroids();
    testPnp();

    if (failures == 0)
        std::cout << "All checks passed." << std::endl;
    else
        std::cout << failures << " check(s) FAILED." << std::endl;
    return failures;
}
