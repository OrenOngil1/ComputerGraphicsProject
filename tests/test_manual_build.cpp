// Headless checks for ManualBuild -- Mode D's hand build, driven through a stub
// FrameCapture over a synthetic mesh. No window, no GL context.
//
// The build is the one place a human's work enters the database, and its data
// is four parallel sequences: a descriptor row, a 3D anchor, the marker index
// that produced them, and the placement error measured at the time. They are
// pushed together by place() and popped together by undo(), and nothing else
// enforces that they stay the same length -- a drift there files a descriptor
// under someone else's 3D point, which no later stage can detect.
//
// The other claim worth pinning is the walk: views with nothing to place must
// be stepped over silently, and the build must report finished() rather than
// running off the end of its waypoint list.

#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "check.h"
#include "../src/state/ManualBuild.h"

namespace {

using HeightFn = float (*)(int, int);

Mesh makeGrid(int cols, int rows, HeightFn h)
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

FramePixels uniformFrame(int width, int height, unsigned char value)
{
    FramePixels frame;
    frame.width  = width;
    frame.height = height;
    frame.rgb.assign((size_t)width * height * 3, value);
    return frame;
}

void paintSquare(FramePixels &frame, int cx, int cy, int half)
{
    for (int y = cy - half; y <= cy + half; y++)
        for (int x = cx - half; x <= cx + half; x++) {
            unsigned char *px = &frame.rgb[((size_t)y * frame.width + x) * 3];
            px[0] = px[1] = px[2] = 0;
        }
}

// Six well-separated squares. Softened for the same reason test_features.cpp
// softens: SIFT collapses razor-edged binary squares into stacks of duplicate
// keypoints, and the production input is a smooth-shaded render that never
// contains a step edge. `variant` shifts the layout so two views do not hand
// back the same descriptors -- which the build would rightly drop as
// re-detections of already-anchored points.
FramePixels featureFrame(int width, int height, int variant)
{
    FramePixels frame = uniformFrame(width, height, 255);
    const int offset = variant * 23;
    const int spots[][2] = { { 90, 90 }, { 250, 70 }, { 410, 100 },
                             { 100, 260 }, { 300, 300 }, { 420, 400 } };
    for (const auto &s : spots)
        paintSquare(frame, s[0] + offset, s[1], 14 + variant * 3);

    cv::Mat rgb(frame.height, frame.width, CV_8UC3, frame.rgb.data());
    cv::GaussianBlur(rgb, rgb, cv::Size(), 2.0);
    return frame;
}

const Camera   kPrototype{ { 0, 0, 0 }, { 0, 0, -1 }, { 0, 1, 0 }, 60.0f, 0.1f, 500.0f };
const Viewport kFrame{ 0, 0, 512, 512 };
constexpr float kTerrainSize = 64.0f;

Waypoint viewAt(float x, float z) { return { glm::vec3(x, 40.0f, z), glm::vec3(0.0f) }; }

// The 3D point a click would land on: the active suggestion's ray, taken to a
// fixed depth. Stands in for the user picking on the global map.
glm::vec3 anchorOnRay(const Waypoint &view, const glm::vec2 &ray, float depth)
{
    Camera camera = kPrototype;
    camera.applyPose(view);
    return camera.position + rayDirection(camera, ray) * depth;
}

}  // namespace

void testManualBuild()
{
    std::cout << "ManualBuild (Mode D's hand build):" << std::endl;

    const Mesh mesh = makeGrid(64, 64, [](int, int) { return 0.0f; });

    // Every view renders its own layout; the middle one of the three-view walk
    // renders flat grey, which SIFT finds nothing in.
    const std::vector<Waypoint> views = { viewAt(-20, -20), viewAt(0, -20), viewAt(20, -20) };
    const glm::vec3 blankAt = views[1].position;

    const FrameCapture render = [&](const Camera &camera, const Viewport &vp) {
        if (glm::distance(camera.position, blankAt) < 0.5f)
            return uniformFrame(vp.width, vp.height, 128);
        const int variant = camera.position.x < 0.0f ? 0 : 2;
        return featureFrame(vp.width, vp.height, variant);
    };

    const ManualBuild::Context ctx{ mesh, kTerrainSize, kPrototype, 5, render };

    // ── an empty waypoint list finishes immediately ───────────
    {
        ManualBuild build({}, kFrame, ctx);
        check(build.finished(), "no recorded views: the build is finished on construction");
        check(build.viewCount() == 0, "no recorded views: nothing to walk");
    }

    // ── the walk ──────────────────────────────────────────────
    {
        ManualBuild build(views, kFrame, ctx);
        check(!build.finished(), "a placeable first view leaves the build unfinished");
        check(build.viewIndex() == 0, "the walk starts at the first view");
        check(build.viewCount() == 3, "the walk knows how many views it has");
        check(build.frame().width == kFrame.width && build.frame().height == kFrame.height,
              "the build keeps the frame size it was given");
        check(build.currentPose().position == views[0].position,
              "currentPose is the view being anchored");

        // Ctrl+X off the first view: the featureless second must be stepped
        // over, landing on the third rather than stopping on nothing.
        build.skipRestOfView(ctx);
        check(build.viewIndex() == 2, "a view with no features is skipped, not stopped on");
        check(!build.finished(), "skipping onto a placeable view does not finish the build");
        check(!build.markers().empty(), "the view reached after the skip offers suggestions");

        build.skipRestOfView(ctx);
        check(build.finished(), "skipping the last view finishes the build");
        check(build.viewIndex() == build.viewCount(),
              "a finished build has walked every view exactly once");
    }

    // ── suggestions ───────────────────────────────────────────
    ManualBuild build(views, kFrame, ctx);

    const size_t offered = build.markers().size();
    check(offered >= 4 && offered <= 5, "the view offers at most the requested suggestions");
    check(build.activeMarker() == 0, "the first suggestion is active");

    bool inFrame = true;
    for (const glm::vec2 &m : build.markers())
        inFrame = inFrame && m.x >= 0.0f && m.x <= 1.0f && m.y >= 0.0f && m.y <= 1.0f;
    check(inFrame, "markers are fractions of the frame");

    check(build.anchoredMarkers().empty() && build.database().empty(),
          "a fresh view starts with nothing anchored");

    // The ray the caller snaps along must be the one place() measures against,
    // or the debrief scores a different line than the user aimed down.
    const glm::vec2 ray = build.activeRay(kPrototype.fov);
    const glm::vec2 expected = fractionToRay(build.markers()[0], kPrototype.fov,
                                             build.frame().aspect());
    check(std::fabs(ray.x - expected.x) < 1e-6f && std::fabs(ray.y - expected.y) < 1e-6f,
          "activeRay is the active marker's ray through the build's own frame");

    if (offered < 4) {
        check(false, "fixture: need at least four suggestions to exercise undo across a skip");
        return;
    }

    // ── place, place, skip ────────────────────────────────────
    //
    // The skip comes LAST on purpose. Undo must reactivate the marker it took
    // back, and the only sequence that tells that apart from a plain
    // `active--` is one where the suggestion immediately before the cursor was
    // skipped rather than placed -- decrementing would hand the user a
    // suggestion they had already passed on, and silently leave the anchor
    // they wanted back unrecoverable.
    build.place(anchorOnRay(views[0], ray, 45.0f), ray, ctx);
    check(build.database().anchors.size() == 1 &&
          build.database().descriptors.rows == 1 &&
          build.anchoredMarkers().size() == 1,
          "placing an anchor grows every parallel sequence by one");
    check(build.anchoredMarkers().back() == 0, "the anchor records which marker produced it");
    check(build.activeMarker() == 1, "placing advances to the next suggestion");

    const glm::vec2 ray2 = build.activeRay(kPrototype.fov);
    build.place(anchorOnRay(views[0], ray2, 55.0f), ray2, ctx);
    check(build.database().anchors.size() == 2 && build.database().descriptors.rows == 2,
          "the second anchor keeps rows and points in lockstep");
    check(build.activeMarker() == 2, "the cursor is on the third suggestion");

    build.skipSuggestion(ctx);
    check(build.activeMarker() == 3, "skipping advances without anchoring");
    check(build.database().anchors.size() == 2, "skipping leaves the database alone");

    // ── undo ──────────────────────────────────────────────────
    const glm::vec3 firstAnchor = build.database().anchors[0];

    build.undo();
    check(build.database().anchors.size() == 1 &&
          build.database().descriptors.rows == 1 &&
          build.anchoredMarkers().size() == 1,
          "undo unwinds every parallel sequence together");
    check(build.activeMarker() == 1,
          "undo reactivates the marker it took back, stepping over the skipped one");
    check(build.database().anchors[0] == firstAnchor, "undo leaves the earlier anchor intact");

    build.undo();
    check(build.database().empty() && build.anchoredMarkers().empty(),
          "undoing back to the start empties the database");
    check(build.activeMarker() == 0, "undo returns to the first placed marker");

    build.undo();
    check(build.database().empty() && build.activeMarker() == 0,
          "undo with nothing placed is a no-op");

    // ── handover ──────────────────────────────────────────────
    const glm::vec2 ray3 = build.activeRay(kPrototype.fov);
    build.place(anchorOnRay(views[0], ray3, 50.0f), ray3, ctx);

    ManualBuild::Result result = build.takeResult();
    check(result.db.anchors.size() == 1 &&
          result.db.anchors.size() == result.placementErrors.size(),
          "the handover carries one placement error per anchor");
    check(result.db.descriptors.rows == (int)result.db.anchors.size(),
          "the handed-over database has a descriptor row per anchor");
}
