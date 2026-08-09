// Headless checks for the geometry behind the global map's view aids -- no
// window, no GL context. Three pieces:
//
//   Mesh::heightAt    the terrain surface sampled in centered world space
//   raycastTerrain    how far along a ray that surface is (the view cone's reach)
//   rayDirection      the world direction of an image observation (the sight line)
//
// rayDirection is the one with a silent failure mode: a sign slip on the image
// convention's y axis mirrors every sight line about the view axis, which still
// looks plausible on screen. It is checked against the renderer's own
// projection rather than against itself.

#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "check.h"
#include "../src/core/Camera.h"
#include "../src/core/Scene.h"
#include "../src/core/Simulation.h"
#include "../src/state/State.h"   // completes State: Simulation's unique_ptr member
#include "../src/state/FeatureMatchState.h"   // layOutBuildRing

namespace {

// A cols x rows grid with height h(x, z), matching the loader's layout:
// vertices[z * cols + x] sits at world (x, h, z), uncentered.
template <typename HeightFn>
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

bool near(float a, float b, float tolerance) { return std::fabs(a - b) < tolerance; }

}  // namespace

void testViewAids()
{
    std::cout << "view aids (heightAt / raycastTerrain / rayDirection):" << std::endl;

    // ── Mesh::heightAt ────────────────────────────────────────
    // A 5x5 grid spans grid coords 0..4, centered to world -2.5..1.5.
    const Mesh flat = makeGrid(5, 5, [](int, int) { return 3.0f; });

    check(flat.heightAt(0.0f, 0.0f).value_or(-1.0f) == 3.0f,
          "flat grid: the surface height is the constant, at the center");
    check(flat.heightAt(-2.5f, -2.5f).value_or(-1.0f) == 3.0f,
          "flat grid: and at the near corner (centering applied)");
    check(flat.heightAt(1.5f, 1.5f).value_or(-1.0f) == 3.0f,
          "flat grid: and at the far corner, inclusive");

    check(!flat.heightAt(1.6f, 0.0f) && !flat.heightAt(-2.6f, 0.0f),
          "outside the grid in x: no surface");
    check(!flat.heightAt(0.0f, 1.6f) && !flat.heightAt(0.0f, -2.6f),
          "outside the grid in z: no surface");
    check(!Mesh{}.heightAt(0.0f, 0.0f), "an unloaded mesh has no surface anywhere");

    // A ramp h = 2x, sampled half a cell in: bilinear interpolation must
    // return the midpoint of the two neighbouring heights, not either one.
    const Mesh ramp = makeGrid(5, 5, [](int x, int) { return 2.0f * (float)x; });
    check(near(ramp.heightAt(-1.0f, 0.0f).value_or(-1.0f), 3.0f, 1e-5f),
          "ramp: half a cell in interpolates between the two vertices");
    check(near(ramp.heightAt(-0.5f, 0.0f).value_or(-1.0f), 4.0f, 1e-5f),
          "ramp: on a vertex returns that vertex's height");

    // ── raycastTerrain ────────────────────────────────────────
    // Straight down from 10 onto a surface at 3: the hit is 7 along the ray.
    const std::optional<float> down =
        raycastTerrain(flat, glm::vec3(0.0f, 10.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f),
                       20.0f, 0.1f);
    check(down && near(*down, 7.0f, 0.01f), "straight down: hits the surface at the right distance");

    check(!raycastTerrain(flat, glm::vec3(0.0f, 10.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                          20.0f, 0.1f),
          "aimed at the sky: no hit");
    check(!raycastTerrain(flat, glm::vec3(100.0f, 10.0f, 100.0f), glm::vec3(0.0f, -1.0f, 0.0f),
                          20.0f, 0.1f),
          "dropped outside the grid: no hit");
    check(!raycastTerrain(flat, glm::vec3(0.0f, 10.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f),
                          5.0f, 0.1f),
          "surface beyond maxDistance: no hit");

    // A shallow descent that starts off the grid and flies in: the march must
    // not give up at the first off-grid sample.
    const std::optional<float> entering =
        raycastTerrain(flat, glm::vec3(-8.0f, 5.0f, 0.0f),
                       glm::normalize(glm::vec3(1.0f, -0.25f, 0.0f)), 30.0f, 0.05f);
    check(entering.has_value(), "a ray entering the grid from outside still hits");

    // ── rayDirection ──────────────────────────────────────────
    const Camera camera{ { 12, 9, 15 }, { 1, 2, 0 }, { 0, 1, 0 }, 50.0f, 0.1f, 1000.0f };
    const Viewport landscape{ 0, 0, 960, 640 };

    check(glm::length(rayDirection(camera, glm::vec2(0.0f))
                      - glm::normalize(camera.target - camera.position)) < 1e-5f,
          "the center ray points straight at the target");

    // The property the sight line rests on: a point placed along the direction
    // of uv's ray rasterizes back to uv. Includes the four image corners, which
    // are the view cone's edges.
    const glm::vec2 uvs[] = {
        { 0.5f, 0.5f }, { 0.2f, 0.8f }, { 0.75f, 0.3f },
        { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 1.0f },
    };
    bool landsOnItsPixel = true;
    for (const glm::vec2 &uv : uvs) {
        const glm::vec2 ray = fractionToRay(uv, camera.fov, landscape.aspect());
        const glm::vec3 point = camera.position + rayDirection(camera, ray) * 50.0f;
        const glm::vec2 expected = uv * glm::vec2((float)landscape.width,
                                                  (float)landscape.height);
        landsOnItsPixel = landsOnItsPixel
                       && glm::length(rasterize(camera, landscape, point) - expected) < 0.1f;
    }
    check(landsOnItsPixel, "a sight line through uv rasterizes back onto uv (corners included)");

    // Distance along the ray must not move the pixel -- that is exactly the
    // ambiguity the sight line is drawn to expose.
    const glm::vec2 ray = fractionToRay(glm::vec2(0.3f, 0.7f), camera.fov, landscape.aspect());
    const glm::vec3 direction = rayDirection(camera, ray);
    check(glm::length(rasterize(camera, landscape, camera.position + direction * 20.0f)
                      - rasterize(camera, landscape, camera.position + direction * 400.0f)) < 0.1f,
          "every point along a sight line shares one pixel (the depth is unrecoverable)");

    // A y-sign slip would mirror the line about the view axis: the top of the
    // frame must map to the world-up side, not the down side.
    const glm::vec2 topRay = fractionToRay(glm::vec2(0.5f, 0.0f), camera.fov, landscape.aspect());
    const glm::vec2 bottomRay = fractionToRay(glm::vec2(0.5f, 1.0f), camera.fov, landscape.aspect());
    check(glm::dot(rayDirection(camera, topRay), cameraBasis(camera).up) > 0.0f &&
          glm::dot(rayDirection(camera, bottomRay), cameraBasis(camera).up) < 0.0f,
          "the top of the frame maps above the view axis, the bottom below");

    // ── isInFrame ─────────────────────────────────────────────
    // Mode D counts the database anchors a view can use, so the boundary is the
    // whole point: an off-by-a-frustum-plane here quietly over- or under-reports
    // how good a capture position is.
    check(isInFrame(camera, landscape, camera.target),
          "what the camera looks at is in frame");
    check(!isInFrame(camera, landscape,
                     camera.position - glm::normalize(camera.target - camera.position) * 10.0f),
          "a point behind the camera is not in frame");
    check(!isInFrame(camera, landscape, camera.position),
          "and neither is the camera's own position");

    // Built from fractionToRay, so this also holds the frustum to the ray
    // mapping checked above: a point placed just inside a frame edge must be
    // in, and one just outside it must be out.
    const glm::vec2 edges[] = { { 0.5f, 0.0f }, { 0.5f, 1.0f },
                                { 0.0f, 0.5f }, { 1.0f, 0.5f } };
    bool edgesAgree = true;
    for (const glm::vec2 &edge : edges) {
        const glm::vec2 toward = glm::mix(edge, glm::vec2(0.5f), 0.02f);   // 2% inside
        const glm::vec2 beyond = glm::mix(edge, glm::vec2(0.5f), -0.05f);  // 5% outside
        const float reach = 60.0f;
        edgesAgree = edgesAgree &&
            isInFrame(camera, landscape,
                      camera.position +
                          rayDirection(camera, fractionToRay(toward, camera.fov,
                                                             landscape.aspect())) * reach) &&
            !isInFrame(camera, landscape,
                       camera.position +
                           rayDirection(camera, fractionToRay(beyond, camera.fov,
                                                              landscape.aspect())) * reach);
    }
    check(edgesAgree, "each frame edge separates in-frame from out on the right side");

    // Depth clipping: the frustum is a volume, not a wedge. A point on the
    // center ray but past the far plane is not drawn, so it is not in frame.
    const glm::vec3 forward = glm::normalize(camera.target - camera.position);
    check(isInFrame(camera, landscape, camera.position + forward * (camera.far * 0.5f)),
          "a point between the clip planes is in frame");
    check(!isInFrame(camera, landscape, camera.position + forward * (camera.far * 1.5f)),
          "a point beyond the far plane is not");

    // ── the V dial ────────────────────────────────────────────
    // The order is a contract: the sight lines and the click snap are tied to
    // Full, so a wraparound slip would leave a user "one press past off" being
    // silently assisted again.
    Simulation sim;
    check(sim.viewAids == ViewAids::Full, "the aids start at full");
    sim.cycleViewAids();
    check(sim.viewAids == ViewAids::ConeOnly, "one press: cone only");
    sim.cycleViewAids();
    check(sim.viewAids == ViewAids::Off, "two presses: off");
    sim.cycleViewAids();
    check(sim.viewAids == ViewAids::Full, "three presses: back to full");

    // ── the calibrated build ring ─────────────────────────────
    // O in RECORD mode replaces the freehand flight; the numbers ARE the
    // calibration, so they are pinned: 16 stops (22.5-degree steps, inside the
    // measured recognition range), all at one height, one radius, aimed at the
    // centre.
    sim.terrainSize = 633.0f;
    layOutBuildRing(sim);
    check(sim.waypoints.size() == 16, "the build ring lays 16 stations");
    check(sim.pathPoints.size() == 17, "and closes its map trace back to the start");
    bool ringHolds = true;
    for (const Waypoint &w : sim.waypoints) {
        ringHolds = ringHolds && near(w.position.y, 0.25f * sim.terrainSize, 0.01f);
        ringHolds = ringHolds && near(glm::length(glm::vec2(w.position.x, w.position.z)),
                                      0.30f * sim.terrainSize, 0.01f);
        ringHolds = ringHolds && glm::length(w.target) < 1e-5f;
    }
    check(ringHolds, "every station sits at the calibrated height and radius, aimed at the centre");

    // ── nearestRecordedView ───────────────────────────────────
    // The envelope readout: which recorded view the camera is closest to, and
    // by how much in eye distance and heading. Recognition depends on both, so
    // a wrong pick or a mirrored angle would coach the pilot into dead air.
    const std::vector<Waypoint> views = {
        { { 0, 0, 0 }, { 10, 0, 0 } },      // at the origin, looking +x
        { { 100, 0, 0 }, { 100, 0, 10 } },  // far away, looking +z
    };
    Camera probe{ { 3, 0, 0 }, { 20, 0, 0 }, { 0, 1, 0 }, 45.0f, 0.1f, 1000.0f };

    check(!nearestRecordedView({}, probe).has_value(),
          "no recorded views: no nearest view");

    std::optional<NearestView> nv = nearestRecordedView(views, probe);
    check(nv && nv->index == 0 && near(nv->distanceUnits, 3.0f, 1e-4f),
          "the closer view wins, at the eye-to-eye distance");
    check(nv && near(nv->headingOffDegrees, 0.0f, 0.01f),
          "matching the view's heading reads as zero degrees off");

    probe.target = glm::vec3(3, 0, 10);   // same spot, turned 90 degrees to +z
    nv = nearestRecordedView(views, probe);
    check(nv && nv->index == 0 && near(nv->headingOffDegrees, 90.0f, 0.01f),
          "turning away from the view's heading is measured in degrees");

    probe.position = glm::vec3(98, 0, 0);   // now beside the far view ...
    probe.target   = glm::vec3(98, 0, 10);  // ... and looking its way (+z)
    nv = nearestRecordedView(views, probe);
    check(nv && nv->index == 1 && near(nv->distanceUnits, 2.0f, 1e-4f) &&
              near(nv->headingOffDegrees, 0.0f, 0.01f),
          "moving across the map hands the readout to the other view");
}
