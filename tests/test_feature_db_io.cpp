// Headless checks for saveFeatureDb / loadFeatureDb -- real files in a temp
// dir, no GL.
//
// What is at stake here is human work. A hand-built database is thirty-odd
// points a person placed one at a time, and this is the only thing standing
// between that and having to do it again. So the round trip has to be exact,
// and every refusal has to be a refusal rather than a half-load: the two arrays
// are indexed in lockstep by the matcher, and a file that survives with
// mismatched counts would anchor descriptors to other points' 3D positions.

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "check.h"
#include "../src/vision/FeatureDbIo.h"

namespace {

namespace fs = std::filesystem;

// A small database with distinguishable rows: descriptor byte values track the
// row index, so a row landing in the wrong place is visible.
FeatureDb makeDb(int rows)
{
    FeatureDb db;
    db.descriptors = cv::Mat(rows, 32, CV_8U);
    for (int i = 0; i < rows; i++) {
        db.descriptors.row(i).setTo((unsigned char)(i * 7 + 1));
        db.anchors.push_back(glm::vec3((float)i, (float)i * 2.5f, (float)-i));
    }
    return db;
}

bool sameDb(const FeatureDb &a, const FeatureDb &b)
{
    if (a.anchors.size() != b.anchors.size() || a.descriptors.rows != b.descriptors.rows ||
        a.descriptors.cols != b.descriptors.cols || a.descriptors.type() != b.descriptors.type())
        return false;
    for (size_t i = 0; i < a.anchors.size(); i++)
        if (a.anchors[i] != b.anchors[i])
            return false;
    return cv::countNonZero(a.descriptors != b.descriptors) == 0;
}

}  // namespace

void testFeatureDbIo()
{
    std::cout << "saveFeatureDb / loadFeatureDb:" << std::endl;

    const fs::path dir = fs::temp_directory_path() / "drone_sim_featuredb_test";
    fs::create_directories(dir);
    const std::string path = (dir / "db.yml").string();
    const std::string terrain = "assets/terrains/terrain1.jpg";

    const FeatureDb original = makeDb(6);
    const std::vector<Waypoint> waypoints = {
        { { 1, 2, 3 }, { 4, 5, 6 } },
        { { -7, 8, -9 }, { 0, 1, 2 } },
    };

    check(saveFeatureDb(path, original, waypoints, terrain), "a built database saves");

    FeatureDb loaded;
    std::vector<Waypoint> loadedWaypoints;
    check(loadFeatureDb(path, loaded, loadedWaypoints, terrain), "and loads back");
    check(sameDb(original, loaded), "descriptors and anchors survive the round trip exactly");
    check(loadedWaypoints == waypoints,
          "the waypoints travel with it (the views the anchors were placed from)");

    // A database anchored on one DEM describes points that do not exist on
    // another, so loading it there must fail rather than localise onto nothing.
    FeatureDb other = makeDb(3);
    std::vector<Waypoint> otherWaypoints;
    check(!loadFeatureDb(path, other, otherWaypoints, "assets/terrains/terrain2.png"),
          "a database built on another terrain is refused");
    check(other.anchors.size() == 3, "and the refusal leaves the current database intact");

    // Garbage in, refusal out -- this is reached from a GLFW key callback, so an
    // exception escaping would unwind through C.
    const std::string junkPath = (dir / "junk.yml").string();
    std::ofstream(junkPath) << "this is not a YAML FileStorage document {{{\n";
    check(!loadFeatureDb(junkPath, other, otherWaypoints, terrain),
          "a malformed file is refused, not thrown out of");

    check(!loadFeatureDb((dir / "nope.yml").string(), other, otherWaypoints, terrain),
          "a missing file is refused");

    check(!saveFeatureDb(path, FeatureDb{}, waypoints, terrain),
          "an empty database is not written over a good one");
    FeatureDb stillThere;
    std::vector<Waypoint> stillWaypoints;
    check(loadFeatureDb(path, stillThere, stillWaypoints, terrain) &&
          sameDb(original, stillThere),
          "the saved database is still the one that was built");

    fs::remove_all(dir);
}
