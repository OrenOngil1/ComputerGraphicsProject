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

// A small database with distinguishable rows: descriptor values track the
// row index, so a row landing in the wrong place is visible. Shaped like the
// real thing (128-float SIFT rows), because the loader refuses anything else.
FeatureDb makeDb(int rows)
{
    FeatureDb db;
    db.descriptors = cv::Mat(rows, 128, CV_32F);
    for (int i = 0; i < rows; i++) {
        db.descriptors.row(i).setTo((float)(i * 7 + 1));
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

    const fs::path dir = fs::temp_directory_path() / "visual_slammer_featuredb_test";
    fs::create_directories(dir);
    const std::string path = (dir / "db.yml").string();
    const std::string terrain = "assets/terrains/terrain1.jpg";

    const FeatureDb original = makeDb(6);
    const std::vector<Waypoint> waypoints = {
        { { 1, 2, 3 }, { 4, 5, 6 } },
        { { -7, 8, -9 }, { 0, 1, 2 } },
    };

    check(saveFeatureDb(path, original, waypoints, terrain, 960, 540),
          "a built database saves");

    FeatureDb loaded;
    std::vector<Waypoint> loadedWaypoints;
    int loadedW = 0, loadedH = 0;
    check(loadFeatureDb(path, loaded, loadedWaypoints, terrain, loadedW, loadedH),
          "and loads back");
    check(sameDb(original, loaded), "descriptors and anchors survive the round trip exactly");
    check(loadedWaypoints == waypoints,
          "the waypoints travel with it (the views the anchors were placed from)");
    check(loadedW == 960 && loadedH == 540,
          "the capture resolution travels with it (the size its descriptors need)");

    // A database anchored on one DEM describes points that do not exist on
    // another, so loading it there must fail rather than localise onto nothing.
    FeatureDb other = makeDb(3);
    std::vector<Waypoint> otherWaypoints;
    int otherW = 0, otherH = 0;
    check(!loadFeatureDb(path, other, otherWaypoints, "assets/terrains/terrain2.png",
                         otherW, otherH),
          "a database built on another terrain is refused");
    check(other.anchors.size() == 3, "and the refusal leaves the current database intact");

    // Garbage in, refusal out -- this is reached from a GLFW key callback, so an
    // exception escaping would unwind through C.
    const std::string junkPath = (dir / "junk.yml").string();
    std::ofstream(junkPath) << "this is not a YAML FileStorage document {{{\n";
    check(!loadFeatureDb(junkPath, other, otherWaypoints, terrain, otherW, otherH),
          "a malformed file is refused, not thrown out of");

    check(!loadFeatureDb((dir / "nope.yml").string(), other, otherWaypoints, terrain,
                         otherW, otherH),
          "a missing file is refused");

    // A valid file from before the capture size was recorded must load, with
    // the size reported as 0x0 so the caller knows to fall back.
    const std::string presizePath = (dir / "presize.yml").string();
    {
        cv::FileStorage fs(presizePath, cv::FileStorage::WRITE);
        fs << "terrain" << terrain << "descriptors" << original.descriptors
           << "anchors" << cv::Mat((int)original.anchors.size(), 3, CV_32F,
                                   (void *)original.anchors.data()).clone();
    }
    FeatureDb presize;
    int presizeW = 7, presizeH = 7;
    check(loadFeatureDb(presizePath, presize, otherWaypoints, terrain, presizeW, presizeH) &&
              presizeW == 0 && presizeH == 0,
          "a file without a recorded capture size loads and reports 0x0");

    // A file from the earlier ORB build carries 32-byte binary rows; SIFT
    // matching cannot use them, so the load must refuse rather than hand the
    // matcher rows of the wrong type.
    const std::string orbPath = (dir / "orb-era.yml").string();
    {
        cv::Mat orbRows(4, 32, CV_8U, cv::Scalar(7));
        cv::Mat anchorRows(4, 3, CV_32F, cv::Scalar(1.0f));
        cv::FileStorage fs(orbPath, cv::FileStorage::WRITE);
        fs << "terrain" << terrain << "descriptors" << orbRows << "anchors" << anchorRows;
    }
    check(!loadFeatureDb(orbPath, other, otherWaypoints, terrain, otherW, otherH),
          "a database saved by the ORB-era build is refused");

    // A database with several appearances of one place -- the normal state
    // after addOtherViewAppearances -- must keep its repeats: collapsing them
    // on the round trip would quietly undo the collection.
    const std::string variantPath = (dir / "variants.yml").string();
    FeatureDb multi = makeDb(4);
    multi.descriptors.push_back(multi.descriptors.row(1).clone());
    multi.anchors.push_back(multi.anchors[1]);
    check(saveFeatureDb(variantPath, multi, waypoints, terrain, 960, 540),
          "a multi-appearance database saves");
    FeatureDb multiBack;
    std::vector<Waypoint> multiWps;
    check(loadFeatureDb(variantPath, multiBack, multiWps, terrain, otherW, otherH) &&
              sameDb(multi, multiBack) && multiBack.places().size() == 4,
          "five appearances of four places survive exactly");

    // Zero waypoints is legal on disk and must not block the anchors; a
    // preloaded waypoint list must come back empty, not stale.
    check(saveFeatureDb(variantPath, original, {}, terrain, 960, 540),
          "a database saves with no waypoints");
    FeatureDb noWp;
    std::vector<Waypoint> noWpWps = waypoints;
    check(loadFeatureDb(variantPath, noWp, noWpWps, terrain, otherW, otherH) &&
              sameDb(original, noWp) && noWpWps.empty(),
          "and loads back with an empty waypoint list");

    check(!saveFeatureDb(path, FeatureDb{}, waypoints, terrain, 960, 540),
          "an empty database is not written over a good one");
    FeatureDb stillThere;
    std::vector<Waypoint> stillWaypoints;
    check(loadFeatureDb(path, stillThere, stillWaypoints, terrain, otherW, otherH) &&
          sameDb(original, stillThere),
          "the saved database is still the one that was built");

    fs::remove_all(dir);
}
