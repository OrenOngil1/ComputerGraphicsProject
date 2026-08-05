#include "FeatureDbIo.h"

#include <filesystem>
#include <iostream>

#include <opencv2/core.hpp>

// glm vectors <-> plain float matrices. cv::FileStorage serialises cv::Mat
// directly, so packing the two glm arrays into one row-per-item Mat each keeps
// the file readable and the reader's shape check trivial (rows = count, cols =
// the fixed width below).
namespace {

constexpr int kAnchorCols     = 3;     // x, y, z
constexpr int kWaypointCols   = 6;     // position xyz, target xyz
constexpr int kDescriptorCols = 128;   // one SIFT descriptor per row

cv::Mat anchorsToMat(const std::vector<glm::vec3> &anchors)
{
    cv::Mat m((int)anchors.size(), kAnchorCols, CV_32F);
    for (int i = 0; i < m.rows; i++) {
        m.at<float>(i, 0) = anchors[i].x;
        m.at<float>(i, 1) = anchors[i].y;
        m.at<float>(i, 2) = anchors[i].z;
    }
    return m;
}

cv::Mat waypointsToMat(const std::vector<Waypoint> &waypoints)
{
    cv::Mat m((int)waypoints.size(), kWaypointCols, CV_32F);
    for (int i = 0; i < m.rows; i++) {
        const Waypoint &w = waypoints[i];
        m.at<float>(i, 0) = w.position.x;
        m.at<float>(i, 1) = w.position.y;
        m.at<float>(i, 2) = w.position.z;
        m.at<float>(i, 3) = w.target.x;
        m.at<float>(i, 4) = w.target.y;
        m.at<float>(i, 5) = w.target.z;
    }
    return m;
}

// A Mat is usable as an array of `cols`-wide rows only if it has exactly that
// shape and type; anything else came from a file we did not write.
bool hasShape(const cv::Mat &m, int cols, int type)
{
    return !m.empty() && m.cols == cols && m.type() == type;
}

}  // namespace

std::string featureDbPath(const std::string &terrainFile)
{
    const std::string stem = std::filesystem::path(terrainFile).stem().string();
    return "captures/featuredb_" + (stem.empty() ? "terrain" : stem) + ".yml";
}

bool saveFeatureDb(const std::string &path, const FeatureDb &db,
                   const std::vector<Waypoint> &waypoints, const std::string &terrainFile,
                   int captureWidth, int captureHeight)
{
    if (db.empty() || db.descriptors.rows != (int)db.anchors.size()) {
        std::cout << "FEATURES: nothing to save -- build a database with G first"
                  << std::endl;
        return false;
    }

    try {
        std::error_code ec;
        const std::filesystem::path parent = std::filesystem::path(path).parent_path();
        if (!parent.empty())
            std::filesystem::create_directories(parent, ec);   // already-exists is not an error

        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            std::cerr << "FEATURES: could not write " << path << std::endl;
            return false;
        }

        fs << "terrain"       << terrainFile;
        fs << "captureWidth"  << captureWidth;
        fs << "captureHeight" << captureHeight;
        fs << "descriptors"   << db.descriptors;
        fs << "anchors"       << anchorsToMat(db.anchors);
        fs << "waypoints"     << waypointsToMat(waypoints);
        fs.release();
    } catch (const cv::Exception &e) {
        std::cerr << "FEATURES: could not write " << path << " (" << e.what() << ")"
                  << std::endl;
        return false;
    }

    std::cout << "FEATURES: saved " << db.anchors.size() << " anchors and "
              << waypoints.size() << " waypoints to " << path << std::endl;
    return true;
}

bool loadFeatureDb(const std::string &path, FeatureDb &db,
                   std::vector<Waypoint> &waypoints, const std::string &terrainFile,
                   int &captureWidth, int &captureHeight)
{
    // Read into locals; the outputs are only touched once everything checks
    // out, so a bad file leaves a good in-memory database alone.
    std::string savedTerrain;
    cv::Mat descriptors, anchorMat, waypointMat;
    int savedWidth = 0, savedHeight = 0;

    try {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cout << "FEATURES: no saved database at " << path << std::endl;
            return false;
        }

        fs["terrain"]     >> savedTerrain;
        fs["descriptors"] >> descriptors;
        fs["anchors"]     >> anchorMat;
        fs["waypoints"]   >> waypointMat;
        // Absent in files from before the size was recorded: load as 0x0 and
        // let the caller fall back to the live viewport.
        if (!fs["captureWidth"].empty())
            fs["captureWidth"] >> savedWidth;
        if (!fs["captureHeight"].empty())
            fs["captureHeight"] >> savedHeight;
    } catch (const cv::Exception &e) {
        std::cerr << "FEATURES: " << path << " is not readable (" << e.what() << ")"
                  << std::endl;
        return false;
    }

    if (savedTerrain != terrainFile) {
        std::cerr << "FEATURES: " << path << " was built on " << savedTerrain
                  << ", not " << terrainFile << " -- its anchors are points on that"
                  << " terrain and would be meaningless here" << std::endl;
        return false;
    }

    if (!hasShape(anchorMat, kAnchorCols, CV_32F) ||
        !hasShape(descriptors, kDescriptorCols, CV_32F) ||
        descriptors.rows != anchorMat.rows) {
        std::cerr << "FEATURES: " << path << " is not usable -- descriptors must be "
                  << kDescriptorCols << "-float SIFT rows pairing one to one with"
                  << " anchors (a database saved by the earlier ORB build must be"
                  << " rebuilt with G and saved again)" << std::endl;
        return false;
    }

    std::vector<Waypoint> loadedWaypoints;
    if (!waypointMat.empty()) {
        if (!hasShape(waypointMat, kWaypointCols, CV_32F)) {
            std::cerr << "FEATURES: " << path << " has malformed waypoints" << std::endl;
            return false;
        }
        loadedWaypoints.reserve(waypointMat.rows);
        for (int i = 0; i < waypointMat.rows; i++) {
            const float *row = waypointMat.ptr<float>(i);
            loadedWaypoints.push_back({ glm::vec3(row[0], row[1], row[2]),
                                        glm::vec3(row[3], row[4], row[5]) });
        }
    }

    db.descriptors = descriptors;
    db.anchors.clear();
    db.anchors.reserve(anchorMat.rows);
    for (int i = 0; i < anchorMat.rows; i++) {
        const float *row = anchorMat.ptr<float>(i);
        db.anchors.push_back(glm::vec3(row[0], row[1], row[2]));
    }
    waypoints     = std::move(loadedWaypoints);
    captureWidth  = savedWidth;
    captureHeight = savedHeight;

    std::cout << "FEATURES: loaded " << db.anchors.size() << " anchors and "
              << waypoints.size() << " waypoints from " << path << std::endl;
    return true;
}
