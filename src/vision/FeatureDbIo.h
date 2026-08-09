#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "FeatureMatching.h"   // FeatureDb
#include "../core/Camera.h"    // Waypoint

// On-disk storage for a hand-built feature database (Mode D's pre-phase). It is
// the one thing here a human makes by hand, minutes at a time; saving it makes
// the database an input to a test run rather than part of one. The waypoints
// travel with it -- the anchors were placed from those views -- and so does the
// flight path, which is only provenance drawing but reads as a poorer build
// when the dots come back without the line they were flown on.

// Where `terrainFile`'s database lives: captures/featuredb_<terrain>.yml.
std::string featureDbPath(const std::string &terrainFile);

// Write the database, its waypoints, the flight path they were recorded on,
// the terrain they belong to, and the capture resolution the descriptors were
// computed at -- that resolution is part of the database's identity, since
// SIFT descriptors shift when the same scene is rasterised at a different
// size. Creates the containing directory. False (with a message) if there is
// nothing to save or the write fails.
bool saveFeatureDb(const std::string &path, const FeatureDb &db,
                   const std::vector<Waypoint> &waypoints,
                   const std::vector<glm::vec3> &pathPoints,
                   const std::string &terrainFile,
                   int captureWidth, int captureHeight);

// Read a database back, replacing `db`, `waypoints`, `pathPoints`, and the
// capture size only on success (a pre-size file loads as 0x0 and a pre-path
// file as an empty path; the caller falls back). Every failure is a refusal,
// not an exception -- this is reached from a GLFW key callback, and unwinding
// through C is undefined: wrong terrain (world-space anchors mean nothing on
// another DEM), descriptor and anchor counts disagreeing (matching indexes
// them in lockstep), or a malformed file.
bool loadFeatureDb(const std::string &path, FeatureDb &db,
                   std::vector<Waypoint> &waypoints, std::vector<glm::vec3> &pathPoints,
                   const std::string &terrainFile,
                   int &captureWidth, int &captureHeight);
