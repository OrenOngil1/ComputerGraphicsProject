#pragma once

#include <string>
#include <vector>

#include "FeatureMatching.h"   // FeatureDb
#include "../core/Camera.h"    // Waypoint

// On-disk storage for a hand-built feature database (Mode D's pre-phase).
//
// The database is the one thing in this app a human makes by hand, minutes at a
// time, and it is otherwise lost on exit. Saving it makes the database an input
// to a test run rather than part of one. The recorded waypoints travel with it:
// the anchors were placed from those views and the all-waypoints evaluation
// replays them, so a database without them is only half restored.

// Where `terrainFile`'s database lives: captures/featuredb_<terrain>.yml.
std::string featureDbPath(const std::string &terrainFile);

// Write the database, its waypoints, the terrain they belong to, and the
// capture resolution the descriptors were computed at -- that resolution is
// part of the database's identity, since SIFT descriptors shift when the same
// scene is rasterised at a different size. Creates the containing directory.
// False (with a message) if there is nothing to save or the write fails.
bool saveFeatureDb(const std::string &path, const FeatureDb &db,
                   const std::vector<Waypoint> &waypoints, const std::string &terrainFile,
                   int captureWidth, int captureHeight);

// Read a database back, replacing `db`, `waypoints`, and the capture size only
// on success. A file from before the size was recorded loads with 0x0 -- the
// caller falls back to the live viewport and the next save records it.
//
// Refuses a file saved under a different terrain (world-space anchors on one DEM
// mean nothing on another) or one whose descriptor and anchor counts disagree
// (matching indexes the two arrays in lockstep). A malformed file is a refusal
// too, not an exception: this is reached from a GLFW key callback, and
// unwinding through C is undefined.
bool loadFeatureDb(const std::string &path, FeatureDb &db,
                   std::vector<Waypoint> &waypoints, const std::string &terrainFile,
                   int &captureWidth, int &captureHeight);
