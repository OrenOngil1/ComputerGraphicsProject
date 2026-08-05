#pragma once

#include <string>
#include <vector>

#include "FeatureMatching.h"   // FeatureDb
#include "../core/Camera.h"    // Waypoint

// On-disk storage for a hand-built feature database (Mode D's pre-phase).
//
// The database is the one thing in this app a human makes by hand, minutes at a
// time, and it is otherwise lost on exit -- so every experiment that should be
// "change one thing, measure again" turns into "place thirty points again
// first". Saving it makes the database an input to a test run rather than part
// of one.
//
// The recorded waypoints travel with it. They are not needed to match against
// the database, but the anchors were placed from those views and the
// all-waypoints evaluation replays them, so a database without them is only
// half restored.

// Where `terrainFile`'s database lives: captures/featuredb_<terrain>.yml.
std::string featureDbPath(const std::string &terrainFile);

// Write the database, its waypoints, and the terrain they belong to. Creates
// the containing directory. False (with a message) if there is nothing to save
// or the file cannot be written.
bool saveFeatureDb(const std::string &path, const FeatureDb &db,
                   const std::vector<Waypoint> &waypoints, const std::string &terrainFile);

// Read a database back, replacing `db` and `waypoints` only on success.
//
// Refuses a file saved under a different terrain -- the anchors are world-space
// points on one DEM and mean nothing on another -- and refuses one whose
// descriptor and anchor counts disagree, since matching indexes the two arrays
// in lockstep. A malformed file is a refusal too, not an exception: this is
// reached from a GLFW key callback, and unwinding through C is undefined.
bool loadFeatureDb(const std::string &path, FeatureDb &db,
                   std::vector<Waypoint> &waypoints, const std::string &terrainFile);
