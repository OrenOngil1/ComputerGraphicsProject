#pragma once

#include <optional>
#include <string>

// Terminal terrain picker: lists the image files in `dir`, prompts the user to
// choose one by number, and returns its full path -- or nullopt if the user
// chose the "Exit" entry, letting the caller unwind normally (destructors run).
// Blocking std::cin prompt: runs between sessions, never inside the render loop.
std::optional<std::string> selectTerrain(const std::string &dir);
