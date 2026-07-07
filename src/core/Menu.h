#pragma once

#include <optional>
#include <string>

// Terminal terrain picker: lists the image files in `dir`, prompts the user to
// choose one by number, and returns its full path -- or nullopt if the user
// chose the "Exit" entry, letting the caller unwind normally (destructors run).
// Blocking std::cin prompt: runs between sessions, never inside the render loop.
std::optional<std::string> selectTerrain(const std::string &dir);

// Terminal count prompt: "<label> (1-max, Enter = fallback): ". One line read;
// anything that isn't a number in [1, max] falls back (plain Enter silently,
// bad input with a message). Shared by the tracker- and feature-count prompts.
size_t promptCount(const std::string &label, size_t max, size_t fallback);
