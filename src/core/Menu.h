#pragma once

#include <string>

// Terminal terrain picker: lists the image files in `dir`, prompts the user to
// choose one by number, and returns its full path. An "Exit" entry terminates the
// program. Runs once at startup, before the GL window opens (a blocking std::cin
// prompt and a live render loop don't mix).
std::string selectTerrain(const std::string &dir);
