#include "Menu.h"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <vector>

namespace fs = std::filesystem;

// Is `path` an image we can load as a DEM? Compares the lowercased extension
// against the handful of formats readTerrain (stb_image) accepts.
static bool isImageFile(const fs::path &path)
{
    std::string ext = path.extension().string();
    for (char &c : ext)
        c = (char)std::tolower((unsigned char)c);

    return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".gif";
}

std::string selectTerrain(const std::string &dir)
{
    // Gather the candidate terrains, then append "Exit" as the final numbered entry.
    std::vector<std::string> files;
    for (const fs::directory_entry &entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file() && isImageFile(entry.path()))
            files.push_back(entry.path().filename().string());
    }
    files.push_back("Exit");

    size_t choice = 0;
    while (true) {
        std::cout << "Available terrains:" << std::endl;
        for (size_t i = 0; i < files.size(); i++)
            std::cout << "  " << i + 1 << ". " << files[i] << std::endl;
        std::cout << "Select a terrain by number: ";

        if (std::cin >> choice && choice >= 1 && choice <= files.size())
            break;

        // Bad input (non-numeric or out of range): clear the fail bit and discard the
        // rest of the line before re-prompting -- otherwise cin stays failed and the
        // loop spins without ever waiting for new input.
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid choice, try again." << std::endl;
    }

    // Discard the rest of the accepted line too: operator>> leaves the trailing
    // newline in the buffer, and the next cin reader (the tracker-count prompt
    // uses getline) must start from a clean line, not inherit ours.
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (choice == files.size()) {   // the trailing "Exit" entry
        std::cout << "Exiting..." << std::endl;
        exit(0);
    }

    return dir + files[choice - 1];
}
