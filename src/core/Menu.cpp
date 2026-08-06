#include "Menu.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <optional>
#include <system_error>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <io.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace fs = std::filesystem;

// Drop keystrokes typed before a prompt existed (e.g. into the terminal while
// the window was minimized) -- otherwise the next getline answers the prompt
// with them. TTY-only: flushing a redirected stdin would eat legitimate
// scripted input. No portable call for this, hence the two OS branches.
// Failure warns instead of throwing: prompts run inside GLFW callbacks, where
// an exception would unwind through C code, and a failed flush only costs a
// stale line -- which the parse below already survives.
#ifdef _WIN32
static void discardPendingInput()
{
    if (!_isatty(_fileno(stdin)))
        return;
    if (!FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE)))
        std::cerr << "Warning: could not flush pending console input" << std::endl;
}
#else
static void discardPendingInput()
{
    if (!isatty(STDIN_FILENO))
        return;
    if (tcflush(STDIN_FILENO, TCIFLUSH) != 0)
        std::cerr << "Warning: could not flush pending terminal input" << std::endl;
}
#endif

// One line -> number in [min, max]; nullopt on anything else. The sign test
// comes before the cast: a negative would wrap to a huge size_t and pass.
static std::optional<size_t> parseCount(const std::string &line, size_t min, size_t max)
{
    try {
        const int n = std::stoi(line);
        if (n >= 0 && (size_t)n >= min && (size_t)n <= max)
            return (size_t)n;
    } catch (...) {}   // stoi: not a number at all
    return std::nullopt;
}

// Is `path` an image we can load as a DEM? Lowercased extension against the
// raster formats cv::imread reads on every platform.
static bool isImageFile(const fs::path &path)
{
    std::string ext = path.extension().string();
    for (char &c : ext)
        c = (char)std::tolower((unsigned char)c);

    return ext == ".jpg" || ext == ".jpeg" || ext == ".png";
}

std::optional<std::string> selectTerrain(const std::string &dir)
{
    // The error_code overload keeps a missing directory from throwing -- the
    // asset paths are relative, so running from outside the repo root lands
    // here, and it should read as a usage error, not a crash.
    std::error_code ec;
    fs::directory_iterator it(dir, ec);
    if (ec) {
        std::cerr << "Cannot read terrain directory '" << dir << "' ("
                  << ec.message() << ") -- run from the repository root." << std::endl;
        return std::nullopt;
    }

    std::vector<std::string> files;
    for (const fs::directory_entry &entry : it) {
        if (entry.is_regular_file() && isImageFile(entry.path()))
            files.push_back(entry.path().filename().string());
    }
    // Directory iteration order is unspecified; sort so the numbering is
    // stable across runs and platforms.
    std::sort(files.begin(), files.end());
    files.push_back("Exit");   // final numbered entry

    discardPendingInput();
    size_t choice = 0;
    while (true) {
        std::cout << "Available terrains:" << std::endl;
        for (size_t i = 0; i < files.size(); i++)
            std::cout << "  " << i + 1 << ". " << files[i] << std::endl;
        std::cout << "Select a terrain by number: ";

        std::string line;
        if (!std::getline(std::cin, line)) {   // stdin closed: treat as Exit
            std::cout << "Exiting..." << std::endl;
            return std::nullopt;
        }
        if (const std::optional<size_t> n = parseCount(line, 1, files.size())) {
            choice = *n;
            break;
        }
        std::cout << "Invalid choice, try again." << std::endl;
    }

    if (choice == files.size()) {   // the trailing "Exit" entry
        std::cout << "Exiting..." << std::endl;
        return std::nullopt;
    }

    return dir + files[choice - 1];
}

// Line-based like every cin reader here: getline consumes whole lines, so no
// reader leaves a stray newline behind for the next one.
size_t promptCount(const std::string &label, size_t max, size_t fallback)
{
    discardPendingInput();
    std::cout << label << " (1-" << max << ", Enter = " << fallback << "): ";

    std::string line;
    if (!std::getline(std::cin, line) || line.empty())
        return fallback;
    if (const std::optional<size_t> n = parseCount(line, 1, max))
        return *n;

    std::cout << "Invalid count -- using " << fallback << std::endl;
    return fallback;
}

std::optional<size_t> promptOptionalCount(const std::string &label, size_t min, size_t max)
{
    discardPendingInput();
    std::cout << label << " (" << min << "-" << max << ", Enter = auto): ";

    std::string line;
    if (!std::getline(std::cin, line) || line.empty())
        return std::nullopt;
    if (const std::optional<size_t> n = parseCount(line, min, max))
        return n;

    std::cout << "Invalid count -- using auto" << std::endl;
    return std::nullopt;
}
