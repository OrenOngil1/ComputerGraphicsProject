#include "Menu.h"

#include <vector>
#include <filesystem>
#include <regex>
#include <iostream>

std::vector<std::string> list;

bool isFileImage(const std::filesystem::path &filePath)
{
    const std::regex imageExtensions("[^\\s]+(.*?)\\.(jpg|jpeg|png|gif|JPG|JPEG|PNG|GIF)$");

    std::string extension = filePath.string();
    
    return std::regex_match(extension, imageExtensions);
}

void getImageFiles(const std::string &dir)
{
    for (const auto &entry : std::filesystem::directory_iterator(dir)) {
        if(entry.exists() && isFileImage(entry.path())) {
            list.push_back(entry.path().filename().string());
        }
    }

    list.push_back("Exit");
}

void printMenu()
{
    std::cout << "Available Images:" << std::endl;
    for (size_t i = 0; i < list.size(); i++) {
        std::cout << i + 1 << ". " << list[i] << std::endl;
    }
}

std::string imagePath(const std::string &dir)
{
    size_t choice;
    getImageFiles(dir);

    do {
        printMenu();
        std::cout << "Select an image by number (or Exit): ";
        std::cin >> choice;
    } while(std::cin.fail() || choice < 1 || choice > list.size());

    if(choice == list.size()) {
        std::cout << "Exiting..." << std::endl;
        exit(0);
    }

    std::string ret = dir + list[choice - 1];

    list.clear();

    return ret;
}
