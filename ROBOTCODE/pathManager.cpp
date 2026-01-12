#include "pathManager.h"

///@author Zayyaan K
///@date 5/5/25
pathManager::pathManager() : mainPath(), pathPointer(&mainPath) {}
pathManager::pathManager(std::vector<std::vector<float> >& initPath) : mainPath(initPath), pathPointer(&mainPath) {}

int pathManager::clearPath() {
    if (pathPointer) { //checks whether pathPointer holds an address
        std::vector<std::vector<float>> tempPath = {}; // Creates empty vector
        pathPointer->swap(tempPath); // Swaps vectors
    }
    return 0;
}
int pathManager::setActivePath(std::vector<std::vector<float>>& newPath) {
    if (pathPointer && !newPath.empty()) {
        clearPath();
        pathPointer->swap(newPath);
    }
    return 0; //Deletes old vectors
}
int pathManager::updatePathFromFile() {
    int static indexLine = 0;
    std::ifstream file(config::filePath);
    std::string line;
    int currLine = 0;
    bool endDataFound = false;
    std::vector<std::vector<float>> newPath;
    float currentFourthValue = 0.0; // Default 4th value
    bool hasFourthValue = false;
    while (currLine < indexLine && std::getline(file, line)) { currLine++; } // Skip lines to last endData
    while (std::getline(file, line)) {
        currLine++;
        std::stringstream ss(line);
        std::string value;
        std::vector<float> row;
        while (std::getline(ss, value, ',')) {
            if (value == "endData") {
                endDataFound = true;
                indexLine = currLine;
                break;
            }
            if (value.empty() || value.at(0) == '#' || value.at(0) == '"' || value.at(0) == '{') continue;
            try { row.push_back(std::stod(value)); }
            catch (const std::invalid_argument&) {
                std::cerr << "Invalid value: " << value << " in file. Skipping." << std::endl;
            }
        }

        if (row.size() == 4) {
            currentFourthValue = row[3];
            hasFourthValue = true;
            row.pop_back(); // Remove the 4th value for storage, will append below
        }

        if (row.size() == 3) {
            if (hasFourthValue) row.push_back(currentFourthValue);
            newPath.push_back(row);
        }

        if (endDataFound) break;
    }
    if (pathPointer && !newPath.empty()) { setActivePath(newPath); }
    return 0;
}
std::vector<std::vector<float>>** pathManager::getPathPointer() {
    return &pathPointer;
} //new