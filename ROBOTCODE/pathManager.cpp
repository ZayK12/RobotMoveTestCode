#include "pathManager.h"

///@author Zayyaan K
///@date 5/5/25
pathManager::pathManager() : mainPath(), pathPointer(&mainPath) {}
pathManager::pathManager(std::vector<std::vector<double> >& initPath) : mainPath(initPath), pathPointer(&mainPath) {}

int pathManager::clearPath() {
    if (pathPointer) { //checks whether pathPointer holds an address
        std::vector<std::vector<double>> tempPath = {}; // Creates empty vector
        pathPointer->swap(tempPath); // Swaps vectors
    }
    return 0;
}
int pathManager::setActivePath(std::vector<std::vector<double>>& newPath) {
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
	std::vector<std::vector<double>> newPath;
    while (currLine < indexLine && std::getline(file, line)) { currLine++; } // Skip lines to last endData
    while (std::getline(file, line)) { // Spits out whole line
        currLine++;
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;
        while (std::getline(ss, value, ',')) { // Splits out individualized values with commas as delimiters
            if (value == "endData") { // 
                endDataFound = true;
                indexLine = currLine;
                break;
            }
            if (value.empty() || value.at(0) == '#' || value.at(0) == '\"' || value.at(0) == '{') continue;
            try { row.push_back(std::stod(value)); }
            catch (const std::invalid_argument&) {
                std::cerr << "Invalid value: " << value << " in file. Skipping." << std::endl;
            }
        }
        if (row.size() == 3) newPath.push_back(row);
        if (endDataFound) break; // Stop reading if "endData" is found
    }
	if (pathPointer && !newPath.empty()) { setActivePath(newPath); }
	return 0;
}
std::vector<std::vector<double>>** pathManager::getPathPointer() {
    return &pathPointer;
}