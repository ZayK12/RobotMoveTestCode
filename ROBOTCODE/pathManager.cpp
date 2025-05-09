#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <string>
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
        pathPointer->clear();
        *pathPointer = newPath;
    }
    return 0; //Deletes old vectors
}
std::vector<std::vector<double>>** pathManager::getPathPointer() {
    return &pathPointer;
}