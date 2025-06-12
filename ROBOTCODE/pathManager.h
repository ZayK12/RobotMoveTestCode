///@author Zayyaan K
///@date 5/5/25
#ifndef pathManager_H
#define pathManager_H
#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <thread>
#include <fstream>
#include <sstream>
#include "config.h"

/// @todo add JerryLum's hot-cold assets and implement an auto changer.
/// @todo add a parser for text files to grab position.
class pathManager {
public:

    /**
     * @author Zayyaan K
     * @date 5/5/25
     * @brief path manager constructor w/ an already active path
     * @param initPath the initial path you want it set to
    */
    pathManager(std::vector<std::vector<double> >& initPath);

    /**
     * @author Zayyaan K
     * @date 5/5/25
     * @brief path manager initializing w/ an empty path
    */
    pathManager();

    /**
     * @author Zayyaan K
     * @date 5/1/25
     * @version 2
     * @details version 1, an input, but since it always intended to be the direct pointer to the active path, I just changed it to be the class' direct pointer.
     * @brief sets the active path to a blank slate and deletes the values from the original
    */
    int clearPath();

    /**
     * @author Zayyaan K
     * @date 5/1/25
     * @version 2
     * @brief Swaps active path values with another path and reassigns the direct pointer
     * @details version 1, had 2 inputs, but since one input always intended to be the direct pointer to the active path, I just changed it to be the class' direct pointer.
     * @param newPath The path you want set to active path.
    */
    int setActivePath(std::vector<std::vector<double> >& newPath);

    /**
     * @author Zayyaan K
     * @date 5/5/25
     * @version 1
     * @brief Assign all path pointers using this function.
     * @return The ram value to a direct pointer to the path vector.
    */
    std::vector<std::vector<double> >** getPathPointer();

	/**
	 * @author Zayyaan K
	 * @date 5/15/25
	 * @brief updates the active path using the jerryLum path file supports multiple paths in one file. Seperated by "endData"
	 * @return 0
	*/
    int updatePathFromFile();

    std::vector<std::vector<double> > mainPath;
private:
    std::vector<std::vector<double> >* pathPointer; //Direct Pointer to the mainPath

};





#endif