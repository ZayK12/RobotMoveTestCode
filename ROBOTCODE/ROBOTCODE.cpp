// ROBOTCODE.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "purePursuit.h"
#include "odometry.h"
#include "pathManager.h"
#include "config.h"
int main() {
    std::vector<std::vector<double> > path = {
        {3,7, 100},
        {4,8, 50},
        {5,9, 100},
        {6,10, 80},
        {7,11, 20},
        {8,12, 82},
        {9,13, 96},
        {10,14, 45},
        {11,15, 75},
        {12,16, 75},
        {13,17, 75},
        {14,18, 75}
    }; // Example path
    double lookAhead = 8.4; // Example look ahead distance
    pathManager activePath = pathManager(path); // Example path manager
    odometry roboOdom = odometry(config::INITIAL_POSITION, config::INITIAL_ORIENTATION, config::WHEEL_CIRCUMFERENCE, config::ENCODER_DISTANCE_LEFT, config::ENCODER_DISTANCE_RIGHT, config::ENCODER_DISTANCE_BACK); // Example odometry @todo 
    Pursuit roboPursuit = Pursuit(activePath.getPathPointer(), roboOdom.getPositionPointer(), lookAhead); // Example pure pursuit
    std::vector<double>* pursuitPointer = &roboPursuit.pursuitPoint;
    
	
    roboPursuit.updatePursuitPoint(); // Example pursuit point calculation
    std::cout << "Pursuit Point: (" << (*pursuitPointer)[0] << ", " << (*pursuitPointer)[1] << ") Max Speed:" << (*pursuitPointer)[2] << std::endl; // Output the pursuit point

    std::vector<std::vector<double> > pathPlusOne = {
        {4,7, 100},
        {5,8, 100},
        {6,9, 100},
        {7,10, 100},
        {8,11, 100},
        {9,12, 100},
        {10,13, 100},
        {11,14, 90},
        {12,15, 80},
        {13,16, 70},
        {14,17, 60},
        {15,18, 50}
    }; // Example path

	activePath.setActivePath(pathPlusOne); //breaks it for some reason
    roboPursuit.updatePursuitPoint(); // Example pursuit point calculation
    std::cout << "Pursuit Point: (" << (*pursuitPointer)[0] << ", " << (*pursuitPointer)[1] << ") Max Speed:" << (*pursuitPointer)[2]  << std::endl; // Output the pursuit point
    return 0;
}


// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
