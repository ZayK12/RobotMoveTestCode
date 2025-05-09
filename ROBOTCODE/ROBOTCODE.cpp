// ROBOTCODE.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "purePursuit.h"
#include "odometry.h"
#include "pathManager.h"
int main() {
    std::vector<std::vector<double> > path = {
        {3,7},
        {4,8},
        {5,9},
        {6,10},
        {7,11},
        {8,12},
        {9,13},
        {10,14},
        {11,15},
        {12,16},
        {13,17},
        {14,18}
    }; // Example path
	std::vector<double> Initialposition = { 2, 5 }; // Initial Position, DO NOT USE AFTER DECLARING WITHIN ODOMETRY CLASS
    double lookAhead = 10.0; // Example look ahead distance
    double wheelCircumference = 0.5; // Example wheel circumference
    pathManager activePath = pathManager(path); // Example path manager
    odometry roboOdom = odometry(Initialposition, 0, wheelCircumference, 2, 2, 2); // Example odometry @todo 
    Pursuit roboPursuit = Pursuit(activePath.getPathPointer(), roboOdom.getPositionPointer(), lookAhead); // Example pure pursuit

    
	
    std::vector<double> pursuitPoint = roboPursuit.updatePursuitPoint(); // Example pursuit point calculation
    std::cout << "Pursuit Point: (" << pursuitPoint[0] << ", " << pursuitPoint[1] << ")" << std::endl; // Output the pursuit point

    std::vector<std::vector<double> > pathPlusOne = {
        {4,7},
        {5,8},
        {6,9},
        {7,10},
        {8,11},
        {9,12},
        {10,13},
        {11,14},
        {12,15},
        {13,16},
        {14,17},
        {15,18}
    }; // Example path

	activePath.setActivePath(pathPlusOne); //breaks it for some reason
    pursuitPoint = roboPursuit.updatePursuitPoint(); // Example pursuit point calculation
    std::cout << "Pursuit Point: (" << pursuitPoint[0] << ", " << pursuitPoint[1] << ")" << std::endl; // Output the pursuit point
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
