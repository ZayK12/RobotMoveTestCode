    // ROBOTCODE.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "purePursuit.h"
#include "odometry.h"
#include "pathManager.h"
#include "config.h"
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include "PID.h"



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
    double lookAhead = 25; // Example look ahead distance
    pathManager activePath = pathManager(path); // Example path manager
    odometry roboOdom = odometry(config::INITIAL_POSITION, config::INITIAL_ORIENTATION, config::WHEEL_CIRCUMFERENCE, config::ENCODER_DISTANCE_LEFT, config::ENCODER_DISTANCE_RIGHT, config::ENCODER_DISTANCE_BACK); // Example odometry @todo 
    Pursuit roboPursuit = Pursuit(activePath.getPathPointer(), roboOdom.getPositionPointer(), lookAhead); // Example pure pursuit
    std::vector<double>* pursuitPointer = &roboPursuit.pursuitPoint;
	activePath.updatePathFromFile(); // Example path update from file   
	PID roboPID = PID(&roboPursuit.pursuitPoint, roboOdom.getPositionPointer(), roboOdom.getOrientationPointer(), config::MAX_FORWARD_SPEED, config::MAX_TURN_SPEED, config::FORWARD_KP, config::FORWARD_KI, config::FORWARD_KD, config::TURN_KP, config::TURN_KI, config::TURN_KD, config::ERROR_MARGIN, config::TURN_ERROR_MARGIN, config::ERROR_SUM_MARGIN, config::PATH_DIVISIONS, config::MOTOR_POWER_CHECK_AMOUNT); // Example PID controller
 
    roboPursuit.updatePursuitPoint(); // Example pursuit point calculation
    std::cout << "Pursuit Point: (" << (*pursuitPointer)[0] << ", " << (*pursuitPointer)[1] << ") Max Speed:" << (*pursuitPointer)[2] << std::endl; // Output the pursuit point


    // Calculate the duration
    // Print the duration

	roboOdom.updatePosition({ (*pursuitPointer)[0], (*pursuitPointer)[1] }); // Example position update

	//activePath.setActivePath(pathPlusOne); 
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
