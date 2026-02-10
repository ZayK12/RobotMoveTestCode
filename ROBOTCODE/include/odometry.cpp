#include "odometry.h"
#include "config.h"
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include "pros/adi.hpp"
#include "pros/imu.hpp"

//Contructor
//2 IMUs
odometry::odometry(std::vector<float> initPos, float initOrientation, float wheelC, float distanceLeft, float distanceRight, float distanceBack, pros::Rotation* _rightEncoder, pros::Rotation* _backEncoder,pros::v5::IMU* _leftIMU, pros::v5::IMU* _rightIMU)
	: MainPosition(initPos), orientation(initOrientation * M_PI / 180.0), wheelCircum(wheelC), disL(distanceLeft), disR(distanceRight), disB(distanceBack), directPositionPtr(&MainPosition), rightEncoder(_rightEncoder), backEncoder(_backEncoder), leftIMU(_leftIMU), rightIMU(_rightIMU), localOffset({0.0, 0.0}) {}

odometry::odometry(float wheelC, float distanceLeft, float distanceRight, float distanceBack, pros::Rotation* _rightEncoder, pros::Rotation* _backEncoder, pros::v5::IMU* _leftIMU, pros::v5::IMU* _rightIMU)
	: MainPosition({0.0, 0.0}), orientation(0), wheelCircum(wheelC), disL(distanceLeft), disR(distanceRight), disB(distanceBack), directPositionPtr(&MainPosition), rightEncoder(_rightEncoder), backEncoder(_backEncoder), leftIMU(_leftIMU), rightIMU(_rightIMU), localOffset({0.0, 0.0}) {}



//Getters
std::vector<float>* odometry::getPositionPointer() { return directPositionPtr; }
float* odometry::getOrientationPointer() { return &orientation; }
//Setters
void odometry::updatePosition(float newX, float newY) {
	MainPosition[0] = newX;
	MainPosition[1] = newY;
}
void odometry::updatePosition(std::vector<float> newPos) {
	MainPosition[0] = newPos[0];
	MainPosition[1] = newPos[1];
}
void odometry::updatePosition(std::vector<float> newPos, float newOrientation) {
	MainPosition[0] = newPos[0];
	MainPosition[1] = newPos[1];
	orientation = overflowCheck(newOrientation);
}
// Helper Functions
float odometry::overflowCheck(float deg) {
	// Normalize to [0, 360)
	float v = fmod(deg, 360.0);
	if (v < 0) v += 360.0;
	if (fabsf(v - 360.0) < TOLERANCE) v = 0.0; // Handle edge case where value is very close to 360
	return v;
}
float odometry::subRadians(float rad1, float rad2) {
	float delta = rad1 - rad2;
	// Normalize to [-pi, pi]
	while (delta > M_PI) delta -= M_TWOPI;
	while (delta <= -M_PI) delta += M_TWOPI; 
	return delta;
}
float odometry::subDegrees(float deg1, float deg2) {
	float delta = deg1 - deg2;
	// Normalize to [-180, 180]
	delta = fmodf(delta + 180.0f, 360.0f);
	if (delta < 0) delta += 360.0f;
	delta -= 180.0f;
	return delta;
}
/**
 * @note: This function is commented out because it may not work as intended, a potential fix is listed above.
float odometry::subRadians(float rad1, float rad2) {
	rad1 = tan((rad1 * pi / 180.0));
	rad2 = tan((rad2 * pi / 180.0));
	float delta = fabsf(rad1 - rad2);
	return delta * pi / 180;
}
*/

void odometry::cartesianToPolar(std::vector<float>& coordSet) {
	// Expect coordSet to contain {x, y} in that order. Compute radius and theta
	// using original x and y values so sign information is preserved.
	float x = coordSet.size() > 0 ? coordSet[0] : 0.0;
	float y = coordSet.size() > 1 ? coordSet[1] : 0.0;
	float radius = sqrtf(x * x + y * y);
	float theta = atan2f(y, x);
	if (coordSet.size() < 2) coordSet.resize(2);
	coordSet[0] = radius;
	coordSet[1] = theta;
}
void odometry::polarToCartesian(float& radius, float& theta) {
	float R = radius;
	float T = theta;
	float X = R * cosf(T);
	float Y = R * sinf(T);
	radius = X;
	theta = Y;
}

void odometry::updateDistances(){
	float static rightInchLast=0, backInchLast=0, orientationLast = 0;
	static bool firstCall = true; // Skip first update to avoid large initial orientation delta caused by sensor startup/tare

	// Get sensor readings with validation
	float rightDeg = rightEncoder->get_position() / 100; // The *100 is because the rotation sensor returns centidegrees
	float backDeg = backEncoder->get_position() / 100;

	// Convert encoder readings directly to inches 
	float rightInch = rightDeg / 360.0 * wheelCircum;
	float backInch = backDeg / 360.0 * wheelCircum;
	
	



	//Our math expects orientation to be counter-clockwise positive. If IMU increases clockwise,
	const float IMU_SIGN = -1.0;
	// Use the circular average of both IMUs and apply the sign consistently.
	float imuHeading = overflowCheck(leftIMU->get_heading());
	imuHeading = IMU_SIGN * imuHeading; // apply sign once, consistently

	// Skip first update to avoid large initial orientation delta
	if (firstCall) {
		rightInchLast = rightInch;
		backInchLast = backInch;
		orientationLast = imuHeading; // already signed and normalized
		firstCall = false;
		// Do not update position on the first reading
		return;
	}

	float rightInchDelta = rightInch - rightInchLast;
	float backInchDelta = backInch - backInchLast;

	//Prevent large jumps
	const float ENCODER_DELTA_MAX = 5.0; // inches per cycle
	if (fabsf(rightInchDelta) > ENCODER_DELTA_MAX || fabsf(backInchDelta) > ENCODER_DELTA_MAX) {
		// Encoder reading is suspect; skip position update but keep tracking the current position
		orientationLast = imuHeading;
		return;
	}

	rightInchLast = rightInch;
	backInchLast = backInch;

	// Handle heading and orientation updates (degrees, signed)
	orientation = imuHeading;
	// Compute and log raw vs normalized orientation deltas for debugging
	float orientationDelta = subDegrees(orientation, orientationLast); // Properly handle wrap-around
	
	
	
	
	orientationLast = orientation;

	calculateLocalOffset(backInchDelta, rightInchDelta, orientationDelta);
}
void odometry::calculateLocalOffset(float backInchDelta_, float rightInchDelta_, float orientationDelta_) {
	// Note: right encoder measures forward displacement (x), back encoder measures lateral displacement (y).
	if (fabsf(orientationDelta_) > TOLERANCE) {
		float orientationDeltaRAD = orientationDelta_ * M_PI / 180.0; // Convert to radians
		// x (forward)
		localOffset[0] = 2.0 * sinf(orientationDeltaRAD / 2.0) * ((rightInchDelta_ / orientationDeltaRAD) + disR);
		// y (lateral) 
		localOffset[1] = 2.0 * sinf(orientationDeltaRAD / 2.0) * ((backInchDelta_ / orientationDeltaRAD) + disB);
		/*
		if(config::debugOdom){
			//printf("\nDis (in) - R: %.3f, L: %.3f \n Orientation %.3f\n ", rightInchDelta_, backInchDelta_, orientationDelta_);
			
			std::stringstream msgDistances;
			msgDistances << "\n Distance (inches) - Right: " << rightInch << " Back: " << backInch << std::endl;
			std::cout << msgDistances.str();
		}*/
	}
	else {
		return;
	}
	if (localOffset.size() < 2) localOffset.resize(2);
	rotateToGlobalFrame();

}

void odometry::rotateToGlobalFrame() {
	if (localOffset.size() < 2) {
		printf("RGF Small");
		return;
		/*
		std::stringstream msgRotateToFrame;
		msgRotateToFrame << "ERROR: rotateToGlobalFrame called with insufficient localOffset size" << std::endl;
		std::cout << msgRotateToFrame.str();
		*/
	}
	//Rotation Matrix (https://en.wikipedia.org/wiki/Rotation_matrix) proof and explanation on the notebook
	float orientationRad = orientation * M_PI / 180.0; // Convert orientation from degrees to radians
	float offsetX = localOffset[0] * cosf(orientationRad) - localOffset[1] * sinf(orientationRad); // xcos(theta) - ysin(theta)
	float offsetY = localOffset[0] * sinf(orientationRad) + localOffset[1] * cosf(orientationRad); // xsin(theta) + ycos(theta)
	
	MainPosition[0] += offsetX; //global pos
	MainPosition[1] += offsetY; //global pos
	if(config::debugOdom){
		printf("LO: (%.3f, %.3f), RGF: (%.3f, %.3f), UGP: (%.3f, %.3f)\n",localOffset[0], localOffset[1], offsetX, offsetY, MainPosition[0], MainPosition[1]);
		
		//std::stringstream msgFinal;
		//msgFinal << "localOffset X=" << localOffset[0] << " Y=" << localOffset[1] << " orientation=" << orientation << "\n" << "Rotated to Global Frame - X=" << localCopy[0] << " Y=" << localCopy[1] << "\n" << "Updated Global Position - X=" << MainPosition[0] << " Y=" << MainPosition[1] << std::endl;
		//std::cout << msgFinal.str();
	}
}
