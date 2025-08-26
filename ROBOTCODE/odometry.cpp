#include "odometry.h"
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
odometry::odometry(std::vector<double> initPos, double initOrientation, double wheelC, double distanceLeft, double distanceRight, double distanceBack, pros::Rotation* _leftEncoder, pros::Rotation* _rightEncoder, pros::Rotation* _backEncoder,pros::v5::IMU* _leftIMU, pros::v5::IMU* _rightIMU) : MainPosition(initPos), orientation(initOrientation * M_PI / 180.0), wheelCircum(wheelC), disL(distanceLeft), disR(distanceRight), disB(distanceBack), directPositionPtr(&MainPosition), leftEncoder(_leftEncoder), rightEncoder(_rightEncoder),backEncoder(_backEncoder), leftIMU(_leftIMU), rightIMU(_rightIMU) {}

odometry::odometry(double wheelC, double distanceLeft, double distanceRight, double distanceBack,pros::Rotation* _leftEncoder, pros::Rotation* _rightEncoder, pros::Rotation* _backEncoder, pros::v5::IMU* _leftIMU, pros::v5::IMU* _rightIMU) : MainPosition({ 0.0,0.0 }), orientation(0), wheelCircum(wheelC), disL(distanceLeft), disR(distanceRight), disB(distanceBack), directPositionPtr(&MainPosition),leftEncoder(_leftEncoder), rightEncoder(_rightEncoder),backEncoder(_backEncoder), leftIMU(_leftIMU), rightIMU(_rightIMU){}



//Getters
std::vector<double>* odometry::getPositionPointer() { return directPositionPtr; }
double* odometry::getOrientationPointer() { return &orientationDeg; }
//Setters
void odometry::updatePosition(double newX, double newY) {
	MainPosition[0] = newX;
	MainPosition[1] = newY;
}
void odometry::updatePosition(std::vector<double> newPos) {
	MainPosition[0] = newPos[0];
	MainPosition[1] = newPos[1];
}
void odometry::updatePosition(std::vector<double> newPos, double newOrientation) {
	MainPosition[0] = newPos[0];
	MainPosition[1] = newPos[1];
	orientation = overflowCheck(newOrientation);
}
// Helper Functions
double odometry::overflowCheck(double deg) {
	// Normalize to [0, 360)
	return fmod(deg, 360.0);
}
double odometry::subRadians(double rad1, double rad2) {
    double delta = rad1 - rad2;
    // Normalize to [-pi, pi]
    return fmod(delta, 2 * M_PI);
}
/**
 * @note: This function is commented out because it may not work as intended, a potential fix is listed above.
double odometry::subRadians(double rad1, double rad2) {
	rad1 = tan((rad1 * pi / 180.0));
	rad2 = tan((rad2 * pi / 180.0));
	double delta = fabs(rad1 - rad2);
	return delta * pi / 180;
}
*/

void odometry::cartesianToPolar(std::vector<double>& coordSet) {
	coordSet[0] = sqrt(pow(coordSet[0], 2) + pow(coordSet[1], 2));
	coordSet[1] = atan2(coordSet[1], coordSet[0]);
}
void odometry::polarToCartesian(double& radius, double& theta) {
	double X = radius * cos(theta);
	double Y = radius * sin(theta);
	radius = X;
	theta = Y;
}

/// @note as of 8/20/25, I have marked this function for redo. 
void odometry::updateDistancesOld() {
	std::cout << "DEVELOPMENT FUNCTION IN USE: ODOMETRY::UPDATEDISTANCESOLD";
	double static leftInchLast=0, rightInchLast=0, backInchLast=0, headingLast = 0;

	double leftDeg = leftEncoder->get_position() * 100; // The *100 is because the rotation sensor returns centidegrees
	double rightDeg = rightEncoder->get_position() * 100;
	double backDeg = backEncoder->get_position() * 100; 
	double leftInch = leftDeg / 360 * wheelCircum;
	double rightInch = rightDeg / 360 * wheelCircum;
	double backInch = backDeg / 360 * wheelCircum;
	double leftInchDelta = leftInch - leftInchLast;
	double rightInchDelta = rightInch - rightInchLast;
	double backInchDelta = backInch - backInchLast;
	leftInchLast = leftInch;
	rightInchLast = rightInch;
	backInchLast = backInch;

	// update heading aswell
	/// @note Currently this only uses the IMU reading to calculate the heading.
	/// @todo Implement a method to calculate heading using the side encoders as well.
	double imuHeading = (leftIMU->get_heading() + rightIMU->get_heading()) / 2;
	double sideHeading = overflowCheck((leftInch - rightInch) / (disL + disR));

	double orientationDelta = (rightInchDelta - leftInchDelta)/ (disL + disR); // Difference since last update
	//orientation += orientationDelta; // Global orientation
	orientation = imuHeading * M_PI / 180.0; // Global orientation in radians
	calculateLocalOffset(backInchDelta, rightInchDelta, orientationDelta);
}
void odometry::updateDistances(){
	double static leftInchLast=0, rightInchLast=0, backInchLast=0, orientationLast = 0;

	double leftDeg = leftEncoder->get_position() * 100; // The *100 is because the rotation sensor returns centidegrees
	double rightDeg = rightEncoder->get_position() * 100;
	double backDeg = backEncoder->get_position() * 100; 

	double leftInch = leftDeg / 360 * wheelCircum;
	double rightInch = rightDeg / 360 * wheelCircum;
	double backInch = backDeg / 360 * wheelCircum;

	double leftInchDelta = leftInch - leftInchLast;
	double rightInchDelta = rightInch - rightInchLast;
	double backInchDelta = backInch - backInchLast;

	leftInchLast = leftInch;
	rightInchLast = rightInch;
	backInchLast = backInch;
	
	double imuHeading = (leftIMU->get_heading() + rightIMU->get_heading()) / 2;
	double sideHeading = orientation + overflowCheck((leftInchDelta + rightInchDelta) / (disL + disR));
	
	//orientation = sideHeading + imuHeading / 2;
	orientation = overflowCheck(imuHeading) * M_PI / 180.0; // Global orientation in deg
	double orientationDelta = orientation - orientationLast;
	orientationLast = orientation;

	calculateLocalOffset(backInchDelta, rightInchDelta, orientationDelta);
}
void odometry::calculateLocalOffset(double backInchDelta_, double rightInchDelta_, double orientationDelta_) {
	if (fabs(orientationDelta_) > TOLERANCE) {
		localOffset[0] = 2 * sin(orientationDelta_ / 2) * ((backInchDelta_ / orientationDelta_) + disB);
		localOffset[1] = 2 * sin(orientationDelta_ / 2) * ((rightInchDelta_ / orientationDelta_) + disR);
	}
	else {
		localOffset[0] = backInchDelta_;
		localOffset[1] = rightInchDelta_;
	}
	rotateToGlobalFrame();
}
void odometry::rotateToGlobalFrame() {
	cartesianToPolar(localOffset);
	localOffset[1] = localOffset[1] - orientation;
	polarToCartesian(localOffset[0], localOffset[1]);
	MainPosition[0] += localOffset[0]; //global pos
	MainPosition[1] += localOffset[1]; //global pos
	orientationDeg = orientation * 180.0 / M_PI; // Convert to degrees
	//std::cout << "X: " << MainPosition[0] << " Y: " << MainPosition[1] << " Orientation: " << orientation << std::endl;
}
