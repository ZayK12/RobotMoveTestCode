#include "odometry.h"
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <string>


odometry::odometry(std::vector<double> initPos, double initOrientation, double wheelC, double distanceLeft, double distanceRight, double distanceBack) : position(initPos), orientation(initOrientation), wheelCircum(wheelC), disL(distanceLeft), disR(distanceRight), disB(distanceBack), pi(3.14159265359) {}
odometry::odometry(double wheelC, double distanceLeft, double distanceRight, double distanceBack) : position({ 0,0 }), orientation(0), wheelCircum(wheelC), disL(distanceLeft), disR(distanceRight), disB(distanceBack), pi(3.14159265359) {}

void odometry::odometry::updatePosition(double newX, double newY) {
	position[0] = newX;
	position[1] = newY;
}

double odometry::overflowCheck(double deg) {
	if (deg == 360) {
		return 0;
	}
	return deg;
}

double odometry::subRadians(double rad1, double rad2) {
	rad1 = tan((rad1 * pi / 180.0));
	rad2 = tan((rad2 * pi / 180.0));
	double delta = fabs(rad1 - rad2);
	return delta * pi / 180;
}
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
void odometry::updateDistances() {
	double static leftInchLast, rightInchLast, backInchLast, headingLast = 0;

	//leftDeg = leftEncoder.get_value(); add library
	//rightDeg = rightEncoder.get_value(); add library
	//backDeg = backEncoder.get_value(); add library
	double leftDeg = 0;
	double rightDeg = 0;
	double backDeg = 0;
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
	double imuHeading = 0; // needs library 
	double sideHeading = (leftInch - rightInch) / (disL + disR);
	double headingDelta = subRadians(headingLast, imuHeading);
	headingLast = 0; // @todo needs library
	orientation = (headingLast + headingDelta) / 2; // Global orientation
	calculateLocalOffset(backInchDelta, rightInchDelta, headingDelta);
}
void odometry::calculateLocalOffset(double backInchDelta_, double rightInchDelta_, double headingDelta_) {

	localOffset = { backInchDelta_, rightInchDelta_ };
	if (headingDelta_ != 0) {
		localOffset[0] = 2 * sin(headingDelta_ / 2) * ((backInchDelta_ / headingDelta_) + disB);
		localOffset[1] = 2 * sin(headingDelta_ / 2) * ((rightInchDelta_ / headingDelta_) + disR);
	}
	else {
		localOffset[0] = backInchDelta_;
		localOffset[1] = rightInchDelta_;
	}
}
void odometry::rotateToGlobalFrame() {
	cartesianToPolar(localOffset);
	localOffset[1] = localOffset[1] - orientation;
	polarToCartesian(localOffset[0], localOffset[1]);
	position[0] += localOffset[0]; //global pos
	position[1] += localOffset[1]; //global pos
}

