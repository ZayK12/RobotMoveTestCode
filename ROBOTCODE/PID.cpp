#include "PID.h"

PID::PID(std::vector<double>* Point, std::vector<double>* position, double* heading, double _maxDSpeed, double _maxTSpeed, double _DKp, double _DKi, double _DKd, double _TKp, double _TKi, double _TKd) :
	targetGlobal(Point), currentPosition(position), currentHeading(heading), maxDSpeed(_maxDSpeed), maxTurnSpeed(_maxTSpeed), DKp(_DKp), DKi(_DKi), DKd(_DKd), TKp(_TKp), TKi(_TKi), TKd(_TKd),
	errorDistance(0.0), errorTurning(0.0), integral(0.0), derivative(0.0), previousError(0.0), turnIntegral(0.0), turnDerivative(0.0), turnPreviousError(0.0),
	errorSumMargin(1.0), turnErrorMargin(1.0) {}

PID::PID(std::vector<double>* Point, std::vector<double>* position, double* heading, double _maxDSpeed, double _maxTSpeed, double _DKp, double _DKi, double _DKd, double _TKp, double _TKi, double _TKd, double ERM, double TERM, double ESM, int pathDiv, double MPCA) :
	targetGlobal(Point), currentPosition(position), currentHeading(heading), maxDSpeed(_maxDSpeed), maxTurnSpeed(_maxTSpeed), DKp(_DKp), DKi(_DKi), DKd(_DKd), TKp(_TKp), TKi(_TKi), TKd(_TKd),
	errorDistance(0.0), errorTurning(0.0), integral(0.0), derivative(0.0), previousError(0.0), turnIntegral(0.0), turnDerivative(0.0), turnPreviousError(0.0),
	errorSumMargin(ERM), turnErrorMargin(TERM), errorMargin(ESM), pathDivamt(pathDiv), motorPowerCheckAmt(MPCA) {
}
bool PID::errorSumUpdate(double inputNumber) {
	static std::vector<double> numberList(10, 0.0); // List to store the last 10 numbers
	//errorSum variable stores the sum of the last 10 numbers
	static double errorSum = 0;
	// index for the circular buffer 
	static int index = 0;
	// updates the errorSum variable by subtracting the soon-to-be-removed number
	errorSum -= numberList[index];
	// replaces the older value with the input number
	numberList[index] = inputNumber;
	// Adds new number to the sum
	errorSum += inputNumber;
	// Increments the index and uses modulus to wrap around. Ex: 11 % 10 = 1
	index = (index + 1) % 10; // Wraps around to 0 after reaching 9
	if (fabs((errorSum / 10) - inputNumber) < errorSumMargin) {
		return true; // Returns true if the average of the last 10 numbers is less than the error margin
	}
	else {
		return false; // Returns false if the average of the last 10 numbers is greater than the error margin
	}
}
std::vector<double> PID::cartesianToClock(const std::vector<double>& localVector) {
	std::vector<double> clockTarget(2);
	clockTarget[0] = sqrt((localVector[0] * localVector[0]) + (localVector[1] * localVector[1]));
	clockTarget[1] = atan2(localVector[1], localVector[0]);
	return clockTarget;
}
double PID::deltaDegrees(double a, double b) {
	double delta = fmod(b - a + 180.0, 360.0) - 180.0;
	return delta;
}
void PID::pidUpdate() {
	
	std::vector<double> targetLocal = { (*targetGlobal)[0] - (*currentPosition)[0], (*targetGlobal)[1] - (*currentPosition)[1] }; // Gets target within local coordinate scope
	// Takes the local target and converts it to clock coordinates
	std::vector<double> clockTarget = cartesianToClock(targetLocal);
	// Sets the error (proportinate) amounts
	errorDistance = clockTarget[0];
	errorTurning = deltaDegrees(*currentHeading, clockTarget[1]);
	//Will use back of robot to move if it is closer
	if (fabs(errorDistance) > 90) {
		errorDistance = -errorDistance;
		errorTurning = deltaDegrees(180, errorTurning);
	}
	// If robot is close enough to end point, stop
	if (fabs(errorDistance) < errorMargin) {
		errorDistance = 0;
		turnIntegral = 0;
	}
	//Updates ID variables
	integral += errorDistance;
	double deriviative = errorDistance - previousError;
	previousError = errorDistance;
	//Updates the turning ID variables
	turnIntegral += errorTurning;
	double turnDerivative = errorTurning - turnPreviousError;
	turnPreviousError = errorTurning;

	// Calculates the power amounts for distance and turning
	double maxSpeed = (*targetGlobal)[2];
	double powerDistance = DKp * errorDistance + DKi * integral + DKd * derivative;
	if (powerDistance > maxSpeed) {
		powerDistance = maxSpeed;
	}
	else if (powerDistance < -maxSpeed) {
		powerDistance = -maxSpeed;
	}
	double powerTurning = TKp * errorTurning + TKi * turnIntegral + TKd * turnDerivative;
	if (powerTurning > maxTurnSpeed) {
		powerTurning = maxTurnSpeed;
	}
	else if (powerTurning < -maxTurnSpeed) {
		powerTurning = -maxTurnSpeed;
	}
	if (errorSumUpdate(errorDistance)) {
		std::cout << "Below Margin" << std::endl;

	}
	else {
		std::cout << "Power Distance left: " << powerDistance + powerTurning << "Power Distance Right:" << powerDistance - powerTurning << std::endl;
	}


}
