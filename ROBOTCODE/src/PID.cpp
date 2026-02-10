#include "PID.h"
#include "config.h"
PID::PID(std::vector<float>* Point, std::vector<float>* position, float* heading, float& _maxDSpeed, float& _maxTSpeed, float& _DKp, float& _DKi, float& _DKd, float& _TKp, float& _TKi, float& _TKd, pros::MotorGroup* _leftMotors, pros::MotorGroup* _rightMotors) :
	targetGlobal(Point), currentPosition(position), currentHeading(heading), maxDSpeed(_maxDSpeed), maxTurnSpeed(_maxTSpeed), DKp(_DKp), DKi(_DKi), DKd(_DKd), TKp(_TKp), TKi(_TKi), TKd(_TKd),
	errorDistance(0.0), errorTurning(0.0), integral(0.0), derivative(0.0), previousError(0.0), turnIntegral(0.0), turnDerivative(0.0), turnPreviousError(0.0),
	errorSumMargin(1.0), turnErrorMargin(1.0), leftside(_leftMotors), rightside(_rightMotors) {}

PID::PID(std::vector<float>* Point, std::vector<float>* position, float* heading, float& _maxDSpeed, float& _maxTSpeed, float& _DKp, float& _DKi, float& _DKd, float& _TKp, float& _TKi, float& _TKd, float& ERM, float& TERM, float& ESM, int& pathDiv, float& MPCA, pros::MotorGroup* _leftMotors, pros::MotorGroup* _rightMotors) :
	targetGlobal(Point), currentPosition(position), currentHeading(heading), maxDSpeed(_maxDSpeed), maxTurnSpeed(_maxTSpeed),DKp(_DKp), DKi(_DKi), DKd(_DKd), TKp(_TKp), TKi(_TKi), TKd(_TKd),
	errorDistance(0.0), errorTurning(0.0), integral(0.0), derivative(0.0), previousError(0.0), turnIntegral(0.0), turnDerivative(0.0), turnPreviousError(0.0), 
	errorSumMargin(ERM), turnErrorMargin(TERM), errorMargin(ESM), pathDivamt(pathDiv), motorPowerCheckAmt(MPCA), leftside(_leftMotors), rightside(_rightMotors) {}

void PID::setLateralConstants(float newDKp, float newDKi, float newDKd){
	DKp = newDKp;
	DKi = newDKi;
	DKd = newDKd;
}
void PID::setTurningConstants(float newTKp, float newTKi, float newTKd){
	TKp = newTKp;
	TKi = newTKi;
	TKd = newTKd;
}



bool PID::errorSumUpdate(float inputNumber) {
	static std::vector<float> numberList(10, 99999); // List to store the last 10 numbers 99999 is used to prevent early triggering
	//errorSum variable stores the sum of the last 10 numbers
	static float errorSum = 0;
	// index for the circular buffer 
	static int index = 0;
	// updates the errorSum variable by subtracting the soon-to-be-removed number
	errorSum -= numberList[index];
	// replaces the older value with the input number
	numberList[index] = inputNumber;
	// Adds new number to the sum
	errorSum += inputNumber;
	printf("errorSum: %.3f\n", errorSum);
	// Increments the index and uses modulus to wrap around. Ex: 11 % 10 = 1
	index = (index + 1) % 10; // Wraps around to 0 after reaching 9
	if (fabsf((errorSum / 10) - inputNumber) < errorSumMargin) {
		//DKp = 0;
		//TKp = 0;
		return true; // Returns true if the average of the last 10 numbers is less than the error margin
	}
	else {
		return false; // Returns false if the average of the last 10 numbers is greater than the error margin
	}
}
std::vector<float> PID::cartesianToClock(const std::vector<float>& localVector) { // DEPRECIATED, REPLACED WITH INLINE FUNCTION
	std::vector<float> clockTarget(2);
	clockTarget[0] = sqrt((localVector[0] * localVector[0]) + (localVector[1] * localVector[1]));
	clockTarget[1] = atan2(localVector[1], localVector[0]);
	return clockTarget;
}
float PID::deltaDegrees(float a, float b) {
	float delta = fmodf(b - a + 180.0, 360.0);
	if (delta < 0) delta += 360.0;
	delta -= 180.0;
	return delta;
}


void PID::pidUpdate(){
	// Calculate local target vector (relative to robot)
	float targetLocalX = (*targetGlobal)[0] - (*currentPosition)[0];
	float targetLocalY = (*targetGlobal)[1] - (*currentPosition)[1];
	float theta = (*currentHeading) * (M_PI/180.0f); // heading in radians
	
	// Rotate the global target vector into the robot-local frame (rotate by -theta)
	// Standard frame: X = forward (drive), Y = right (strafe)
	float localizedX = targetLocalX * cosf(theta) + targetLocalY * sinf(theta);   
	float localizedY = -targetLocalX * sinf(theta) + targetLocalY * cosf(theta);  




	// errorDistance is forward motion (X component in robot frame)
	errorDistance = localizedX;

	float errorTurning = atan2(localizedY, localizedX) * 180.0 / M_PI; // angle in degrees

	if(config::debugPID){
		//printf("\nLX:%.3f, LY:%.3f, ED:%.3f, ET:%.3f", localizedX, localizedY, errorDistance, errorTurning);
		//printf("\nAngle to Target: %.3f", angleToTarget);
	}
	


	// If target is behind, reverse direction
	if (fabsf(errorTurning) > 90 && (*targetGlobal)[4] == 1) {
		errorDistance = -errorDistance;
		errorTurning = deltaDegrees(errorTurning, 180);
		if(config::debugPID){
			printf(" Reversing\n\n\n\n\n\n\n\n");
		}
	}

	
	//Updates ID variables
	integral += errorDistance;
	float deriviative = errorDistance - previousError;
	previousError = errorDistance;
	//Updates the turning ID variables
	turnIntegral += errorTurning;
	float turnDerivative = errorTurning - turnPreviousError;
	turnPreviousError = errorTurning;

	// Calculates the power amounts for distance and turning
	float maxSpeed = (*targetGlobal)[2];
	float maxTurnSpeed = (*targetGlobal)[3];

	float powerDistance = DKp * errorDistance + DKi * integral + DKd * derivative; // Pid formula
	//float powerDistance = DKp * localizedX + DKi * integral + DKd * derivative; // Pid formula
	powerDistance = std::clamp(powerDistance, -maxSpeed, maxSpeed); // Keep values between max speed in both directions

	float powerTurning = TKp * errorTurning + TKi * turnIntegral + TKd * turnDerivative; // Pid formula
	powerTurning = std::clamp(powerTurning, -maxTurnSpeed, maxTurnSpeed);

	float powerStrafe = SKp * errorStrafe * SKi * strafeIntegral + SKd * strafeDerivative;
	if (powerStrafe > maxStrafeSpeed){
		powerStrafe = maxStrafeSpeed;
	}
	else if (powerStrafe < -maxStrafeSpeed){
		powerStrafe = -maxStrafeSpeed;
	}
	if (fabsf(errorDistance) < errorMargin && finalPointInPath) {
		powerDistance = 0;
		powerTurning = 0;
		integral = 0;
		turnIntegral = 0;
		finalPointInPath = false;
		pathComplete = true;
	}
	if(errorDistance < turnErrorMargin && finalPointInPath){ // If robot is close enough to the final point, turn off turning controller to prevent osicilations
		powerTurning = 0;
	}
	/*
	//float distanceFromPoint = sqrt(pow(targetLocal[0], 2) + pow(targetLocal[1], 2));
	if (!errorSumUpdate(errorDistance) && config::debugPID){
		printf("Point reached!");
	}
	*/
		
	leftside->move(powerDistance - powerTurning);
	rightside->move(powerDistance + powerTurning);
	if(config::debugPID){
		printf("\errorTurning: %.3f,", errorTurning);
		//printf("\nPDL:%.3f, PDR:%.3f, PT:%.3f,", powerDistance+powerTurning, powerDistance - powerTurning, powerTurning);
		//std::cout <<"Power Distance left: " << powerDistance + powerTurning << "Power Distance Right:" << powerDistance - powerTurning  << "\nPower Turning: " << powerTurning << std::endl;
	}


}
