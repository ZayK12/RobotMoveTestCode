#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <array>
#include "main.h"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"      // For pros::Motor
#include "pros/motor_group.hpp" // For pros::MotorGroup
class PID
{
	public:
		/**
		* @author  Zayyaan K
		* @date    5/21/25
		* @brief   Constructor for the PID class.
		* @param   Point Pointer to the target point.
		* @param   position Pointer to the current position.
		* @param   heading Pointer to the current heading
		* @param   _maxDSpeed Maximum speed for forward movement (_ is to prevent name clashes)
		* @param   _maxTSpeed Maximum speed for turning movement (_ is to prevent name clashes)
		* @param   _DKp Forward Proportional constant.			 (_ is to prevent name clashes)
		* @param   _DKi Forward Integral constant.				 (_ is to prevent name clashes)
		* @param   _DKd Forward Derivative constant.			 (_ is to prevent name clashes)
		* @param   _TKp Turning Proportional constant.			 (_ is to prevent name clashes)
		* @param   _TKi Turning Integral constant.				 (_ is to prevent name clashes)
		* @param   _TKd Turning Derivative constant.	    */// (_ is to prevent name clashes)
		PID(std::vector<float>* Point, std::vector<float>* position, float* heading, float& _maxDSpeed, float& _maxTSpeed,float& _DKp, float& _DKi, float& _DKd, float& _TKp, float& _TKi, float& _TKd, pros::MotorGroup* _leftMotors, pros::MotorGroup* _rightMotors);
		/**
		* @author  Zayyaan K
		* @date    5/21/25
		* @brief   Constructor for the PID class with additional parameters.
		* @param   ERM error margin before cancelling
		* @param   TERM turning error margin before cancelling
		* @param   ESM error sum margin (read function to understand)
		* @param   pathDiv number of divisions for the final path
		* @param   MPCA motor power check amount (if the motor power is less than this, stop the robot)
		* All other parameters are the same as the other constructor.
		*/
		PID(std::vector<float>* Point, std::vector<float>* position, float* heading, float& _maxDSpeed, float& _maxTSpeed, float& _DKp, float& _DKi, float& _DKd, float& _TKp, float& _TKi, float& _TKd,float& ERM, float& TERM, float& ESM, int& pathDiv, float& MPCA, pros::MotorGroup* _leftMotors, pros::MotorGroup* _rightMotors);
		/**
		* @author  Zayyaan K
		* @date    5/21/25
		* @brief   updates pid values
		*/
		void pidUpdate();

		/**
		 * @author  Zayyaan K
		 * @date    9/2/25
		 * @brief   Sets global target point
		 */
		void setTargetPoint();
		
		/**
		 * @author  Zayyaan K
		 * @date    9/2/25
		 * @brief   changes the max D speed
		 * @param   newMaxSpeed new max speed*/
		void setMaxDSpeed(float newMaxSpeed);
		/**
		 * @author  Zayyaan K
		 * @date    9/2/25
		 * @brief   changes the max turn speed
		 * @param   newMaxTurnSpeed new max turn speed */
		void setMaxTurnSpeed(float newMaxTurnSpeed);
		/**
		 * @author Zayyaan K
		 * @date 12/26/25
		 * @brief Sets lateral constants
		 * @param newDKp new proprotional constant for lateral movement
		 * @param newDKi new integral constant for lateral movement
		 * @param newDKd new derivative constant for lateral movement */
		void setLateralConstants(float newDKp, float newDKi, float newDKd);
		/**
		 * @author Zayyaan K
		 * @date 12/26/25
		 * @brief Sets turning constants
		 * @param newTKp new proprotional constant for turning
		 * @param newTKi new integral constant for turning
		 * @param newTKd new derivative constant for turning */
		void setTurningConstants(float newTKp, float newTKi, float newTKd);

		bool finalPointInPath = false;
		bool pathComplete = false;
	private:
		std::vector<float>* targetGlobal; // Pointer to the target point	
		std::vector<float>* currentPosition; //Pointer to the robot's position
		pros::MotorGroup* leftside; // Pointer to the left motors
  		pros::MotorGroup* rightside; // Pointer to the right motors
		pros::Motor* strafeMotor; // Pointer to the strafe motor (may be referred to as H motor)
		float* currentHeading; // Pointer to the robot's heading (Radians)
		float DKp, DKi, DKd; // Forward PID constants
		float TKp, TKi, TKd; // Turning PID constants
		float SKp, SKi, SKd; // Strafing PID constants
		float maxDSpeed, maxTurnSpeed, maxStrafeSpeed; // Maximum speed 
		float errorDistance, errorTurning, errorStrafe; // Error values
		float integral, derivative, previousError; //ID variables
		float turnIntegral, turnDerivative, turnPreviousError; //Turning ID variables
		float strafeIntegral, strafeDerivative, strafePreviousError; // Strafing ID variables
		float errorSumMargin, turnErrorMargin, errorMargin = 1; // Error margin
		float strafeErrorMargin, strafeSumErrorMargin = 1; // Strafe error margins
		int pathDivamt = 4; // Number of divisions for final path 
		float motorPowerCheckAmt = 10.0; // if motor power is less than this, stop the robot

		// 5/21/25 Modified version of my original errorSumUpdate function
		// Uses a circular buffer rather than the old linear one
		//goal of function is to delete the oldest number (last number) and add the new number upfront and update //the errorSum accordingly
		bool errorSumUpdate(float inputNumber);
		
		// Converts a vector from cartesian coordinates (x, y) to clock coordinates (radius, angle)
		std::vector<float> cartesianToClock(const std::vector<float>& localVector);

		//Gets the angle between two points in degrees
		float deltaDegrees(float a, float b); 



};

