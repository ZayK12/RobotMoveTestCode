#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <array>
#include "api.h"
#include "pros/adi.hpp"
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
		* @param   _TKd Turning Derivative constant.*/	      // (_ is to prevent name clashes)
		PID(std::vector<double>* Point, std::vector<double>* position, double* heading, double _maxDSpeed, double _maxTSpeed, double _DKp, double _DKi, double _DKd, double _TKp, double _TKi, double _TKd, pros::MotorGroup* _leftMotors, pros::MotorGroup* _rightMotors);
		/**
		* @author  Zayyaan K
		* @date    5/21/25
		* @brief   Constructor for the PID class with additional parameters.
		* @param   ERM error margin before cancelling
		* @param   TERM turning error margin before cancelling
		* @param   ESM error sum margin (read function to understand)
		* @param   pathDiv number of divisions for the final path
		* @param   MPCA motor power check amount (if the motor power is less than this, stop the robot)
		*/
		PID(std::vector<double>* Point, std::vector<double>* position, double* heading, double _maxDSpeed, double _maxTSpeed, double _DKp, double _DKi, double _DKd, double _TKp, double _TKi, double _TKd, double ERM, double TERM, double ESM, int pathDiv, double MPCA, pros::MotorGroup* _leftMotors, pros::MotorGroup* _rightMotors);
		/**
		* @author  Zayyaan K
		* @date    5/21/25
		* @brief   updates pid values
		*/
		void pidUpdate();
		
	private:
		std::vector<double>* targetGlobal; // Pointer to the target point	
		std::vector<double>* currentPosition; //Pointer to the robot's position
		std::vector<double>* targetLocal; // Pointer to the target point
		pros::MotorGroup* leftside; // Pointer to the left motors
  		pros::MotorGroup* rightside; // Pointer to the right motors
		double* currentHeading; // Pointer to the robot's heading
		double DKp, DKi, DKd; // Forward PID constants
		double TKp, TKi, TKd; // Turning PID constants
		double maxDSpeed, maxTurnSpeed = 80; // Maximum speed 
		double errorDistance, errorTurning; // Error values
		double integral, derivative, previousError; //ID variables
		double turnIntegral, turnDerivative, turnPreviousError; //Turning ID variables
		double errorSumMargin, turnErrorMargin, errorMargin = 1; // Error margin
		int pathDivamt = 4; // Number of divisions for final path 
		double motorPowerCheckAmt = 10.0; // if motor power is less than this, stop the robot

		// 5/21/25 Modified version of my original errorSumUpdate function
		// Uses a circular buffer rather than the old linear one
		//goal of function is to delete the oldest number (last number) and add the new number upfront and update //the errorSum accordingly
		bool errorSumUpdate(double inputNumber);
		
		// Converts a vector from cartesian coordinates (x, y) to clock coordinates (radius, angle)
		std::vector<double> cartesianToClock(const std::vector<double>& localVector);

		//Gets the angle between two points in degrees
		double deltaDegrees(double a, double b); 



};

