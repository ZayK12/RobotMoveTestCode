#include "main.h"
#include "api.h"
#include "pathManager.h"
#include "odometry.h"
#include "purePursuit.h"
#include "PID.h"
#include "config.h"
#include "pathManager.h"

pros::adi::Encoder encoderLeft ('A', 'B');
pros::adi::Encoder encoderRight ('C', 'D');
pros::adi::Encoder encoderBack ('E', 'F');


/*
Motor Ports goes as follows:
1 - Front Left Motor 
2 - Back Left Motor
3 - Front Right Motor
4 - Back Right Motor*/
pros::MotorGroup leftMotors ({1, 2}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup rightMotors ({3, 4}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);


 
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
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
    double lookAhead = 8; // Example look ahead distance
    pathManager activePath = pathManager(path); // Example path manager
    odometry roboOdom = odometry(config::INITIAL_POSITION, config::INITIAL_ORIENTATION, config::WHEEL_CIRCUMFERENCE, config::ENCODER_DISTANCE_LEFT, config::ENCODER_DISTANCE_RIGHT, config::ENCODER_DISTANCE_BACK, &encoderLeft, &encoderRight, &encoderBack); // Example odometry @todo 
    Pursuit roboPursuit = Pursuit(activePath.getPathPointer(), roboOdom.getPositionPointer(), lookAhead); // Example pure pursuit
    std::vector<double>* pursuitPointer = &roboPursuit.pursuitPoint;
    PID roboPID = PID(&roboPursuit.pursuitPoint, roboOdom.getPositionPointer(), roboOdom.getOrientationPointer(), config::MAX_FORWARD_SPEED, config::MAX_TURN_SPEED, config::FORWARD_KP, config::FORWARD_KI, config::FORWARD_KD, config::TURN_KP, config::TURN_KI, config::TURN_KD, config::ERROR_MARGIN, config::TURN_ERROR_MARGIN, config::ERROR_SUM_MARGIN, config::PATH_DIVISIONS, config::MOTOR_POWER_CHECK_AMOUNT, &leftMotors, &rightMotors); // Example PID controller
    //mmactivePath.updatePathFromFile(); // Example path update from file   
    
    std::vector<double>* posPointer14 = roboOdom.getPositionPointer();
    
   
    roboPursuit.updatePursuitPoint(); // Example pursuit point calculation
    std::cout << " bitch robot position" << (*posPointer14)[0] << ", " << (*posPointer14)[1] << std::endl;
    std::cout << "Pursuit Point: (" << (*pursuitPointer)[0] << ", " << (*pursuitPointer)[1] << ") Max Speed:" << (*pursuitPointer)[2] << std::endl; // Output the pursuit point
    roboPID.pidUpdate();
    
	//activePath.setActivePath(pathPlusOne); 
    roboPursuit.updatePursuitPoint(); // Example pursuit point calculation
    std::cout << "Pursuit Point: (" << (*pursuitPointer)[0] << ", " << (*pursuitPointer)[1] << ") Max Speed:" << (*pursuitPointer)[2]  << std::endl; // Output the pursuit point
}
