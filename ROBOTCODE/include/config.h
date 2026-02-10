#ifndef CONFIG_H
#define CONFIG_H
#include <vector>
namespace config {

	inline constexpr double WHEEL_CIRCUMFERENCE = 0.5;

	inline constexpr double ENCODER_DISTANCE_LEFT = 2.0;
	inline constexpr double ENCODER_DISTANCE_RIGHT = 2.0;
	inline constexpr double ENCODER_DISTANCE_BACK = 2.0;

	inline constexpr double INITIAL_ORIENTATION = 0.0;
	extern std::vector<double> INITIAL_POSITION;

	extern std::string filePath;

	//Initial PID paramaters
	extern double MAX_FORWARD_SPEED; // Maximum forward speed
	extern double MAX_TURN_SPEED; // Maximum turning speed
	extern double FORWARD_KP; // Forward Proportional constant
	extern double FORWARD_KI; // Forward Integral constant
	extern double FORWARD_KD; // Forward Derivative constant
	extern double TURN_KP; // Turning Proportional constant
	extern double TURN_KI; // Turning Integral constant
	extern double TURN_KD; // Turning Derivative constant
	extern double ERROR_MARGIN; // Error margin for stopping
	extern double TURN_ERROR_MARGIN; // Turning error margin for stopping
	extern double ERROR_SUM_MARGIN; // Error sum margin for stopping
	extern int PATH_DIVISIONS; //Number of divisions for final path
	extern double MOTOR_POWER_CHECK_AMOUNT; // If motor power is less than this, stop the robot

}
// You could also add motor declarations here if needed.
#endif // CONFIG_H

