#include <vector>
#include "config.h"
#include <string>
namespace config {

	std::vector<double> INITIAL_POSITION = { 1, 4 };
	std::string filePath = "C:\\Users\\Admin\\Downloads\\path.jerryio2.txt";

	//Initial PID paramaters
	double MAX_FORWARD_SPEED = 100.0; // Maximum forward speed
	double MAX_TURN_SPEED = 100.0; // Maximum turning speed
	double FORWARD_KP = 0.1; // Forward Proportional constant
	double FORWARD_KI = 0.01; // Forward Integral constant
	double FORWARD_KD = 0.01; // Forward Derivative constant
	double TURN_KP = 0.1; // Turning Proportional constant
	double TURN_KI = 0.01; // Turning Integral constant
	double TURN_KD = 0.01; // Turning Derivative constant
	double ERROR_MARGIN = 1.0; // Error margin for stopping
	double TURN_ERROR_MARGIN = 1.0; // Turning error margin for stopping
	double ERROR_SUM_MARGIN = 1.0; // Error sum margin for stopping
	int PATH_DIVISIONS = 4; // Number of divisions for final path
	double MOTOR_POWER_CHECK_AMOUNT = 10.0; // If motor power is less than this, stop the robot

}
// You could also add motor declarations here if needed.


