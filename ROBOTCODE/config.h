#ifndef CONFIG_H
#define CONFIG_H
#include <vector>
namespace config {

	const double WHEEL_CIRCUMFERENCE = 0.5;

	const double ENCODER_DISTANCE_LEFT = 2.0;
	const double ENCODER_DISTANCE_RIGHT = 2.0;
	const double ENCODER_DISTANCE_BACK = 2.0;

	const std::vector<double> INITIAL_POSITION = { 2.0, 5.0 };
	const double INITIAL_ORIENTATION = 0.0;
}
// You could also add motor declarations here if needed.
#endif // CONFIG_H

