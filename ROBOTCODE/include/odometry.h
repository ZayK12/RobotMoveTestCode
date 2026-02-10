#ifndef odometry_H
#define odometry_H
#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <thread>
#include "main.h"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
/**
 * @todo add encoders within the constructor
 * @todo import the necessary libraries for whatever robotics interfac
 * @todo add other classes within the constructor ( this needs to be done for all files)
 * @todo maybe pass
 *
*/

class odometry {
public:

    /// @author Zayyaan Kazi
    /// @date 5/6/25
    /// @brief constructor w/ position and orientation inputs
    /// @param initPos Initial Position of robot
    /// @param initOrientation Initial orientation of robot
    /// @param wheelC wheel circumference
    /// @param distanceLeft distance from left encoder base to tracking center
    /// @param distanceRight distance from right encoder base to tracking center
    /// @param distanceBack  distance from back encoder of the robot to tracking center
    /// @param _rightEncoder pointer to right encoder
    /// @param backEncoder pointer to back encoder
    odometry(std::vector<float> initPos, float initOrientation, float wheelC, float distanceLeft, float distanceRight, float distanceBack, pros::Rotation* _rightEncoder, pros::Rotation* backEncoder, pros::v5::IMU* _leftIMU, pros::v5::IMU* _rightIMU);


    /**
     * @author Zayyaan K
     * @date 5/6/25
     * @brief constructor w/o position and orientation inputs
     * @param wheelC wheel circumference
     * @param distanceLeft distance from left encoder base to tracking center
     * @param distanceRight distance from right encoder base to tracking center
     * @param distanceBack  distance from back encoder of the robot to tracking center */
    odometry(float wheelC, float distanceLeft, float distanceRight, float distanceBack,pros::Rotation* _rightEncoder, pros::Rotation* backEncoder, pros::v5::IMU* _leftIMU, pros::v5::IMU* _rightIMU);
    
   
    /**
     * @author Zayyaan K
     * @date 4/28/25
     * @brief Updates position values w/ changing ram values
     * @param newX new X value
     * @param newY new Y value */
    void updatePosition(float newX, float newY);

    /**
     * @author Zayyaan K
     * @date 5/9/25
     * @brief Updates position values w/ changing ram values using a vector
     * @param newPos new position. */
    void updatePosition(std::vector<float> newPos);

    /**
	* @author Zayyaan K
    * @date 6/4/25
	* @brief Updates Orientation and position values w/ changing ram values using a vector
	* @param newPos new position vector
    * @param newOrientation new orientation value*/
    void updatePosition(std::vector<float> newPos, float newOrientation);
    
    /**
    * @author Zayyaan K
	* @date 5/9/25
    * @brief Gets the pointer to the position vector
	* @return pointer to the position vector */
    std::vector<float>* getPositionPointer();

	/**
	 * @author Zayyaan K
	 * @date 6/4/25
	 * @brief Gets the pointer to the orientation value
	 * @return pointer to the orientation value (radians)*/
    float* getOrientationPointer();
    

    std::vector<float> MainPosition;
    /**
     * @brief updates encoder values
     * @note also automatically calls the calculate local offset, may need to do some c++ trickery */
    void updateDistances();

private:
    
    std::vector<float>* directPositionPtr;
    float orientation; 
    const float disL;
    const float disR;
    const float disB;
    const float wheelCircum;
    pros::Rotation* rightEncoder;
    pros::Rotation* backEncoder;
    pros::v5::IMU* leftIMU;
    pros::v5::IMU* rightIMU;
    const float TOLERANCE = 1e-5; //Tolerance for floating point comparisons
    
    std::vector<float> localOffset;
    /**
     * @brief  This function is to prevent the IMU to return a rotation of 360 as 0 is preferred
     * @param deg the variable to check*/
    float overflowCheck(float deg);
    /**
     * @brief gets the delta (absolute difference) of 2 radians
     * @param rad1 Radian 1
     * @param rad2 Radian 2
     * @return the change between rad1 and rad2. */
    float subRadians(float rad1, float rad2);
    /**
     * @brief gets the delta (absolute difference) of 2 degrees, normalized to [-180, 180]
     * @param deg1 Degree 1
     * @param deg2 Degree 2
     * @return the change between deg1 and deg2. */
    float subDegrees(float deg1, float deg2);
    /**
     * @param coordSet pointer to vector holding doubles. Typically {0,0}. Mutates them */
    void cartesianToPolar(std::vector<float>& coordSet);
    /**
     * @brief converts a polar set of coordinates to a cartesian set of coordinates
     * @param radius radius from polar set
     * @param theta theta from polar set*/
    void polarToCartesian(float& radius, float& theta);
    

    /**
     * @brief updates the changes in X and Y values*/
    void calculateLocalOffset(float backInchDelta_, float rightInchDelta_, float headingDelta_);

    /**
     * @brief Fixes the rotation offset and updates the global position values */
    void rotateToGlobalFrame();


};




#endif
