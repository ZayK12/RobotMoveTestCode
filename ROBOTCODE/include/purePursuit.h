///@file purePursuit.h
///@brief This is the header file for the pure pursuit algorithm.
///@author Zayyaan K
///@date 4/17/25
#ifndef purePursuit_H
#define purePursuit_H

/// <summary>
/// Floating point imperfection, especailly at this level creates problems, on 5/9/25 I added this tolerance for whenever you are subtracting two floating point numbers.
/// </summary>
const float TOLERANCE = 1e-4; //Tolerance for floating point comparisons

#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <thread>
class Pursuit {
public:
    /// @author Zayyaan K
    /// @date 5/25/25
    /// @brief Constructor along with initalizer
    /// @param p Path
    /// @param pos robot position
    /// @param LKA Look Ahead
    Pursuit(std::vector<std::vector<float> >** p, std::vector<float>* pos, float LKA);

    /// @brief calculates a valid pursuit point within the given path
    /// @return a vector with position
    std::vector<float> updatePursuitPoint();

    /// @brief mutator method for lookAhead paramater
    /// @param newLKA the new value for lookAhead.
    void setLookAhead(float newLKA);

    std::vector<float> pursuitPoint = { 0.0, 0.0 }; // the set pursuit point, initialized at 0,0 to prevent errors
    int startIndex = 0;
	
private:
    float lookAhead;
	std::vector<std::vector<float> >** pathPointer; // pointer to the path
	std::vector<float>* positionPointer; // pointer to the robot position


    /// @author Zayyyan K
    /// @date 4/15/25
    /// @brief gets the sign of the number
    /// @param number number to get the sign of 
    /// @return -1 or 1. 
    float sign(float number) const;


    /// @author Zayyaan K
    /// @date 4/16/25 
    /// @brief returns the distance between two points
    /// @param point0 x1, y1
    /// @param point1 x2, y2
    /// @return the distance between point0 and point1
    float distance(std::vector<float> point0, std::vector<float> point1) const;

    /// @author Zayyaan K
    /// @date 1/16/26 
    /// @brief returns the distance between two points. This is 
    /// @param point0 x1, y1
    /// @param point1 x2, y2
    /// @return the distance between point0 and point1
    float distanceSquared(std::vector<float> point0, std::vector<float> point1) const;

    /// @author Zayyaan K
    /// @date 1/16/26 
    /// @brief returns the distance between two points. Using floats for point0
    /// @param pointX
    /// @param pointY
    /// @param point1 x2, y2
    /// @return the squared distance between point0 and point1
    float distanceSquared(float pointX, float pointY, std::vector<float> point1) const;

    /// @author Zayyaan K
    /// @date 1/21/26 
    /// @brief returns the distance between two points. Using floats for point0
    /// @param pointX
    /// @param pointY
    /// @param endPointX
    /// @param endPointY
    /// @return the distance between point0 and point1
    float distanceSquared(float pointX, float pointY, float endPointX, float endPointY) const;

    /// @author Zayyaan K
    /// @date 4/16/25
    /// @brief checks to find the closest poitn
    /// @param point point you want to measure to
    /// @param pointSet points you want to test
    /// @return the point from the pointset that is closest to point.
    std::vector<float> closestPoint(std::vector<float> point, std::vector<std::vector<float> > pointSet) const;

    /// @author Zayyaan K
    /// @date 1/16/25
    /// @brief checks to find the closest poitn
    /// @param pointX point you want to measure to X value
    /// @param pointY point you want to measure to Y value
    /// @param pointSet points you want to test
    /// @return the point from the pointset that is closest to point.
    std::vector<float> closestPoint(float pointX, float pointY, std::vector<std::vector<float> > pointSet) const;

    /// @author Zayyyan K
    /// @date 4/7/25
    /// @brief Finds discriminant between start and end of line
    /// @param point0X start of line
    /// @param point0Y start of line
    /// @param point1X end of line
    /// @param point1Y end of line
    /// @param dis distance between the two lines
    /// @param determen Determinant, pointer to local variable in main function to change. Added 4/15/25 to stop having to recalculate multiple times around the code.
    /// @return Discriminant
    float findDiscrim(float point0X, float point0Y, float point1X, float point1Y, float dis, float& determen);


    /// @author Zayyaan K
    /// @date 4/16/25
    /// @brief Checks to see if the point is within the line segment
    /// @param startPoint the starting point of the line segment
    /// @param endPoint th ending point of the line segment 
    /// @param sol1 the point to check
    /// @return true if within the segment, and false if not.
    bool inLimit(float startPointX, float startPointY, float endPointX, float endPointY, std::vector<float> sol1);
    /// @author Zayyaan K
    /// @date 1/16/26
    /// @brief Checks to see if the point is within the line segment using squared distances
    /// @param startPoint the starting point of the line segment
    /// @param endPoint th ending point of the line segment 
    /// @param sol1 the point to check
    /// @return true if within the segment, and false if not.
    bool inLimit2(std::vector<float> startPoint, std::vector<float> endPoint, std::vector<float> sol1);
    /// @author Zayyaan K
    /// @date 1/21/26
    /// @brief Checks to see if the point is within the line segment using squared distances
    /// @param startPointX starting X coordinate
    /// @param startPointY starting Y coordinate
    /// @param endPoint th ending point of the line segment 
    /// @param sol1 the point to check
    /// @return true if within the segment, and false if not.
    bool inLimit2(float startpointX, float startPointY, float endPointX, float endPointY , std::vector<float> sol1);
    /// @author Zayyaan K
    /// @date 4/15/25
    /// @brief Finds the XY values of both line intersections
    /// @param discrim Discriminant returned by the findDiscrim function
    /// @param determen Deteminant changed by the pointer in the findDiscrim function
    /// @param distanceX the distance between the 2 x variables, in p1 and p2
    /// @param distanceY the distance between the 2 y variables, in p1 and p2
    /// @param distance distance between p1 and p2
    /// @return 2 arrays containing both intersections local coordinates
    std::vector<std::vector<float> > hitPoints(float discrim, float determen, float distanceX, float distanceY, float distance);




};




#endif
