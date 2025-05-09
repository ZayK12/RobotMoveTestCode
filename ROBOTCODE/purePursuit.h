///@file purePursuit.h
///@brief This is the header file for the pure pursuit algorithm.
///@author Zayyaan K
///@date 4/17/25
#ifndef purePursuit_H
#define purePursuit_H

/// <summary>
/// Floating point imperfection, especailly at this level creates problems, on 5/9/25 I added this tolerance for whenever you are subtracting two floating point numbers.
/// </summary>
const double TOLERANCE = 1e-6; //Tolerance for floating point comparisons

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
    Pursuit(std::vector<std::vector<double> >** p, std::vector<double>* pos, double LKA);

    /// @brief calculates a valid pursuit point within the given path
    /// @return a vector with position
    std::vector<double> updatePursuitPoint();

    /// @brief mutator method for lookAhead paramater
    /// @param newLKA the new value for lookAhead.
    void setLookAhead(double newLKA);
private:
    double lookAhead;
    std::vector<std::vector<double> >** pathPointer;
    std::vector<double>* positionPointer;


    /// @author Zayyyan K
    /// @date 4/15/25
    /// @brief gets the sign of the number
    /// @param number number to get the sign of 
    /// @return -1 or 1. 
    double sign(double number) const;


    /// @author Zayyaan K
    /// @date 4/16/25 
    /// @brief returns the distance between two points
    /// @param point0 x1, y1
    /// @param point1 x2, y2
    /// @return the distance between point0 and point1
    double distance(std::vector<double> point0, std::vector<double> point1) const;


    /// @author Zayyaan K
    /// @date 4/16/25
    /// @brief checks to find the closest poitn
    /// @param point point you want to measure to
    /// @param pointSet points you want to test
    /// @return the point from the pointset that is closest to point.
    std::vector<double> closestPoint(std::vector<double> point, std::vector<std::vector<double> > pointSet) const;

    /// @author Zayyyan K
    /// @date 4/7/25
    /// @brief Finds discriminant between start and end of line
    /// @param point0 start of line
    /// @param point1 end of line
    /// @param dis distance between the two lines
    /// @param determen Determinant, pointer to local variable in main function to change. Added 4/15/25 to stop having to recalculate multiple times around the code.
    /// @return Discriminant
    double findDiscrim(std::vector<double> point0, std::vector<double> point1, double dis, double& determen);


    /// @author Zayyaan K
    /// @date 4/16/25
    /// @brief Checks to see if the point is within the line segment
    /// @param startPoint the starting point of the line segment
    /// @param endPoint th ending point of the line segment 
    /// @param sol1 the point to check
    /// @return true if within the segment, and false if not.
    bool inLimit(std::vector<double> startPoint, std::vector<double> endPoint, std::vector<double> sol1);
    /// @author Zayyaan K
    /// @date 4/15/25
    /// @brief Finds the XY values of both line intersections
    /// @param p1 start of line local coordinates
    /// @param p2 end of line local coordinates
    /// @param discrim Discriminant returned by the findDiscrim function
    /// @param determen Deteminant changed by the pointer in the findDiscrim function
    /// @param distanceX the distance between the 2 x variables, in p1 and p2
    /// @param distanceY the distance between the 2 y variables, in p1 and p2
    /// @param distance distance between p1 and p2
    /// @return 2 arrays containing both intersections local coordinates
    std::vector<std::vector<double> > hitPoints(std::vector<double> p1, std::vector<double> p2, double discrim, double determen, double distanceX, double distanceY, double distance);




};




#endif