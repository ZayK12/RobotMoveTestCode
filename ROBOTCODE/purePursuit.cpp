#include "purePursuit.h"
///@author Zayyaan K
///@date 4/25/25
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <array>
#include <vector>
#include <string>




Pursuit::Pursuit(std::vector<std::vector<double>>** p, std::vector<double>* pos, double LKA) : pathPointer(p), positionPointer(pos), lookAhead(LKA) {}


void Pursuit::setLookAhead(double newLKA) {
    lookAhead = newLKA;
}
double Pursuit::sign(double number) const {
    return number / fabs(number);
}
double Pursuit::distance(std::vector<double> point0, std::vector<double> point1) const {
    return sqrt(pow(point0[0] - point1[0], 2) + pow(point0[1] - point1[1], 2));
}


std::vector<double> Pursuit::closestPoint(std::vector<double> point, std::vector<std::vector<double>> pointSet) const {
    if (distance(point, pointSet[0]) > distance(point, pointSet[1])) {
        return pointSet[1];
    }
    return pointSet[0];
}

double Pursuit::findDiscrim(std::vector<double> point0, std::vector<double> point1, double dis, double& determen) {

    double* determ = &determen;

    *determ = point0[0] * point1[1] - point0[1] * point1[0]; //x1*y2 - x2*y1

    double Discrim = pow(lookAhead, 2) * pow(dis, 2) - pow(*determ, 2); //lookAhead^2 * dis^2 - determ^2
    return Discrim;
}

std::vector<std::vector<double>> Pursuit::hitPoints(std::vector<double> p1, std::vector<double> p2, double discrim, double determen, double distanceX, double distanceY, double distance) {

    std::vector<double> intsec1 = {//Positive Variation
        //X positive value
        (determen * distanceY + sign(distanceY) * distanceX * sqrt(discrim)) / pow(distance, 2),
        //Y positive value
        (-determen * distanceX + fabs(distanceY) * sqrt(discrim)) / pow(distance,2) };
    std::vector<double> intsec2 = {//Negative Variation
        //X negative value
        (determen * distanceY - sign(distanceY) * distanceX * sqrt(discrim)) / pow(distance, 2),
        //Y negative value
        (-determen * distanceX - fabs(distanceY) * sqrt(discrim)) / pow(distance,2) };
    std::vector <std::vector<double>> intsecs = { intsec1, intsec2 };
    return intsecs;
}

bool Pursuit::inLimit(std::vector<double> startPoint, std::vector<double> endPoint, std::vector<double> sol1) {
    if (fabs(distance(startPoint, sol1) + distance(sol1, endPoint) - distance(startPoint, endPoint)) < TOLERANCE) {
        return true;
    }
    return false;
}

std::vector<double> Pursuit::updatePursuitPoint() {


    double DisX;
    double DisY;
    double DisR;
    double discrim;
    double determinant;
    int startIndex = 0;

    const auto& path = **pathPointer;          // 5/7 Derefrences the path vector to increase performance 
    const auto& position = *positionPointer;  // 5/7 Derefrences the position vector to increase performance 
 
    
    double distanceToEnd = distance(path[path.size() - 1], position);
    if (lookAhead > distanceToEnd) {
        return path[path.size() - 1];
    }
    for (int i = 0; i < path.size() - 1; i++) {
        //convert points to local arrays
        std::vector<double> lPoint0 = { path[i][0] - position[0], path[i][1] - position[1] };
        std::vector<double> lPoint1 = { path[i + 1][0] - position[0], path[i + 1][1] - position[1] };
        //distance between both X values
        DisX = lPoint1[0] - lPoint0[0];
        //distance between both Y values
        DisY = lPoint1[1] - lPoint0[1];
        //abs distance between both points
        DisR = sqrt(pow(DisX, 2) + pow(DisY, 2));
        discrim = findDiscrim(lPoint0, lPoint1, DisR, determinant);
        if (discrim < -TOLERANCE) { continue; } // Discrim being 0 means the circle is not on the line, so it continues to the next point.

        //Function to find where it hit the line
        std::vector<std::vector<double>> solutionsLocal = hitPoints(lPoint0, lPoint1, discrim, determinant, DisX, DisY, DisR);


        //Checks if the solutions are within the line segment
        bool solution1InLimit = inLimit(lPoint0, lPoint1, solutionsLocal[0]);
        bool solution2InLimit = inLimit(lPoint0, lPoint1, solutionsLocal[1]);


        //If both are within limits decides what point to go with. Also updates the startIndex variable
        if (solution1InLimit && solution2InLimit) {
            //Finds the closest point 
            std::vector<double> gSolution = closestPoint(lPoint1, solutionsLocal);

            //the if statement below is to prevent the search algorithm from targeting segments the robot has already traveled.
            if (distance(gSolution, lPoint1) > distance({ 0.0,0.0 }, lPoint1)) {
                startIndex = i;
            }
            else {
                startIndex = i + 1;
            }
            gSolution[1] += position[1];
            gSolution[0] += position[0];
            return gSolution;
        }
        else if (solution1InLimit && !solution2InLimit) //if solution1 is in limit and solution 2 isn't
        {
            std::vector<double> gSolution = solutionsLocal[0];
            if (distance(gSolution, lPoint1) > distance({ 0.0,0.0 }, lPoint1)) {
                startIndex = i;
            }
            else {
                startIndex = i + 1;
            }
            gSolution[0] += position[0];
            gSolution[1] += position[1];
            return gSolution;
        }

    }
    for (int i = 0; i < path.size(); i++) {
        std::cout << "X:" << path[i][0] << " Y:" << path[i][0] << std::endl;
    }
    return path[path.size() - 1];
}