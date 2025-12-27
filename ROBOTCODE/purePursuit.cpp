#include "purePursuit.h"
#include "config.h"
///@author Zayyaan K
///@date 4/25/25
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>




Pursuit::Pursuit(std::vector<std::vector<double>>** p, std::vector<double>* pos, double LKA) : pathPointer(p), positionPointer(pos), lookAhead(LKA), pursuitPoint({ 0, 0, 0 ,0, 0}) {}


void Pursuit::setLookAhead(double newLKA) {
    lookAhead = newLKA;
}
double Pursuit::sign(double number) const {
    if (number == 0) {
        return 1;
    }
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
    if (Discrim < TOLERANCE && Discrim > -TOLERANCE) {
        Discrim = 0.0; //Corrects for floating point errors
    }
    // Clamp negative discriminants to zero to prevent NaN from sqrt
    if (Discrim < 0.0) {
        Discrim = 0.0;
    }
    if (config::debugPursuit) {
        printf("\n Dis: %.3f, Det: %.3f\n", Discrim, *determ);
        //std::cout << "\n Discriminant: " << Discrim << " Determinant: " << *determ << std::endl;
    }
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
    if (config::debugPursuit) {
        printf("Intersection Points: \nPair 1: (%.3f,%.3f) \nPair2(%.3f,%.3f)\n", intsec1[0],intsec1[1],intsec2[0],intsec2[1]);
        /*
        std::cout << "Intersection Points: \n";
        std::cout << "X1: " << intsec1[0] << " Y1: " << intsec1[1] << std::endl;
        std::cout << "X2: " << intsec2[0] << " Y2: " << intsec2[1] << std::endl;
        */
    }
    return intsecs;
}

bool Pursuit::inLimit(std::vector<double> startPoint, std::vector<double> endPoint, std::vector<double> sol1) {
    // Check if intersection point lies on the line segment using parametric projection
    double dx = endPoint[0] - startPoint[0];
    double dy = endPoint[1] - startPoint[1];
    double lengthSq = dx*dx + dy*dy;
    
    if (lengthSq < 1e-10) {  // Segment is essentially a point
        return false;
    }
    
    // Project sol1 onto the line segment: t = dot(sol1-start, end-start) / |end-start|^2
    double t = ((sol1[0] - startPoint[0]) * dx + (sol1[1] - startPoint[1]) * dy) / lengthSq;
    
    // Accept if t is between 0 and 1 (point lies on the segment)
    // Use a reasonable tolerance for floating point comparison
    const double t_tolerance = 0.01; // Allow small overshoot
    if (t >= -t_tolerance && t <= 1.0 + t_tolerance) {
        if (config::debugPursuit) {
            printf("\nPoint:(%.3f,%.3f) ON segment [t:%.3f]", sol1[0], sol1[1], t);
        }
        return true;
    }
    
    if (config::debugPursuit) {
        printf("\nPoint:(%.3f,%.3f) NOT on segment [t:%.3f]", sol1[0], sol1[1], t);
    }
    return false;
}

std::vector<double> Pursuit::updatePursuitPoint() {
    if(config::debugPursuit){
        printf("\nUpdate pursuit point start");
    }
    double DisX;
    double DisY;
    double DisR;
    double discrim;
    double determinant;
    bool foundSolution = false;

    const auto& path = **pathPointer;          // 5/7/25 Derefrences the path vector to increase performance 
    const auto& position = *positionPointer;  // 5/7/25 Derefrences the position vector to increase performance 
 
    if (config::debugPursuit) {
        printf("\nPOS:(%.3f,%.3f)", position[0],position[1]);
        //std::cout << "Robot Position: (" << position[0] << ", " << position[1] << ")" << std::endl;
    }
    double distanceToEnd = distance(path[path.size() - 1], position);
    if (config::debugPursuit) {
        printf("Distance to end: %.3f", distanceToEnd);
        //std::cout << "Distance to end: " << distanceToEnd << std::endl;
    }
    if (lookAhead > distanceToEnd) {
        if (config::debugPursuit) {
           printf("Lookahead greater than distance to end.");
        }
        return path[path.size() - 1];
    }
    for (int i = startIndex; i < path.size() - 1; i++) {
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
        if (discrim <= 0) { continue; } // Discrim being 0 means the circle is not on the line, so it continues to the next point.

        //Function to find where it hit the line
        std::vector<std::vector<double>> solutionsLocal = hitPoints(lPoint0, lPoint1, discrim, determinant, DisX, DisY, DisR);


        //Checks if the solutions are within the line segment
        bool solution1InLimit = inLimit(lPoint0, lPoint1, solutionsLocal[0]);
        bool solution2InLimit = inLimit(lPoint0, lPoint1, solutionsLocal[1]);


        //If both are within limits decides what point to go with. Also updates the startIndex variable
        if (solution1InLimit && solution2InLimit) {
            //Finds the closest point ahead of the robot (not behind)
            //Always prefer the point closer to the end of the segment to progress forward
            std::vector<double> lclosestPoint = closestPoint(lPoint1, solutionsLocal);
            pursuitPoint[0] = lclosestPoint[0];// Set the local X value for pursuitPoint
            pursuitPoint[1] = lclosestPoint[1];// Set the local Y value for pursuitPoint
			pursuitPoint[2] = (path[i][2] + path[i+1][2]) / 2; //average max speed of the 2 points
            pursuitPoint[3] = (path[i][3] + path[i+1][3]) / 2; //average max speed of the 2 points
            pursuitPoint[4] = (path[i+1][4]);
            
            pursuitPoint[0] += position[0]; //Convert from local to global frame
            pursuitPoint[1] += position[1]; //Convert from local to global frame
            
            if (config::debugPursuit) {
                printf("\nFound on segment %d: (%.3f, %.3f)", i, pursuitPoint[0], pursuitPoint[1]);
            }
            
            foundSolution = true;


        }
        else if (solution1InLimit && !solution2InLimit) { //if solution1 is in limit and solution 2 isn't
            pursuitPoint[0] = solutionsLocal[0][0]; // Set the local X value for pursuitPoint
			pursuitPoint[1] = solutionsLocal[0][1]; // Set the local Y value for pursuitPoint
            pursuitPoint[2] = (path[i][2] + path[i + 1][2]) / 2; //average max speed of the 2 points
            pursuitPoint[3] = (path[i][3] + path[i+1][3]) / 2; //average max speed of the 2 points
            pursuitPoint[4] = (path[i+1][4]);
			
            pursuitPoint[0] += position[0]; //Convert from local to global frame
            pursuitPoint[1] += position[1]; //Convert from local to global frame
            
            if (config::debugPursuit) {
                printf("\nFound on segment %d (sol1): (%.3f, %.3f)", i, pursuitPoint[0], pursuitPoint[1]);
            }
            foundSolution = true;
        }
        else if (!solution1InLimit && solution2InLimit) { //if solution2 is in limit and solution 1 isn't
            pursuitPoint[0] = solutionsLocal[1][0]; // Set the local X value for pursuitPoint
            pursuitPoint[1] = solutionsLocal[1][1]; // Set the local Y value for pursuitPoint
            pursuitPoint[2] = (path[i][2] + path[i + 1][2]) / 2; //average max speed of the 2 points
            pursuitPoint[3] = (path[i][3] + path[i+1][3]) / 2; //average max speed of the 2 points
            pursuitPoint[4] = (path[i+1][4]);
            
            pursuitPoint[0] += position[0];//Convert from local to global frame
            pursuitPoint[1] += position[1];//Convert from local to global frame
            
            if (config::debugPursuit) {
                printf("\nFound on segment %d (sol2): (%.3f, %.3f)", i, pursuitPoint[0], pursuitPoint[1]);
            }

            foundSolution = true;
        }

        if((distance(pursuitPoint, path[i+1]) < distance(position, path[i+1])) && foundSolution){ // if pursuitPoint is closer to next point in path than robot
            startIndex = i;
            startIndex = startIndex;
            return pursuitPoint;
        }
        else if(foundSolution){
            startIndex += 1;
        }
        else{
            pursuitPoint[0] = path[startIndex][0];
            pursuitPoint[1] = path[startIndex][1];
            pursuitPoint[2] = path[startIndex][2];
            pursuitPoint[3] = path[startIndex][3];
            pursuitPoint[4] = path[startIndex][4];
        }
        
    } // end of for loop

    return pursuitPoint;
}