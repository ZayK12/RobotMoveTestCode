#include "purePursuit.h"
#include "config.h"
///@author Zayyaan K
///@date 4/25/25
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>




Pursuit::Pursuit(std::vector<std::vector<float>>** p, std::vector<float>* pos, float LKA) : pathPointer(p), positionPointer(pos), lookAhead(LKA), pursuitPoint({ 0, 0, 0 ,0, 0}) {}


void Pursuit::setLookAhead(float newLKA) {
    lookAhead = newLKA;
}
float Pursuit::sign(float number) const {
    if (number == 0) {
        return 1;
    }
    return number / fabsf(number);
}
float Pursuit::distance(std::vector<float> point0, std::vector<float> point1) const {
    return sqrtf((point0[0] - point1[0]) * (point0[0] - point1[0]) + (point0[1] - point1[1]) * (point0[1] - point1[1]));
}
float Pursuit::distanceSquared(std::vector<float> point0, std::vector<float> point1) const {
    return ((point0[0] - point1[0]) * (point0[0] - point1[0]) + (point0[1] - point1[1]) * (point0[1] - point1[1]));
}
float Pursuit::distanceSquared(float pointX, float pointY, std::vector<float> point1) const {
    return ((pointX - point1[0]) * (pointX - point1[0]) + (pointY - point1[1]) * (pointY - point1[1]));
}
float Pursuit::distanceSquared(float pointX, float pointY, float point1X, float point1Y) const {
    return ((pointX - point1X) * (pointX - point1X) + (pointY - point1Y) * (pointY - point1Y));
}
std::vector<float> Pursuit::closestPoint(float pointX, float pointY, std::vector<std::vector<float>> pointSet) const {
    if (distanceSquared(pointX, pointY, pointSet[0]) > distanceSquared(pointX, pointY, pointSet[1])) {
        return pointSet[1];
    }
    return pointSet[0];
}

float Pursuit::findDiscrim(float point0X, float point0Y, float point1X, float point1Y, float dis, float& determen) {

    float* determ = &determen;

    *determ = point0X * point1Y - point0Y * point1X; //x1*y2 - x2*y1

    float Discrim = (lookAhead*lookAhead) * (dis * dis) - (*determ * *determ); //lookAhead^2 * dis^2 - determ^2
    if (Discrim < TOLERANCE && Discrim > -TOLERANCE) {
        Discrim = 0.0; //Corrects for floating point errors
    }
    if (config::debugPursuit) {
        printf("\n Dis: %.3f, Det: %.3f\n", Discrim, *determ);
        //std::cout << "\n Discriminant: " << Discrim << " Determinant: " << *determ << std::endl;
    }
    return Discrim;
}

std::vector<std::vector<float>> Pursuit::hitPoints(float discrim, float determen, float distanceX, float distanceY, float distance) {
    float sqrtDiscrim = sqrt(discrim);
    float distanceSquared = distance*distance;
    std::vector<float> intsec1 = {//Positive Variation
        //X positive value
        (determen * distanceY + sign(distanceY) * distanceX * sqrtDiscrim) / distanceSquared,
        //Y positive value
        (-determen * distanceX + fabsf(distanceY) * sqrtDiscrim) / distanceSquared };
    std::vector<float> intsec2 = {//Negative Variation
        //X negative value
        (determen * distanceY - sign(distanceY) * distanceX * sqrtDiscrim) / distanceSquared,
        //Y negative value
        (-determen * distanceX - fabsf(distanceY) * sqrtDiscrim) / distanceSquared };
    printf("\n Sol 1: X:%.3f, Y:%.3f \n Sol 2: X:%.3f, Y:%.3f", intsec1[0], intsec1[1], intsec2[0], intsec2[1]);
    std::vector <std::vector<float>> intsecs = { intsec1, intsec2 };
    return intsecs;
}

bool Pursuit::inLimit(float startPointX, float startPointY, float endPointX, float endPointY, std::vector<float> sol1) {
    // Check if intersection point lies on the line segment using parametric projection
    float dx = endPointX - startPointX;
    float dy = endPointY - startPointY;
    float lengthSq = dx*dx + dy*dy;
    
    if (lengthSq < 1e-10) {  // Segment is essentially a point
        return false;
    }
    
    // Project sol1 onto the line segment: t = dot(sol1-start, end-start) / |end-start|^2
    float t = ((sol1[0] - startPointX) * dx + (sol1[1] - startPointY) * dy) / lengthSq;
    
    // Accept if t is between 0 and 1 (point lies on the segment)
    // Use a reasonable tolerance for floating point comparison
    if (t >= -TOLERANCE && t <= 1.0 + TOLERANCE) {
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

bool Pursuit::inLimit2(std::vector<float> startPoint, std::vector<float> endPoint, std::vector<float> sol1){
    if (fabsf(distanceSquared(startPoint, sol1) + distanceSquared(sol1, endPoint) - distanceSquared(startPoint, endPoint)) < .05) {
        return true;
    }
    return false;
}
bool Pursuit::inLimit2(float startpointX, float startPointY, float endPointX, float endPointY, std::vector<float> sol1){
    if (fabsf(distanceSquared(startpointX, startPointY, sol1) + distanceSquared(endPointX, endPointY, sol1) - distanceSquared(startpointX, startPointY, endPointX, endPointY)) < .05) {
        float x = fabsf(distanceSquared(startpointX, startPointY, sol1) + distanceSquared(endPointX, endPointY, sol1) - distanceSquared(startpointX, startPointY, endPointX, endPointY));
        printf("%.3f", x);
        return true;
    }
    return false;
}
std::vector<float> Pursuit::updatePursuitPoint() {
    if(config::debugPursuit){
        printf("\nUpdate pursuit point start");
    }
    float DisX;
    float DisY;
    float DisR;
    float discrim;
    float determinant;
    bool foundSolution = false;

    const auto& path = **pathPointer;          // 5/7/25 Derefrences the path vector to increase performance 
    const auto& position = *positionPointer;  // 5/7/25 Derefrences the position vector to increase performance 
 
    if (config::debugPursuit) {
        printf("\nPOS:(%.3f,%.3f)", position[0],position[1]);
        //std::cout << "Robot Position: (" << position[0] << ", " << position[1] << ")" << std::endl;
    }
    float distanceToEnd = distance(path[path.size() - 1], position);
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
        float lPoint0X = path[i][0] - position[0];
        float lPoint0Y = path[i][1] - position[1];
        float lPoint1X = path[i + 1][0] - position[0];
        float lPoint1Y = path[i + 1][1] - position[1];
        //distance between both X values
        DisX = lPoint1X - lPoint0X;
        //distance between both Y values
        DisY = lPoint1Y - lPoint0Y;
        //abs distance between both points
        DisR = sqrtf( (DisX * DisX) + (DisY * DisY) );
        discrim = findDiscrim(lPoint0X, lPoint0Y, lPoint1X, lPoint1Y, DisR, determinant);
        if (discrim < 0) { continue; } // Discrim being 0 means the circle is not on the line, so it continues to the next point.

        //Function to find where it hit the line
        std::vector<std::vector<float>> solutionsLocal = hitPoints(discrim, determinant, DisX, DisY, DisR);


        //Checks if the solutions are within the line segment
        bool solution1InLimit = inLimit(lPoint0X, lPoint0Y, lPoint1X, lPoint1Y, solutionsLocal[0]);
        bool solution2InLimit = inLimit(lPoint0X, lPoint0Y, lPoint1X, lPoint1Y, solutionsLocal[1]);


        //If both are within limits decides what point to go with. Also updates the startIndex variable
        if (solution1InLimit && solution2InLimit) {
            //Always prefer the point closer to the end of the segment to progress forward
            std::vector<float> lclosestPoint = closestPoint(lPoint1X, lPoint1Y, solutionsLocal); // Compare both potential solutions to find whatever one is closest to the end segment point
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

        if((distanceSquared(pursuitPoint, path[i+1]) < distanceSquared(position, path[i+1])) && foundSolution){ // if pursuitPoint is closer to next point in path than robot
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