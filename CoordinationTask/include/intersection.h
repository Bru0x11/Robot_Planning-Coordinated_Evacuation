#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "src/visilibity.hpp"
#include <vector>
#include <cmath>

/*
Struct representing a intersection.
For each robot, we define the travel time to reach the intersection point
*/
struct Intersection{
    double timeRobot1 {};
    double timeRobot2 {};
};

/*
Obtain the travel time for a certain segment of the path
*/
double getTravelTime(VisiLibity::Polyline& robotPath, double costantSpeed=0.3);

/*
Check whether two point in a path are near each other by taking into consideration a distance threshold
*/
bool arePointsNear(double x1, double y1, double x2, double y2, double distanceThreshold);

/*
Obtain the intersection between two paths.
Return a list with the two segments of paths that reach the intersection point
*/
std::vector<VisiLibity::Polyline> getIntersectionPoint(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, double distanceThreshold);

/*
Compute the travel time for two interesecting paths
*/
Intersection getPathIntersection(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2);

double myAbs(double x);

#endif