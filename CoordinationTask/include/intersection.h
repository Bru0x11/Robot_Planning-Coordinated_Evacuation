#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "src/visilibity.hpp"
#include <vector>
#include <cmath>

struct Intersection{
    double timeRobot1 {};
    double timeRobot2 {};
};

double getTravelTime(VisiLibity::Polyline& robotPath, double costantSpeed=0.3);

bool arePointsNear(double x1, double y1, double x2, double y2, double distanceThreshold);

std::vector<VisiLibity::Polyline> getIntersectionPoint(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, double distanceThreshold);

Intersection getPathIntersection(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2);

double myAbs(double x);

#endif