#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "src/visilibity.hpp"
#include <vector>

struct Intersection{
    VisiLibity::Point point {};
    double timeRobot1 {};
    double timeRobot2 {};
};

double travelTime(VisiLibity::Point& point, VisiLibity::Polyline& robotPath, double costantSpeed=0.3);

bool isWithinRange(double x1, double y1, double x2, double y2, double granularity);

VisiLibity::Point getIntersectionPoint(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, double granularity);

Intersection getPathIntersection(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2);

#endif