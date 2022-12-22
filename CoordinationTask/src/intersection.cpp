#include "intersection.h"

double getTravelTime(VisiLibity::Point point, VisiLibity::Polyline robotPath, double costantSpeed){
    double pathLength {robotPath.lenght()};
    double travelTime {pathLength*costantSpeed};
    return travelTime;
}

bool isWithinRange(double x1, double y1, double x2, double y2, double granularity){
    
}

VisiLibity::Point getIntersectionPoint(Visilibity::Polyline robotPath1, VisiLibity::Polyline robotPath2, double granularity){
    robotPath1[0]
}

Intersection getPathIntersection(Visilibity::Polyline robotPath1, VisiLibity::Polyline robotPath2){
    VisiLibity::Point intersectionPoint {getIntersectionPoint(robotPath1, robotPath2)};
    double travelTimeRobot1 {getTravelTime(intersectionPoint, robotPath1)};
    double travelTimeRobot2 {getTravelTime(intersectionPoint, robotPath2)};

    Intesection result {intersectionPoint, travelTimeRobot1, travelTimeRobot2};
    return result;    
}
