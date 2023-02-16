#include "../include/intersection.h"

double getTravelTime(VisiLibity::Point& point, VisiLibity::Polyline& robotPath, double costantSpeed){
    double pathLength {robotPath.length()};
    double travelTime {pathLength*costantSpeed};
    return travelTime;
}

bool isWithinRange(double x1, double y1, double x2, double y2, double granularity){
    return ((((x1 >= x2-granularity) && (x1 <= x2+granularity)) && ((y1 >= y2-granularity) && (y1 <= y2+granularity))) ? true : false);
}

VisiLibity::Point getIntersectionPoint(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, double granularity){
    int lengthOfVector {robotPath1.size()};

    for (int i=0; i<lengthOfVector; i++){
        if (isWithinRange(robotPath1[i].x(), robotPath1[i].y(), robotPath2[i].x(), robotPath2[i].y(), granularity)){
            return VisiLibity::Point {robotPath1[i]};
        }
    }
}

Intersection getPathIntersection(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2){
    double granularity = 0.3;
    double costantSpeed = 2;
    VisiLibity::Point intersectionPoint {getIntersectionPoint(robotPath1, robotPath2, granularity)};
    double travelTimeRobot1 {getTravelTime(intersectionPoint, robotPath1, costantSpeed)};
    double travelTimeRobot2 {getTravelTime(intersectionPoint, robotPath2, costantSpeed)};

    Intersection result {intersectionPoint, travelTimeRobot1, travelTimeRobot2};
    return result;    
}
