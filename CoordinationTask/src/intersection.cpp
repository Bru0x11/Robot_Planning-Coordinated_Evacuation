#include "../include/intersection.h"

double getTravelTime(VisiLibity::Point& point, VisiLibity::Polyline& robotPath, double costantSpeed){
    double pathLength {robotPath.length()};
    double travelTime {pathLength*costantSpeed};
    return travelTime;
}

bool isWithinRange(double x1, double y1, double x2, double y2, double granularity){
    return ((((x1 >= x2-granularity) && (x1 <= x2+granularity)) && ((y1 >= y2-granularity) && (y1 <= y2+granularity))) ? true : false);
}


bool arePointsNear(double x1, double y1, double x2, double y2, double distanceThreshold) {
    double distance = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    return distance <= distanceThreshold;
}


VisiLibity::Point getIntersectionPoint(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, double granularity){
    int lengthOfVector {robotPath1.size()};
    double distanceThreshold = 0.5;

    for (int i=0; i<lengthOfVector; i++){
        // if (isWithinRange(robotPath1[i].x(), robotPath1[i].y(), robotPath2[i].x(), robotPath2[i].y(), granularity)){
        //     return VisiLibity::Point {robotPath1[i]};
        // }
        if (arePointsNear(robotPath1[i].x(), robotPath1[i].y(), robotPath2[i].x(), robotPath2[i].y(), distanceThreshold)){
             return VisiLibity::Point {robotPath1[i]};
         }

    }
}

Intersection getPathIntersection(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2){
    double granularity = 0.3;
    double costantSpeed = 1;
    VisiLibity::Point intersectionPoint {getIntersectionPoint(robotPath1, robotPath2, granularity)};
    std::cout<<"the intersection point is at: "<<intersectionPoint<<std::endl;
    double travelTimeRobot1 {getTravelTime(intersectionPoint, robotPath1, costantSpeed)};
    std::cout<<"Travel time 1: "<<travelTimeRobot1<<std::endl;

    double travelTimeRobot2 {getTravelTime(intersectionPoint, robotPath2, costantSpeed)};
    std::cout<<"Travel time 2: "<<travelTimeRobot2<<std::endl;

    Intersection result {intersectionPoint, travelTimeRobot1, travelTimeRobot2};
    return result;    
}

double myAbs(double x){
    if(x>0){return x;}
    else{return -x;} 
}
