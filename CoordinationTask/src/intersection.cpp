#include "../include/intersection.h"

double getTravelTime(VisiLibity::Polyline& robotPath, double costantSpeed){
    double pathLength {robotPath.length()};
    double travelTime {pathLength/costantSpeed};
    return travelTime;
}


bool arePointsNear(double x1, double y1, double x2, double y2, double distanceThreshold) {
    double distance = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    return distance <= distanceThreshold;
}


std::vector<VisiLibity::Polyline> getIntersectionPoint(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, double distanceThreshold){
    int lengthOfVector = std::min(robotPath1.size(),robotPath2.size()); 

    std::vector<VisiLibity::Polyline> segments;
    VisiLibity::Polyline segmentRobot1;
    VisiLibity::Polyline segmentRobot2;

    for (int i=0; i<lengthOfVector; i++){
        segmentRobot1.push_back(VisiLibity::Point(robotPath1[i].x(), robotPath1[i].y()));
        segmentRobot2.push_back(VisiLibity::Point(robotPath2[i].x(), robotPath2[i].y()));
        if (arePointsNear(robotPath1[i].x(), robotPath1[i].y(), robotPath2[i].x(), robotPath2[i].y(), distanceThreshold)){
            segments.push_back(segmentRobot1);
            segments.push_back(segmentRobot2);
            return segments;
         }
    }
}

Intersection getPathIntersection(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2){
    double robotSize = 0.5;
    double rho = 0.1;
    double distanceThreshold = robotSize + rho;
    
    double costantSpeed = 0.3;
    std::vector<VisiLibity::Polyline> segments = getIntersectionPoint(robotPath1, robotPath2, distanceThreshold);


    double travelTimeRobot1 {getTravelTime(segments[0], costantSpeed)};
    std::cout<<"Travel time 1: "<<travelTimeRobot1<<std::endl;

    double travelTimeRobot2 {getTravelTime(segments[1], costantSpeed)};
    std::cout<<"Travel time 2: "<<travelTimeRobot2<<std::endl;

    Intersection result {travelTimeRobot1, travelTimeRobot2};
    return result;    
}

double myAbs(double x){
    if(x>0){return x;}
    else{return -x;} 
}
