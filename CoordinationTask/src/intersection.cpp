#include "../include/intersection.h"

double getTravelTime(VisiLibity::Polyline& robotPath, double costantSpeed){
    double pathLength {robotPath.length()};
    double travelTime {pathLength/costantSpeed};
    return travelTime;
}


bool arePointsNear(double x1, double y1, double x2, double y2, double distanceThreshold) {
    double distance = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    //std::cout<<"Distance: "<<distance<<std::endl;
    return distance <= distanceThreshold;
}


std::vector<VisiLibity::Polyline> getIntersectionPoint(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, double distanceThreshold){
    //int lengthOfVector = std::min(robotPath1.size(),robotPath2.size()); 
    // std::cout<<"robotPath1.size() "<<robotPath1.size()<<std::endl;
    // std::cout<<"robotPath2.size(): "<<robotPath2.size()<<std::endl;
    //std::cout<<"Length: "<<lengthOfVector<<std::endl;

    std::vector<VisiLibity::Polyline> segments;
    VisiLibity::Polyline segmentRobot1;
    VisiLibity::Polyline segmentRobot2;

    for (int i=0; i<robotPath1.size(); i++){
        segmentRobot1.push_back(VisiLibity::Point(robotPath1[i].x(), robotPath1[i].y()));
        
        for(int j=0; j<robotPath2.size();j++){
           
            if (arePointsNear(robotPath1[i].x(), robotPath1[i].y(), robotPath2[j].x(), robotPath2[j].y(), distanceThreshold)){
                for(int k=0; k<j; k++){
                    segmentRobot2.push_back(VisiLibity::Point(robotPath2[k].x(), robotPath2[k].y()));
                }
                // std::cout<<"intersection point robot1: "<<VisiLibity::Point(robotPath1[i].x(), robotPath1[i].y())<<std::endl;
                // std::cout<<"intersection point robot2: "<<VisiLibity::Point(robotPath2[j].x(), robotPath2[j].y())<<std::endl;
                segments.push_back(robotPath1);
                segments.push_back(robotPath2);
                
                return segments;
            }    
        }
    }
    segments.push_back(robotPath1);
    segments.push_back(robotPath2);
    return segments;
}

Intersection getPathIntersection(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2){
    double robotSize = 0.5;
    double rho = 0.3;
    //double distanceThreshold = robotSize + rho;
    double distanceThreshold = 0.5;
    
    double costantSpeed = 0.3;
    std::vector<VisiLibity::Polyline> segments = getIntersectionPoint(robotPath1, robotPath2, distanceThreshold);

    // std::cout<<"robot1 lenght : "<<segments[0].length()<<std::endl;
    // std::cout<<"robot2 lenght : "<<segments[1].length()<<std::endl;
    double travelTimeRobot1 {getTravelTime(segments[0], costantSpeed)};
    // std::cout<<"Travel time 1: "<<travelTimeRobot1<<std::endl;

    double travelTimeRobot2 {getTravelTime(segments[1], costantSpeed)};
    // std::cout<<"Travel time 2: "<<travelTimeRobot2<<std::endl;

    Intersection result {travelTimeRobot1, travelTimeRobot2};
    return result;    
}

double myAbs(double x){
    if(x>0){return x;}
    else{return -x;} 
}
