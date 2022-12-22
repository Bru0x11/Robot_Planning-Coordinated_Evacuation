#include "coordination.h"

int main(){

    std::vector<Intersection> allIntersections {};
    allIntersections.push_back(getPathIntersection(robotPath1, robotPath2));
    allIntersections.push_back(getPathIntersection(robotPath1, robotPath3));
    allIntersections.push_back(getPathIntersection(robotPath2, robotPath3));

    


    return 0;
}