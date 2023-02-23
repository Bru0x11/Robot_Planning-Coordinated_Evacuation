#ifndef COORDINATION_H
#define COORDINATION_H

#include "intersection.h"
#include <cstdlib>
#include <vector>

/*
Struct that represents the delta between two robot paths
*/
struct DeltaTime{
    double delta {};
    int fasterRobot {};
};

/*
Struct that indicates the delay amount that has to be applied to a robot in order to perform the coordination task
*/
struct RobotInitialization{
    int robot {};
    double delay {};
};

/*
Given all the three paths, returns the actuation order together with the delay of every robot
*/
std::vector<RobotInitialization> coordination(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, VisiLibity::Polyline& robotPath3);

#endif