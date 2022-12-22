#ifndef COORDINATION_H
#define COORDINATION_H

#include "intersection.h"
#include <cstdlib>


struct DeltaTime{
    double delta {};
    int fasterRobot {};
};

struct RobotInitialization{
    int robot {};
    int delay {};
};

std::vector<RobotInitialization> coordination(Visilibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, VisiLibity::Polyline& robotPath3)


#endif