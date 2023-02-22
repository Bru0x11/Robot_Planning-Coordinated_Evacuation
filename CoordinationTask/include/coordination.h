#ifndef COORDINATION_H
#define COORDINATION_H

#include "intersection.h"
#include <cstdlib>
#include <vector>


struct DeltaTime{
    double delta {};
    int fasterRobot {};
};

struct RobotInitialization{
    int robot {};
    double delay {};
};

std::vector<RobotInitialization> coordination(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, VisiLibity::Polyline& robotPath3);


#endif