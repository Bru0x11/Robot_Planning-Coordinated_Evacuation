#include "point.h"
#include "square.h"
#include "triangle.h"
#include "obstacles.h"
#include "contour.h"
#include "gate"
#include<list> 

#ifndef MAP_H
#define MAP_H

struct Map{

    Contour contour;
    Obstacles obstacles;
    Gate gate;
 
}

#endif