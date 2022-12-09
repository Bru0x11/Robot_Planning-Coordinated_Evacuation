#include "point.h"
#include "square.h"
#include "triangle.h"
#include<list> 

#ifndef OBSTACLES_H
#define OBSTACLES_H

struct Obstacles{

    list<Triangle> triangles;
    list<Square> squares;
 
}

#endif