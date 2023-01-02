#ifndef INTERPOLATION_H
#define INTERPOLATION_H

//#include "src/visilibity.hpp"
#include "../../VisiLibity1/src/visilibity.hpp"

#include <vector>
#include <iostream>
#include <cfloat>

#include "dubins.h"
//#include "include/dubins.h"

#include "math.h"
#include <fstream>

using namespace VisiLibity;
using namespace std;

class Vector
{
public:
    double x;
    double y;

    Vector(double x, double y);
   
    double norm();
};

class Line{
    public:
    double a;
    double b;
    double c;
    double m;
    double q;  
    bool vertical = false;
    bool horizontal = false;

    friend ostream & operator << (ostream &out, Line l);

    Line(double a, double b, double c);    
    
    Line(double m, double q);

    Line find_parallel(double distance);
        
    Line find_perpendicular(Point p);

    static Line get_line_from_points(Point p1, Point p2);
        
    static double get_q_from_point(Point p, double m);
        
    static double compute_m(Point p0, Point p1);

    static Line get_line_from_m_and_p(Point p, double m);

        
};

Point find_lines_intersection(Line l1, Line l2);

Point find_entrance(Point a, Point b, Point c, double minR);

Point find_exit(Point a, Point b, Point c, double minR);

Polyline get_points_from_arc(Arc a, int npts);
    
Polyline get_points_from_curve(Curve c, int npts);

//TODO: IF X1>X0
Polyline get_points_line(Point p0, Point p1);

double compute_arc_length(Point a, Point b, double minR);

double compute_angle(Point a, Point b);
 
Arc get_arc(Point entrance, Point exit, double angle_entrance, double angle_exit, double minR);

Environment get_environment1();

Environment get_environment2();

Environment get_environment3();

Curve get_first_trait_dubins(Polyline shortest_path, double th0, double minR);

Curve get_last_trait_dubins(Polyline shortest_path, double thf, double minR);

vector<Arc> get_arcs_interpolation(Polyline shortest_path, double minR);

Polyline interpolation(Polyline shortest_path, double th0, double thf, double minR);

#endif