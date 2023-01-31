#ifndef INTERPOLATION_H
#define INTERPOLATION_H

//#include "src/visilibity.hpp"
#include "../../VisiLibity1/src/visilibity.hpp"

#include "include/offsetFunctions.h"

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
    static double dot(Vector v1, Vector v2);
    static double angle(Vector v1, Vector v2);
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
        
    Line find_perpendicular(VisiLibity::Point p);

    static Line get_line_from_points(VisiLibity::Point p1, VisiLibity::Point p2);
        
    static double get_q_from_point(VisiLibity::Point p, double m);
        
    static double compute_m(VisiLibity::Point p0, VisiLibity::Point p1);

    static Line get_line_from_m_and_p(VisiLibity::Point p, double m);

        
};

VisiLibity::Point find_lines_intersection(Line l1, Line l2);

VisiLibity::Point find_entrance(VisiLibity::Point a, VisiLibity::Point b, VisiLibity::Point c, double minR);

VisiLibity::Point find_exit(VisiLibity::Point a, VisiLibity::Point b, VisiLibity::Point c, double minR);

Polyline get_points_from_arc(Arc a, int npts);
    
Polyline get_points_from_curve(Curve c, int npts);

//TODO: IF X1>X0
Polyline get_points_line(VisiLibity::Point p0, VisiLibity::Point p1);

double compute_arc_length(VisiLibity::Point a, VisiLibity::Point b, double minR);

double compute_angle(VisiLibity::Point a, VisiLibity::Point b);
 
Arc get_arc(VisiLibity::Point entrance, VisiLibity::Point exit, double angle_entrance, double angle_exit, double minR);

Environment get_environment1();

Environment get_environment2();

Environment get_environment3();

Environment get_env_offset(Environment env, double minR, double minH);

Curve get_first_trait_dubins(Polyline shortest_path, double th0, double minR);

Curve get_last_trait_dubins(Polyline shortest_path, double thf, double minR);

vector<Arc> get_arcs_interpolation(Polyline shortest_path, double minR);

double find_min_angle(Polygon points);

double offset_calculator(double minAngle, double minR, double minH);

int env_holes(Environment env);

Polyline interpolation(Polyline shortest_path, double th0, double thf, double minR);

#endif