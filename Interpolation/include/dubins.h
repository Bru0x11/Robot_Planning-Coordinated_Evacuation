#ifndef DUBINS_H
#define DUBINS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std::chrono_literals;
//#include "src/visilibity.hpp"
#include "../../VisiLibity1/src/visilibity.hpp"

#include <iostream>
#include <cstdlib>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

using namespace std;

// Create a structure representing an arc of a Dubins curve (straight or
// circular)
struct Arc{
    double x0;
    double y0;
    double th0;
    double k;
    double L;
    double xf;
    double yf;
    double thf;
};


//Create a structure representing a Dubins curve (composed by three arcs)
struct Curve{
    //three arcs
    Arc a1;
    Arc a2;
    Arc a3;
    double L;

};


//TODO: change name pos
struct Pos{
    double x;
    double y;
    double th;
};

struct Solution{
    bool ok;
    double sc_s1;
    double sc_s2; 
    double sc_s3;
};

//Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t
//otherwise
double sinc(double t);

//Normalize an angle (in range [0,2*pi))
double mod2pi(double ang);

//Normalize an angular difference (range (-pi, pi])
double rangeSymm(double ang);


//Check validity of a solution by evaluating explicitly the 3 equations 
//defining a Dubins problem (in standard form)
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf);

/////////////////////////////////////////////////////////////////////
//Functions to scale and solve Dubins problems
/////////////////////////////////////////////////////////////////////

//Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
//return a double[] array containing [sc_th0, sc_thf, sc_Kmax, lambda]
double* scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

//Scale the solution to the standard problem back to the original problem
//return a pointer of double containing {s1, s2, s3}
double* scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3);

//LSL
Solution LSL(double sc_th0, double sc_thf, double sc_Kmax);

//RSR
Solution RSR(double sc_th0, double sc_thf, double sc_Kmax);

//LSR
Solution LSR(double sc_th0, double sc_thf, double sc_Kmax);

//RSL
Solution RSL(double sc_th0, double sc_thf, double sc_Kmax);


//RLR
Solution RLR(double sc_th0, double sc_thf, double sc_Kmax);


//LRL
Solution LRL(double sc_th0, double sc_thf, double sc_Kmax);

//Evaluate an arc (circular or straight) composing a Dubins curve, at a 
//given arc-length s

Pos circline(double s, double x0, double y0, double th0, double k);


//Create a structure representing an arc of a Dubins curve (straight or
//circular)
Arc dubinsarc(double x0, double y0, double th0, double k, double L);

Curve dubinscurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2);

struct Type_curve_sol;

//Solve the Dubins problem for the given input parameters.
//Return the type and the parameters of the optimal curve
Curve dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

vector<Curve> multipoint_dubins(VisiLibity::Polyline points, double Kmax);

#endif