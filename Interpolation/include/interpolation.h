#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "../../VisiLibity1/src/visilibity.hpp"
#include "include/offsetFunctions.h"
#include "dubins.h"
#include "math.h"

#include <vector>
#include <iostream>
#include <cfloat>
#include <fstream>

using namespace VisiLibity;
using namespace std;

/*
Class that represents a Vector of two coordinates.
Also these methods are defined:
- constructur -> builds the object Vector.
- norm -> returns the norm.
- dot -> performs dot product between two Vectors.
- angle -> computes the angle between two Vectors.
*/
class Vector{
    public:
    double x;
    double y;

    Vector(double x, double y);
   
    double norm();
    static double dot(Vector v1, Vector v2);
    static double angle(Vector v1, Vector v2);
};


/*
Class that represents a Line.
*/
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

    Line findParallel(double distance);
        
    Line findPerpendicular(VisiLibity::Point point);

    static Line getLineFromPoints(VisiLibity::Point point1, VisiLibity::Point point2);
        
    static double getQFromPoint(VisiLibity::Point point, double m);
        
    static double computeM(VisiLibity::Point point0, VisiLibity::Point point1);

    static Line getLineFromMAndP(VisiLibity::Point point, double m);    
};

/*
Finds the intersection between two Lines.
*/
VisiLibity::Point findLinesIntersection(Line line1, Line line2);

/*
Find the distance given three Points.
*/
double findDistance(VisiLibity::Point point0,VisiLibity::Point point1, VisiLibity::Point point2, double minimumCurvatureRadius);

/*
Given a initial and final point of the segment and a distance value, compute the entrance point.
*/
VisiLibity::Point findEntrance(VisiLibity::Point point0, VisiLibity::Point point1, double minimumCurvatureRadius);

/*
Given a initial and final point of the segment and a distance value, compute the exit point.
*/
VisiLibity::Point findExit(VisiLibity::Point point0, VisiLibity::Point point1, double minimumCurvatureRadius);

/*
Given an Arc, retrive its points.
*/
Polyline getPointsFromArc(Arc arc, int npts);

/*
Given a Curve, retrive its points.
*/
Polyline getPointsFromCurve(Curve curve, int npts);

/*
Given a Line, retrive its points.
*/
Polyline getPointsLine(VisiLibity::Point p0, VisiLibity::Point p1);

/*
Given two Points, compute the lenght of the Arc created by them.
*/
double computeArcLength(VisiLibity::Point point0, VisiLibity::Point point1, double minimumCurvatureRadius);

/*
Given two Points, compute angle between two Vectors.
*/
double computeAngle(VisiLibity::Point point0, VisiLibity::Point point1);

/*
Given the entry and exit point plus the related angles, retrieve the Arc.
*/
Arc getArc(VisiLibity::Point entrance, VisiLibity::Point exit, double angleEntrance, double angleExit, double minimumCurvatureRadius);

/*
Build a predefined environment.
This function can be used to create different scenarios as examples.
*/
Environment getEnvironment();

/*
Given an envirnoment, offsetts and merges all the polygons within it.
*/
Environment getOffsettedEnvironment(Environment environment, double minimumCurvatureRadius, double robotSize);

/*
Compute the first trait of the robot trajectory using Dubins curves.
*/
Curve getFirstTraitDubins(Polyline shortestPath, double theta0, double minimumCurvatureRadius);

/*
Compute the last of the robot trajectory using Dubins curves.
*/
Curve getLastTraitDubins(Polyline shortestPath, double thetaf, double minimumCurvatureRadius);

/*
Given a polygon, finds the minimum angle.
*/
double findMinAngle(Polygon points);

/*
Compute the offset that has to be applied to a certain polygon.
*/
double offsetCalculator(double minAngle, double minimumCurvatureRadius, double robotSize);

/*
For printing purposes. Prints the minimum angle and related offset of evey polygon in the environment.
*/
int environmentHoles(Environment environment);

/*
Create the interpolation to pass by the polygon without touching them and without using Dubins curves.
*/
vector<Arc> getArcsInterpolation(Polyline shortestPath, double minimumCurvatureRadius);

/*
Perform the interpolation of the entire shortest path
*/
Polyline interpolation(Polyline shortestPath, double theta0, double thetaF, double minimumCurvatureRadius);

#endif