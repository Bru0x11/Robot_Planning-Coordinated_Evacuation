#ifndef OFFSETFUNCTIONS_H
#define OFFSETFUNCTIONS_H

#include "CPP/Clipper2Lib/include/clipper2/clipper.h"
#include "CPP/Utils/clipper.svg.h"
#include "CPP/Utils/clipper.svg.utils.h"

#include "src/visilibity.hpp"

#include <vector>
#include <string>

using namespace Clipper2Lib;

/*
Given a vector of Point, creates a polygon. The points have to be listed in a clockwise order (from bottom_left onwards).
Returns a list of Point (PathsD) that describes the polygon.
*/
PathsD createPolygon(const std::vector<VisiLibity::Point>& points);

/*
Given a polygon and a value, offsets the polygon by that amount. If the parameter isMapContour is set to true, return the frame representing the
contour of map. Returns the list of Point (PathsD) that defines the new polygon.
*/
PathsD offsetPolygon(const PathsD& polygon, float offsetSize, bool isMapContour);

/*
Given two polygons, returns true if they are intersected, false otherwise.
*/
auto AreIntersected(const PathsD& firstPolygon, const PathsD& secondPolygon);

/*
Given two polygons, merge the two together. Returns the list of Point (PathsD) creating the resulting polygon.
This method has to be used after checking the intersection between the two polygons.
*/
auto mergePolygons(const PathsD& firstPolygon, const PathsD& secondPolygon);

/*
Given a new polygon, check whether it intersects all the other polygons (map included).
If this is the case, merge the two polygons and eliminate the previous one.
*/
void checkIntersections(const PathsD& newPolygon, std::vector<PathsD>& previousPolygons);

/*
Recursive call of checkIntersections method.
*/
void checkIntersectionsRec(const PathsD& newPolygon, std::vector<PathsD>& previousPolygons, int i=0);

/*
Given a PathD, returns list of VisiLibity::Point.
*/
std::vector<VisiLibity::Point> translatePolygon(const PathsD& originalPolygon);

/*
Method use for printing purposes.
Requires firefox installed.
*/
void System(const std::string& filename);

#endif