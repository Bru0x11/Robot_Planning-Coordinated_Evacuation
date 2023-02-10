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
Returns a list of Point that describe the polygon.
*/
PathsD createPolygon(const std::vector<VisiLibity::Point>& points);

/*
Given a polygon and a value, offsets the polygon by that amount. If the parameter isMapContour is set to true, return the frame representing the
contour of map. Returns the list of Point that define the new polygon.
*/
PathsD offsetPolygon(const PathsD& polygon, float offsetSize, bool isMapContour);

/*
Given two polygons, returns true if they are intersected.
*/
auto AreIntersected(const PathsD& firstPolygon, const PathsD& secondPolygon);

/*
Given two polygons, merge the two together (has to be used after checking the intersection between the two). Returns the list of Point creating
the resulting polygon.
*/
auto mergePolygons(const PathsD& firstPolygon, const PathsD& secondPolygon);


void checkIntersections(const PathsD& newPolygon, std::vector<PathsD>& previousPolygons, int from = 0);

/*
Given a PathD, returns list of VisiLibity::Point
*/
std::vector<VisiLibity::Point> translatePolygon(const PathsD& originalPolygon);

void System(const std::string& filename);




#endif