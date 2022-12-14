#ifndef OFFSETFUNCTIONS_H
#define OFFSETFUNCTIONS_H

struct Point{
    float x{};
    float y{};
    float z{};
};

/*
Given a vector of Point, creates a polygon. The points have to be listed in a clockwise order (from bottom_left onwards).
Returns a list of Point that describe the polygon.
*/
auto createPolygon(const std::vector<::Point>& points);

/*
Given a polygon and a value, offsets the polygon by that amount. If the parameter isMapContour is set to true, return the frame representing the
contour of map. Returns the list of Point that define the new polygon.
*/
auto offsetPolygon(const PathsD& polygon, float offsetSize, bool isMapContour);

/*
Given two polygons, returns true if they are intersected.
*/
auto AreIntersected(const PathsD& firstPolygon, const PathsD& secondPolygon);

/*
Given two polygons, merge the two together (has to be used after checking the intersection between the two). Returns the list of Point creating
the resulting polygon.
*/
auto mergePolygons(const PathsD& firstPolygon, const PathsD& secondPolygon);

#endif