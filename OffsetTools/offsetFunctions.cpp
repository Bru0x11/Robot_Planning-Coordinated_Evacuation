#include "CPP/Clipper2Lib/include/clipper2/clipper.h"
#include "CPP/Utils/clipper.svg.h"
#include "CPP/Utils/clipper.svg.utils.h"

#include "offsetFunctions.h"

#include <vector>
#include <string>

using namespace Clipper2Lib;

auto createPolygon(const std::vector<Point>& points){
  PathsD polygon;
  std::string stringPoint{};

  for (Point point : points){ //need to format the point in the right way to create the path
    stringPoint.append(std::to_string(point.x));
    stringPoint.append(",");
    stringPoint.append(std::to_string(point.y));
    stringPoint.append(", ");
  }
  
  stringPoint.erase(stringPoint.size() - 2);

  polygon.push_back(MakePathD(stringPoint));
  return polygon;
}

auto offsetPolygon(const PathsD& polygon, float offsetSize, bool isMapContour=false){
  PathsD offsettedPolygon;

  offsettedPolygon = InflatePaths(polygon, offsetSize, JoinType::Miter, EndType::Polygon);
  
  if (isMapContour){
    offsettedPolygon = Xor(offsettedPolygon, polygon, FillRule::NonZero);
  }

  return offsettedPolygon;
}

auto AreIntersected(const PathsD& firstPolygon, const PathsD& secondPolygon){
  auto intersection = Intersect(firstPolygon, secondPolygon, FillRule::NonZero);
  return ((intersection.size() > 0) ? true : false);
}

auto mergePolygons(const PathsD& firstPolygon, const PathsD& secondPolygon){
  return Union(firstPolygon, secondPolygon, FillRule::NonZero);
}

/* FOR PRINTING PURPOSES
void System(const std::string& filename){
#ifdef _WIN32
  system(filename.c_str());
#else
  system(("firefox " + filename).c_str());
#endif
}

void printPolygons(){
  FillRule fr = FillRule::EvenOdd;
  SvgWriter svg;
  svg.AddPaths(name_of_polygon, false, fr, 0x100066FF, 0x400066FF, 1, false);
  svg.SaveToFile("prova.svg", 800, 600, 0);
  System("prova.svg");
}*/
