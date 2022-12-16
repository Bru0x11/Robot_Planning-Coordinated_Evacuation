#include "offsetFunctions.h"

auto createPolygon(const std::vector<VisiLibity::Point>& points){
  PathsD polygon;
  std::string stringPoint{};

  for (VisiLibity::Point point : points){ //need to format the point in the right way to create the path
    stringPoint.append(std::to_string(point.x()));
    stringPoint.append(",");
    stringPoint.append(std::to_string(point.y()));
    stringPoint.append(", ");
  }
  
  stringPoint.erase(stringPoint.size() - 2);

  polygon.push_back(MakePathD(stringPoint));
  return polygon;
}

auto offsetPolygon(const ::PathsD& polygon, float offsetSize, bool isMapContour=false){
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

auto translatePolygon(const PathsD& originalPolygon){
  std::vector<VisiLibity::Point> listOfPoints {};

  for (PointD point : originalPolygon[0]){
    listOfPoints.push_back(VisiLibity::Point(point.x, point.y));
  }

  return listOfPoints;
}

//FOR PRINTING PURPOSES
void System(const std::string& filename){
#ifdef _WIN32
  system(filename.c_str());
#else
  system(("firefox " + filename).c_str());
#endif
}


//Example of print
int main(){

  std::vector<VisiLibity::Point> mapPoints {};
  mapPoints.push_back(VisiLibity::Point(0.0, 0.0));
  mapPoints.push_back(VisiLibity::Point(20.0, 0.0));
  mapPoints.push_back(VisiLibity::Point(20.0, 20.0));
  mapPoints.push_back(VisiLibity::Point(0.0, 20.0));


  std::vector<VisiLibity::Point> trianglePoints {};
  trianglePoints.push_back(VisiLibity::Point(6.0, 7.0));
  trianglePoints.push_back(VisiLibity::Point(1.0, 2.0));
  trianglePoints.push_back(VisiLibity::Point(6.0, 2.0));
  
  std::vector<VisiLibity::Point> squarePoints {};
  squarePoints.push_back(VisiLibity::Point(2.0, 9.0));
  squarePoints.push_back(VisiLibity::Point(2.0, 14.0));
  squarePoints.push_back(VisiLibity::Point(8.0, 14.0));
  squarePoints.push_back(VisiLibity::Point(8.0, 9.0));


  PathsD triangle {createPolygon(trianglePoints)};
  PathsD square {createPolygon(squarePoints)};
  PathsD map {offsetPolygon(createPolygon(mapPoints), 2.0, true)};


  FillRule fr = FillRule::EvenOdd;
  SvgWriter svg;
  svg.AddPaths(triangle, false, fr, 0x100066FF, 0x400066FF, 1, false);
  svg.AddPaths(square, false, fr, 0x10AA66FF, 0xAA0066FF, 1, false);
  svg.AddPaths(map, false, fr, 0x10FF66FF, 0xFF0066FF, 1, false);
  svg.SaveToFile("triangle_try.svg", 800, 600, 0);
  System("triangle_try.svg");
}
