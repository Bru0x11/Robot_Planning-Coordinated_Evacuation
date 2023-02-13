#include "../include/offsetFunctions.h"

PathsD createPolygon(const std::vector<VisiLibity::Point>& points){
  PathsD polygon;
  std::string stringPoint{};

  //Need to format the point in the right way in order to create the path
  for (VisiLibity::Point point : points){ 
    stringPoint.append(std::to_string(point.x()));
    stringPoint.append(",");
    stringPoint.append(std::to_string(point.y()));
    stringPoint.append(", ");
  }
  
  stringPoint.erase(stringPoint.size() - 2);

  polygon.push_back(MakePathD(stringPoint));
  return polygon;
}

PathsD offsetPolygon(const ::PathsD& polygon, float offsetSize, bool isMapContour=false){
  PathsD offsettedPolygon{};

  if(isMapContour){
    offsettedPolygon = InflatePaths(polygon, offsetSize, JoinType::Miter, EndType::Polygon);
    offsettedPolygon = Xor(offsettedPolygon, polygon, FillRule::NonZero);
  }
  else{
    offsettedPolygon = InflatePaths(polygon, offsetSize, JoinType::Miter, EndType::Polygon);
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

void checkIntersections(const PathsD& newPolygon, std::vector<PathsD>& previousPolygons){
  checkIntersectionsRec(newPolygon, previousPolygons, 0);
}

void checkIntersectionsRec(const PathsD& newPolygon, std::vector<PathsD>& previousPolygons, int i){
  PathsD mergedPoly{};

  if((previousPolygons.size() != 0) && (i < previousPolygons.size())){
    PathsD ithPoly = previousPolygons.at(i);

    if(AreIntersected(newPolygon, ithPoly)){
      mergedPoly = mergePolygons(newPolygon, ithPoly);
      previousPolygons.erase(previousPolygons.begin() + i);
      checkIntersections(mergedPoly, previousPolygons);
    }else{
      checkIntersectionsRec(newPolygon, previousPolygons, i+1);
    }

  }else{
      previousPolygons.push_back(newPolygon);
  }
}

std::vector<VisiLibity::Point> translatePolygon(const PathsD& originalPolygon){
  std::vector<VisiLibity::Point> listOfPoints{};

  for (PointD point : originalPolygon[0]){
    listOfPoints.push_back(VisiLibity::Point(point.x, point.y));
  }

  return listOfPoints;
}

void System(const std::string& filename){
#ifdef _WIN32
  system(filename.c_str());
#else
  system(("firefox " + filename).c_str());
#endif
}