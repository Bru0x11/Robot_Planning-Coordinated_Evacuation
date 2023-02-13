#include "../include/offsetFunctions.h"

PathsD createPolygon(const std::vector<VisiLibity::Point>& points){
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

PathsD offsetPolygon(const ::PathsD& polygon, float offsetSize, bool isMapContour=false){
  PathsD offsettedPolygon;

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

void checkIntersections(const PathsD& newPolygon, std::vector<PathsD>& previousPolygons, int from){

  PathsD merged_poly {newPolygon};
  for(int i = from; i<previousPolygons.size(); i++){
    PathsD ith_poly = previousPolygons.at(i);
    if(AreIntersected(newPolygon, ith_poly)){
      merged_poly = mergePolygons(newPolygon, ith_poly);

      std::cout << "OTHER CODE BEFORE ELIMINATION:\n";
      for (int k=0; k<previousPolygons.size(); k++){
          std::cout << previousPolygons[k] << '\n';
      }

      previousPolygons.erase(previousPolygons.begin() + i);

      std::cout << "OTHER CODE AFTER ELIMINATION:\n";
      for (int k=0; k<previousPolygons.size(); k++){
          std::cout << previousPolygons[k] << '\n';
      }

      checkIntersections(merged_poly, previousPolygons, i);
      break;
    }
  }

  std::cout << "OTHER CODE BEFORE RETURN:\n";
  for (int k=0; k<previousPolygons.size(); k++){
      std::cout << previousPolygons[k] << '\n';
  }

  previousPolygons.push_back(merged_poly);

  std::cout << "OTHER CODE AFTER RETURN:\n";
  for (int k=0; k<previousPolygons.size(); k++){
      std::cout << previousPolygons[k] << '\n';
  }

}

std::vector<VisiLibity::Point> translatePolygon(const PathsD& originalPolygon){
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
// int main(){

//   std::vector<VisiLibity::Point> mapPoints {};
//   mapPoints.push_back(VisiLibity::Point(-8, 8));
//   mapPoints.push_back(VisiLibity::Point(8, -8));
//   mapPoints.push_back(VisiLibity::Point(8, 8));
//   mapPoints.push_back(VisiLibity::Point(-8, 8));

//   std::vector<VisiLibity::Point> trianglePoints {};
//   trianglePoints.push_back(VisiLibity::Point(-2, 1));
//   trianglePoints.push_back(VisiLibity::Point(-2, 3));
//   trianglePoints.push_back(VisiLibity::Point(4, 3));
//   trianglePoints.push_back(VisiLibity::Point(4, 1));
  
//   std::vector<VisiLibity::Point> squarePoints1 {};
//   squarePoints1.push_back(VisiLibity::Point(-4, -3));
//   squarePoints1.push_back(VisiLibity::Point(-4, -1.5));
//   squarePoints1.push_back(VisiLibity::Point(3, -1.5));
//   squarePoints1.push_back(VisiLibity::Point(3, -3)); 

//   PathsD triangle {createPolygon(trianglePoints)};
//   PathsD square1 {createPolygon(squarePoints1)};
//   PathsD map {offsetPolygon(createPolygon(mapPoints), 0.5, true)};

//   std::vector<VisiLibity::Point> translatedMap {translatePolygon(map)};

//   FillRule fr = FillRule::EvenOdd;
//   SvgWriter svg;
  
//   svg.AddPaths(triangle, false, fr, 0x10AA66FF, 0xAA0066FF, 1, false);
//   svg.AddPaths(square1, false, fr, 0x10AA66FF, 0xAA0066FF, 1, false);
//   svg.AddPaths(map, false, fr, 0x10FF66FF, 0xFF0066FF, 1, false); 

//   svg.SaveToFile("sample_map.svg", 800, 600, 0);
//   System("sample_map.svg");
// }
