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


//Ashkan
double find_min_angle(Polygon points){
    double min_angle = 5;
    int indx_p1, indx_p2, indx_p3;

    for (int i = 0; i < points.n(); i++) {
//        cout << " point "<<i<<" "<<points[i].x()<<" "<<points[i].y() <<endl;
//         v1;
//        Vector v2;

        if(i == points.n() - 1){
            indx_p1 = i-1;
            indx_p2 = i;
            indx_p3 = 0;
        } if (i == 0){
            indx_p1 = points.n() - 1;
            indx_p2 = i;
            indx_p3 = i+1;
        }

        Vector v1 = Vector(points[indx_p2].x() - points[indx_p1].x(), points[indx_p2].y() - points[indx_p1].y());
        Vector v2 = Vector(points[indx_p2].x() - points[indx_p3].x(), points[indx_p2].y() - points[indx_p3].y());
        double angle = Vector::angle(v1,v2);
        if (min_angle > angle){
            min_angle = angle;
        }
    }
    return min_angle;
}

double offset_calculator(double minAngle, double minR, double minH){
    double offset;
    offset = max(minR * (1 - sin(minAngle/2)) + minH * sin(minAngle/2), minH);
    return offset;
}

int env_holes(Environment env){
    double angle;
    double offset;
    for(int i = 1 ; i < env.h() + 1 ; i++){
        angle = find_min_angle(env[i]);
        offset = offset_calculator(angle, 1.5, 0.5);
        cout<< "hole "<< i << " min angle: " << angle <<": \n";
        cout<< "hole "<< i << " offset: " << offset <<": \n";
    }
    return 0;
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
  mapPoints.push_back(VisiLibity::Point(0.0, 30.0));
  mapPoints.push_back(VisiLibity::Point(30.0, 30.0));
  mapPoints.push_back(VisiLibity::Point(30.0, 0.0));

  std::vector<VisiLibity::Point> trianglePoints {};
  trianglePoints.push_back(VisiLibity::Point(2.0, 20.0));
  trianglePoints.push_back(VisiLibity::Point(5.0, 28.0));
  trianglePoints.push_back(VisiLibity::Point(8.0, 20.0));
  
  std::vector<VisiLibity::Point> squarePoints1 {};
  squarePoints1.push_back(VisiLibity::Point(2.0, 6.0));
  squarePoints1.push_back(VisiLibity::Point(2.0, 12.0));
  squarePoints1.push_back(VisiLibity::Point(16.0, 12.0));
  squarePoints1.push_back(VisiLibity::Point(16.0, 6.0));

  std::vector<VisiLibity::Point> squarePoints2 {};
  squarePoints2.push_back(VisiLibity::Point(12.0, 18.0));
  squarePoints2.push_back(VisiLibity::Point(12.0, 24.0));
  squarePoints2.push_back(VisiLibity::Point(28.0, 24.0));
  squarePoints2.push_back(VisiLibity::Point(28.0, 18.0));

  std::vector<VisiLibity::Point> startingP {};
  startingP.push_back(VisiLibity::Point(5.0, 3.0));

  PathsD triangle {createPolygon(trianglePoints)};
  PathsD square1 {createPolygon(squarePoints1)};
  PathsD square2 {createPolygon(squarePoints2)};
  PathsD map {offsetPolygon(createPolygon(mapPoints), 1.0, true)};

  std::vector<VisiLibity::Point> translatedMap {translatePolygon(map)};

  FillRule fr = FillRule::EvenOdd;
  SvgWriter svg;
  /*
  svg.AddPaths(triangle, false, fr, 0x10AA66FF, 0xAA0066FF, 1, false);
  svg.AddPaths(square1, false, fr, 0x10AA66FF, 0xAA0066FF, 1, false);
  svg.AddPaths(square2, false, fr, 0x10AA66FF, 0xAA0066FF, 1, false);
  svg.AddPaths(map, false, fr, 0x10FF66FF, 0xFF0066FF, 1, false); 

  svg.SaveToFile("sample_map.svg", 800, 600, 0);
  System("sample_map.svg");*/
}
