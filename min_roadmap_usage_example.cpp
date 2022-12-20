#include "VisiLibity1/src/visilibity.hpp"
#include <vector>
#include <iostream>
#include <cfloat>
#include "dubins.h"
#include "math.h"
#include "interpolation.h"

#include <fstream>

using namespace VisiLibity;
using namespace std;

int main()
{

    // Create and open a text file
    ofstream MyFile("env.csv");
    MyFile<<"x,y"<<endl;

    Environment env = get_environment();

    //ROAD MAP
    Visibility_Graph graph = Visibility_Graph(env, 0);

    //DEFINE ROBOT MIN_CURVATURE_RADIUS
    double minR = 1.5;
    //DEFINE START AND END POINTS
    Point start_test = Point(3.0, 1.0);
    Point end = Point(4.0, 17.0);
    //DEFINE START AND END ANGLES 
    double th0 = 0;
    double thf = M_PI/2;

    //FIND SHORTES PATH
    Polyline shortest_path = env.shortest_path(start_test, end, graph, 0.0);

    cout << "Enviroment is valid: " << env.is_valid() << endl;
    cout << "Shortest_path: " << endl;
    cout << shortest_path << endl;


    Polyline points_final_path = interpolation(shortest_path, th0, thf, minR);


    for(int i=0; i<points_final_path.size(); i++){
        double x = points_final_path[i].x();
        double y = points_final_path[i].y();
        MyFile<<x<<","<<y<<endl;
    }


    // Close the file
    MyFile.close();


    return 0;
}