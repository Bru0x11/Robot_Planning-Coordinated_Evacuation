#include "../include/dubins.h"
#include "../include/interpolation.h"
#include "../../VisiLibity1/src/visilibity.hpp"
#include <unistd.h>

using namespace VisiLibity;
using namespace std;

int main(){
    string filename = "example1.environment";

    // Line l = Line(1,0,0);
    cout<<"angle: "<<compute_angle(Point(0,0), Point(-5,-5))<<endl;

    sleep(5);

    Environment env = Environment(filename);
    cout<<"ho letto env"<<endl;

    //ROAD MAP
    Visibility_Graph graph = Visibility_Graph(env, 0.1);

    //DEFINE ROBOT MIN_CURVATURE_RADIUS
    double minR = 1.5;
    //DEFINE START AND END POINTS
    double x0 = 0;
    double y0 = 0;

    Point start_test = Point(0, 0);
    Point end = Point(4.0, 17.0);
    //DEFINE START AND END ANGLES 
    double th0 = 0;
    double thf = M_PI/2;

    //FIND SHORTES PATH
    Polyline shortest_path = env.shortest_path(start_test, end, graph, 0.1);

    cout << "Enviroment is valid: " << env.is_valid() << endl;
    cout << "Shortest_path: " << endl;
    cout << shortest_path << endl;

    cout << "env: "<<endl;
    cout<<env<<endl;

    //cout << "graph: "<<endl<<graph<<endl;
    sleep(5); 

    cout<<"faccio interpolation"<<endl;

    Polyline points_final_path = interpolation(shortest_path, th0, thf, minR);
    return 0;
}