#include "../include/dubins.h"
#include "../include/interpolation.h"
#include "../../VisiLibity1/src/visilibity.hpp"
#include <unistd.h>

using namespace VisiLibity;
using namespace std;




int main(int argc, char** argv){
    
    string filename = "example1.environment";
    Environment env = get_environment3();

    double epsilon = 0.001;

    //ROAD MAP
    Visibility_Graph graph = Visibility_Graph(env, epsilon);

    //DEFINE ROBOT MIN_CURVATURE_RADIUS
    double minR = 1;
    //DEFINE START AND END POINTS
    double x0 = 3;
    double y0 = -8;

    Point start_test = Point(x0, y0);
    Point end = Point(0, 4);
    //DEFINE START AND END ANGLES 
    double th0 = 0;
    double thf = M_PI/2;

    //FIND SHORTES PATH
    Polyline shortest_path = env.shortest_path(start_test, end, graph, epsilon);

    cout << "Enviroment is valid: " << env.is_valid(epsilon) << endl;
    cout << "Shortest_path: " << endl;
    cout << shortest_path << endl;

    cout << "env: "<<endl;
    cout<<env<<endl;

    //cout << "graph: "<<endl<<graph<<endl;
    //sleep(5); 

    cout<<"faccio interpolation"<<endl;

    Polyline points_final_path = interpolation(shortest_path, th0, thf, minR);

    ofstream MyFile("points.csv");

    // Scrivi ogni punto su file
    for (int i = 0; i < points_final_path.size(); i++){
        MyFile << points_final_path[i].x() << ", " <<points_final_path[i].y() << endl;
    }
    // Chiudi il file
    MyFile.close();

    return 0;
}