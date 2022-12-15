#include "VisiLibity1/src/visilibity.hpp"
#include <vector>
#include <iostream>
#include "dubins.h"
#include "math.h"


using namespace VisiLibity;
using namespace std;

class Vector{
public: 
    double x;
    double y;

    Vector(int x, int y)
    {
        // Constructor
        this->x = x;
        this->y = y;
    }

    double norm(){
        double n = sqrt(pow(this -> x, 2) + pow(this -> y, 2));
        return n;
    }
};

Point find_entrance(Point p0, Point p1, double distance){
    double x0 = p0.x();
    double y0 = p0.y();
    double x1 = p1.x();
    double y1 = p1.y();

    Vector v = Vector(x1-x0, y1-y0);
    double n = v.norm();
    Vector unit_v = Vector(v.x/n, v.y/n);
    
    double x_entrance = x0 + distance*unit_v.x;
    double y_entrance = y0 + distance*unit_v.y;
    Point entrance = Point(x_entrance, y_entrance);
    
    return entrance;
};

int main(){

    vector<Point> points_obs1;
    points_obs1.push_back(Point(1.0, 2.0));
    points_obs1.push_back(Point(6.0, 7.0));
    points_obs1.push_back(Point(6.0, 2.0));

    Polygon obs1 = Polygon(points_obs1);
    cout<<"obs1 area: "<<obs1.area()<<endl;

    vector<Point> points_obs2;
    points_obs2.push_back(Point(2.0, 9.0));
    points_obs2.push_back(Point(2.0, 14.0));
    points_obs2.push_back(Point(8.0, 14.0));
    points_obs2.push_back(Point(8.0, 9.0));

    Polygon obs2 = Polygon(points_obs2);

    cout<<"obs2 area: "<<obs2.area()<<endl;

    vector<Point> points_env;
    points_env.push_back(Point(0.0, 0.0));
    points_env.push_back(Point(15.0, 0.0));
    points_env.push_back(Point(15.0, 15.0));
    points_env.push_back(Point(0.0, 15.0));

    Polygon poly_env = Polygon(points_env);

    //cout<<obs2.area();

    vector<Polygon> obstacles;
    obstacles.push_back(poly_env);
    obstacles.push_back(obs1);
    obstacles.push_back(obs2);

    Environment env = Environment(obstacles);

    Visibility_Graph graph = Visibility_Graph(env, 0);

    Point start = Point(3.0, 1.0);
    Point end = Point(9.0, 14.0);
    Polyline shortest_path = env.shortest_path(start, end, graph, 0.0);

    //cout<<shortest_path.length()<<endl;

    cout<<"Enviroment is valid: "<<env.is_valid()<<endl;
    //cout<<"\n"<<env<<endl;

    cout<<"Shortest_path: "<<endl;
    cout<<shortest_path<<endl;

    
    //TO DO: dubins from start point to first point


    //for Point in shortest path
    int path_length = shortest_path.size();

    for(int i = 1; i < path_length; i++){
        Point p0 = shortest_path[i];
    }



    Point p1 = Point(0.0, 0.0);
    Point p2 = Point(0.0, 5.0);

    double distance = 2;
    Point p3 = find_entrance(p1, p2, 2.0);

    cout<<"x: "<<p3.x()<<endl;
    cout<<"y: "<<p3.y()<<endl;







    /*
    vector<Point> poses;

    Pos pose_temp;
    Point position_temp;

      
    //Define problem data
    double th0 = 0;
    double thf = 0;
    double Kmax = 3.0;

    //Find optimal Dubins solution
    vector<Curve> dubins_curves = multipoint_dubins(shortest_path, Kmax);

    int npts = 100;
    Curve c1 = dubins_curves[0];

    for (int j=0; j<npts; j++){

        double s = c1.a1.L/npts * j;

        Pos pose;
        pose = circline(s, c1.a1.x0, c1.a1.y0, c1.a1.th0, c1.a1.k);

        position_temp.set_x(pose.x);
        position_temp.set_y(pose.y);

        poses.push_back(position_temp);
    }
    */

    //cout<<"poses[0]: "<<poses<<endl;

    return 0;
}