#include "VisiLibity1/src/visilibity.hpp"
#include <vector>
#include <iostream>
#include <cfloat>
#include "dubins.h"
#include "math.h"



#include <fstream>

using namespace VisiLibity;
using namespace std;

class Vector
{
public:
    double x;
    double y;

    Vector(double x, double y)
    {
        // Constructor
        this->x = x;
        this->y = y;
    }

    double norm()
    {
        double n = sqrt(pow(this->x, 2) + pow(this->y, 2));
        return n;
    }
};

double compute_m(Point p0, Point p1){
    if ((p1.x() - p0.x()) != 0)
    {
        return (p1.y() - p0.y()) / (p1.x() - p0.x());
    }
    return DBL_MAX;
}

double find_distance(Point p0, Point p1, Point p2, double minR)
{
    double m1, m2;
    // Compute m
    m1 = compute_m(p0,p1);

    m2 = compute_m(p1,p2);

    double tan_angle = (m1 - m2) / (1 + (m1 * m2));
    double angle = atan(tan_angle);

    double distance = minR * (1 / tan(angle / 2));

    return abs(distance);
}

// given the initial and final points of the segment and a distance, compute the entrance point
Point find_entrance(Point p0, Point p1, double distance)
{
    double x0 = p0.x();
    double y0 = p0.y();
    double x1 = p1.x();
    double y1 = p1.y();

    Vector v = Vector(x1 - x0, y1 - y0);
    double n = v.norm();

    double unit_v_x = v.x / n;
    double unit_v_y = v.y / n;

    Vector unit_v = Vector(unit_v_x, unit_v_y);

    double x_entrance = x0 + distance * unit_v.x;
    double y_entrance = y0 + distance * unit_v.y;
    Point entrance = Point(x_entrance, y_entrance);

    return entrance;
};

Point find_exit(Point p0, Point p1, double distance)
{
    Point exit = find_entrance(p1, p0, distance);
    return exit;
}

int main()
{

    // Create and open a text file
    ofstream MyFile("env.csv");

    vector<Point> points_obs1;
    points_obs1.push_back(Point(1.0, 2.0));
    points_obs1.push_back(Point(6.0, 7.0));
    points_obs1.push_back(Point(6.0, 2.0));

    MyFile<<"x,y"<<endl;
    for (Point point : points_obs1){
        MyFile<<point.x()<<","<<point.y()<<endl;
    }

    Polygon obs1 = Polygon(points_obs1);

    vector<Point> points_obs2;
    points_obs2.push_back(Point(2.0, 9.0));
    points_obs2.push_back(Point(2.0, 14.0));
    points_obs2.push_back(Point(8.0, 14.0));
    points_obs2.push_back(Point(8.0, 9.0));

    for (Point point : points_obs2){
        MyFile<<point.x()<<","<<point.y()<<endl;
    }

    Polygon obs2 = Polygon(points_obs2);

    vector<Point> points_env;
    points_env.push_back(Point(0.0, 0.0));
    points_env.push_back(Point(15.0, 0.0));
    points_env.push_back(Point(15.0, 15.0));
    points_env.push_back(Point(0.0, 15.0));

    //for (Point point : points_env){
    //    MyFile<<point.x()<<" "<<point.y()<<endl;
    //}

    Polygon poly_env = Polygon(points_env);

    // cout<<obs2.area();

    vector<Polygon> obstacles;
    obstacles.push_back(poly_env);
    obstacles.push_back(obs1);
    obstacles.push_back(obs2);

    Environment env = Environment(obstacles);

    Visibility_Graph graph = Visibility_Graph(env, 0);

    Point start_test = Point(3.0, 1.0);
    Point end = Point(9.0, 14.0);
    Polyline shortest_path = env.shortest_path(start_test, end, graph, 0.0);

    // cout<<shortest_path.length()<<endl;

    cout << "Enviroment is valid: " << env.is_valid() << endl;
    // cout<<"\n"<<env<<endl;

    cout << "Shortest_path: " << endl;
    cout << shortest_path << endl;


    for (int i = 0; i<shortest_path.size(); i++){
        MyFile<<shortest_path[i].x()<<","<<shortest_path[i].y()<<endl;
    }


    // for Point in shortest path
    int path_length = shortest_path.size();
    Point start = shortest_path[0];
    Point p0 = shortest_path[1];
    Point p1 = shortest_path[2];

    double th0 = 0;
    //Find final angle of first trait
    double m = compute_m(p0,p1);
    double th1 = atan(m);

    double minR = 0.3;
    double Kmax = 1/minR;
    Curve first_trait = dubins_shortest_path(start.x(), start.y(), th0, p0.x(), p0.y(), th1, Kmax);

    Polyline path; 
    path.push_back(start);

    for (int i = 0; i < path_length - 2; i++)
    {
        Point a = shortest_path[i];
        Point b = shortest_path[i+1];
        Point c = shortest_path[i+2];

        double distance = find_distance(a, b, c, minR);

        Point exit = find_exit(a, b, minR);
        Point entrance = find_entrance(b, c, minR);

        path.push_back(exit);
        path.push_back(entrance);
    }

    Point pn_1 = shortest_path[path_length-3];
    Point pn = shortest_path[path_length-2];
    Point goal = shortest_path[path_length-1];

    double thf = 0;
    double m_final_segment = compute_m(pn,goal);
    double thn = atan(m_final_segment);

    Curve last_trait = dubins_shortest_path(goal.x(), goal.y(), th0, pn.x(), pn.y(), thf, Kmax);


    cout << "Interpolated path: " << endl;
    cout << path << endl;

    for (int i = 0; i<path.size(); i++){
        MyFile<<path[i].x()<<","<<path[i].y()<<endl;
    }

    // Close the file
    MyFile.close();

    /*
    Point p0_ = Point(0, 0);
    Point p1_ = Point(4, 0);
    Point p2_ = Point(4, 4);

    cout << find_distance(p0_, p1_, p2_, 2.0) << endl;

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

    // cout<<"poses[0]: "<<poses<<endl;

    return 0;
}