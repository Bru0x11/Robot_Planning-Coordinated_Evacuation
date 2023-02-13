#include <vector>
#include <iostream>
#include <cfloat>
#include "math.h"

#include "include/interpolation.h"
//#include "../include/interpolation.h"


#include <fstream>

using namespace VisiLibity;
using namespace std;

Vector::Vector(double x, double y){
        // Constructor
        this->x = x;
        this->y = y;
    }

double Vector::norm(){
        double n = sqrt(pow(this->x, 2) + pow(this->y, 2));
        return n;
    }

double Vector::dot(Vector v1, Vector v2){
    return ((v1.x)*(v2.x)) + ((v1.y)*(v2.y));
}

double Vector::angle(Vector v1, Vector v2){
    return acos(dot(v1, v2) / (v1.norm()*v2.norm()));
}


Line::Line(double a, double b, double c){
    
    if(b==0){
        this->vertical = true;
        this->a = a;
        this->b = b;
        this->c = c;
        this->m = DBL_MAX;
        this->q = -c/a;
    }
    else{
        if(a==0){
            this->horizontal = true;
        }
        this->a = a;
        this->b = b;
        this->c = c;
        this->m = -a/b;
        this->q = -c/b;
    }
}

Line::Line(double m, double q){
    this->m = m;
    this->q = q;


    if(m==DBL_MAX){
        this->vertical = true;
        this->a=1;
        this->c=-q;
        this->b=0;
    }
    else{
        if(m==0){
            this->horizontal = true;
        }
        this->a = -m;
        this->b = 1;
        this->c = -q;
    }

}

Line Line::find_parallel(double distance){
    
    double c_new;
    if(this->vertical){
        c_new = this->c + distance;
        return Line(this->a, this->b, c_new);
    }

    c_new = this->c - ((sqrt(pow(this->a,2) + pow(this->b,2)))*distance);
    return Line(this->a, this->b, c_new);

}

Line Line::find_perpendicular(VisiLibity::Point p){

    if(this->vertical){
        return Line(0, p.y());
    }

    if(this->horizontal){
        return Line(1, 0, -p.x());
    }
    
    double mp = -1/(this->m);
    //double qp = Line::get_q_from_point(p,mp);

    return Line::get_line_from_m_and_p(p,mp);
}

Line Line::get_line_from_m_and_p(VisiLibity::Point p, double m){
    if(m==DBL_MAX){
        return Line(1, 0, -p.x());
    }
    if(m==0){
        return Line(0, 1, -p.y());
    }

    double q = p.y() - m*p.x();
    return Line(m,q);
}


Line Line::get_line_from_points(VisiLibity::Point p1, VisiLibity::Point p2){
    double m,q;
    if(p1.x() == p2.x()){
        m = DBL_MAX;
        q = p1.y();
        return Line(m, q);
    }

    m = compute_m(p1, p2);

    return get_line_from_m_and_p(p1,m);
}




double Line::get_q_from_point(VisiLibity::Point p, double m){
    if(m==DBL_MAX){
        return p.x();
    }
    if(m==0){
        return p.y();
    }

    return p.y() - m*p.x();

}

double Line::compute_m(VisiLibity::Point p0, VisiLibity::Point p1){
    double x1 = p1.x();
    double x0 = p0.x();

    if ((x1-x0) != 0)
    {
        double y1,y0;
        y1 = p1.y();
        y0 = p0.y();
        return (y1-y0) / (x1-x0);
    }
    return DBL_MAX;
}


ostream & operator << (ostream &out, Line l){
    out<<l.a<<"x+"<<l.b<<"y+"<<l.c<<"=0"<<endl;
    return out;
}


VisiLibity::Point find_lines_intersection(Line l1, Line l2){
    
    double a1 = l1.a;
    double b1 = l1.b; 
    double c1 = l1.c;
    double a2 = l2.a;
    double b2 = l2.b;
    double c2 = l2.c;
    double x = (b1*c2 - b2*c1)/(a1*b2 - a2*b1);
    double y = (c1*a2 - c2*a1)/(a1*b2 - a2*b1);

    return VisiLibity::Point(x,y);
}

double find_distance(VisiLibity::Point p0,VisiLibity::Point p1, VisiLibity::Point p2, double minR)
{

    Vector v1 = Vector(p1.x() - p0.x(), p1.y() - p0.y());
    Vector v2 = Vector(p2.x() - p1.x(), p2.y() - p1.y());

    double angle = Vector::angle(v1, v2);

    angle = M_PI - angle; 

    double distance = minR * (1 / tan(angle / 2));

    return abs(distance);
}


// given the initial and final points of the segment and a distance, compute the entrance point
VisiLibity::Point find_entrance(VisiLibity::Point p0, VisiLibity::Point p1, double distance)
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

    double x_entrance = x1 - distance * unit_v.x;
    double y_entrance = y1 - distance * unit_v.y;
    VisiLibity::Point entrance = VisiLibity::Point(x_entrance, y_entrance);

    return entrance;
};

VisiLibity::Point find_exit(VisiLibity::Point p0, VisiLibity::Point p1, double distance)
{
    VisiLibity::Point exit = find_entrance(p1, p0, distance);
    return exit;
}

Polyline get_points_from_arc(Arc a, int npts){
    
    Polyline points;
    VisiLibity::Point point;
    Pos pose;

    double k = a.k;

    double true_x = a.xf;
    double true_y = a.yf;

    Pos temp = circline(a.L, a.x0, a.y0, a.th0, a.k);

    double epsilon = 0.01;

    if((abs(temp.x - true_x)>epsilon) || (abs(temp.y - true_y)>epsilon)){
        a.k = -k;
    }

    for (int j=0; j<npts; j++){

          double s = a.L/npts * j;

          pose = circline(s, a.x0, a.y0, a.th0, a.k);

          point.set_x(pose.x);
          point.set_y(pose.y);

          points.push_back(point);
     }

    return points; 
}

Polyline get_points_from_curve(Curve c, int npts){

    int npts_arc = (int)(npts/3); 

    Arc a1 = c.a1;
    Arc a2 = c.a2;
    Arc a3 = c.a3;

    Polyline line1 = get_points_from_arc(a1, npts_arc);
    Polyline line2 = get_points_from_arc(a2, npts_arc);
    Polyline line3 = get_points_from_arc(a3, npts_arc);
    
    Polyline tot_line;
    tot_line.append(line1);
    tot_line.append(line2);
    tot_line.append(line3);

    return tot_line;
}


Polyline get_points_line(VisiLibity::Point p0, VisiLibity::Point p1){
    
    Polyline points;
    Line l = Line::get_line_from_points(p0, p1);

    double x0 = p0.x();
    double x1 = p1.x();

    if(x1>x0){
        for(int i=1; i<100; i++){
            double gamma = i*0.01;
            //cout<<"gamma: "<<gamma<<endl;
            double temp_x = (1-gamma)*x0 + (gamma*x1);
            //cout<<"temp_x: "<<temp_x<<endl;
            double temp_y = l.m*temp_x + l.q;
            //cout<<"temp_y: "<<temp_y<<endl;

            VisiLibity::Point p =VisiLibity::Point(temp_x, temp_y);

            points.push_back(p);
        }
    }

    else if(x1<x0){
        for(int i=1; i<100; i++){
            double gamma = i*0.01;
            //cout<<"gamma: "<<gamma<<endl;
            double temp_x = (1-gamma)*x0 + (gamma*x1);
            //cout<<"temp_x: "<<temp_x<<endl;
            double temp_y = l.m*temp_x + l.q;
            //cout<<"temp_y: "<<temp_y<<endl;

            VisiLibity::Point p = VisiLibity::Point(temp_x, temp_y);

            points.push_back(p);
        }
    }

    else{
        double temp_x= p0.x();
        for(int i=1; i<100; i++){
            double gamma = i*0.01;
            //cout<<"gamma: "<<gamma<<endl;
            double temp_y = (1-gamma)*p0.y() + (gamma*p1.y());
            //cout<<"temp_x: "<<temp_x<<endl;
            //cout<<"temp_y: "<<temp_y<<endl;
            VisiLibity::Point p = VisiLibity::Point(temp_x, temp_y);

            points.push_back(p);
        }
    }

    return points;

}


double compute_arc_length(VisiLibity::Point a, VisiLibity::Point b, double minR){
    
    double points_distance = sqrt(pow(b.x() - a.x(),2) + pow(b.y() - a.y(),2));

    double central_angle = acos(1 - (pow(points_distance,2)/(2*pow(minR, 2))));
    double L = minR * central_angle;

    return L;
}


double compute_angle(VisiLibity::Point a, VisiLibity::Point b){

    Vector v1 = Vector(b.x() - a.x(), b.y() - a.y());
    Vector v2 = Vector(1, 0);

    double angle = Vector::angle(v1, v2);

    if(v1.y < 0){
        angle = -angle;
    }

    return angle;
}

Arc get_arc(VisiLibity::Point entrance, VisiLibity::Point exit, double angle_entrance, double angle_exit, double minR){
    
    Arc arc;

    arc.x0 = entrance.x();
    arc.y0 = entrance.y();
    arc.th0 = angle_entrance;
    arc.xf = exit.x();
    arc.yf = exit.y();
    arc.thf = angle_exit;
    arc.k = 1/minR;
    arc.L = compute_arc_length(entrance, exit, minR);

    cout<<"L arc: "<<arc.L<<endl;
    return arc;

}

Environment get_environment1(){

    vector<VisiLibity::Point> points_obs1;
    points_obs1.push_back(VisiLibity::Point(4,4));
    points_obs1.push_back(VisiLibity::Point(4, 8));
    points_obs1.push_back(VisiLibity::Point(11, 8));
    points_obs1.push_back(VisiLibity::Point(11, 4));
    Polygon obs1 = Polygon(points_obs1);

    vector<VisiLibity::Point> points_obs2;
    points_obs2.push_back(VisiLibity::Point(8, 10));
    points_obs2.push_back(VisiLibity::Point(7, 12));
    points_obs2.push_back(VisiLibity::Point(14, 12));
    points_obs2.push_back(VisiLibity::Point(14, 10));
    Polygon obs2 = Polygon(points_obs2);

    vector<VisiLibity::Point> points_obs3;
    points_obs3.push_back(VisiLibity::Point(13, 6));
    points_obs3.push_back(VisiLibity::Point(13, 8));
    points_obs3.push_back(VisiLibity::Point(16, 8));
    points_obs3.push_back(VisiLibity::Point(16, 6));
    Polygon obs3 = Polygon(points_obs3);



    vector<VisiLibity::Point> points_env;
    points_env.push_back(VisiLibity::Point(-20.0, -20.0));
    points_env.push_back(VisiLibity::Point(20.0, -20.0));
    points_env.push_back(VisiLibity::Point(20.0, 20.0));
    points_env.push_back(VisiLibity::Point(-20.0, 20.0));

    Environment poly_env = Environment(points_env);

    poly_env.add_hole(obs1);
    poly_env.add_hole(obs2);
    poly_env.add_hole(obs3);


    return poly_env;
}

Environment get_environment2(){

    vector<VisiLibity::Point> points_obs1;
    points_obs1.push_back(VisiLibity::Point(2, 2));
    points_obs1.push_back(VisiLibity::Point(2, 3));
    points_obs1.push_back(VisiLibity::Point(3, 3));
    points_obs1.push_back(VisiLibity::Point(3, 2));

    Polygon obs1 = Polygon(points_obs1);

    vector<VisiLibity::Point> points_obs2;
    points_obs2.push_back(VisiLibity::Point(-1, 4));
    points_obs2.push_back(VisiLibity::Point(-1, 5));
    points_obs2.push_back(VisiLibity::Point(5, 5));
    points_obs2.push_back(VisiLibity::Point(5, 4));
    
    
    
    Polygon obs2 = Polygon(points_obs2);

    vector<VisiLibity::Point> points_env;
    points_env.push_back(VisiLibity::Point(-20.0, -20.0));
    points_env.push_back(VisiLibity::Point(20.0, -20.0));
    points_env.push_back(VisiLibity::Point(20.0, 20.0));
    points_env.push_back(VisiLibity::Point(-20.0, 20.0));

    Environment poly_env = Environment(points_env);

    poly_env.add_hole(obs1);
    poly_env.add_hole(obs2);

    return poly_env;
}

Environment get_environment3(){

    vector<VisiLibity::Point> points_obs1;

    points_obs1.push_back(VisiLibity::Point(-1, -7));
    points_obs1.push_back(VisiLibity::Point(-1, -4));
    points_obs1.push_back(VisiLibity::Point(5, -4));
    points_obs1.push_back(VisiLibity::Point(5, -7));    

    Polygon obs1 = Polygon(points_obs1);

    vector<VisiLibity::Point> points_obs2;

    points_obs2.push_back(VisiLibity::Point(2, -2));
    points_obs2.push_back(VisiLibity::Point(2, -1));
    points_obs2.push_back(VisiLibity::Point(4, -1));
    // points_obs2.push_back(VisiLibity::Point(4, -2));
    
    Polygon obs2 = Polygon(points_obs2);

    vector<VisiLibity::Point> points_obs3;
    points_obs3.push_back(VisiLibity::Point(-2, 2));
    points_obs3.push_back(VisiLibity::Point(-2, 3));
    points_obs3.push_back(VisiLibity::Point(1, 3));
    points_obs3.push_back(VisiLibity::Point(1, 2));
    Polygon obs3 = Polygon(points_obs3);

    vector<VisiLibity::Point> points_env;
    points_env.push_back(VisiLibity::Point(-20.0, -20.0));
    points_env.push_back(VisiLibity::Point(20.0, -20.0));
    points_env.push_back(VisiLibity::Point(20.0, 20.0));
    points_env.push_back(VisiLibity::Point(-20.0, 20.0));

    // vector<VisiLibity::Point> points_obs4;
    // points_obs4.push_back(VisiLibity::Point(19, 0));
    // points_obs4.push_back(VisiLibity::Point(19, 1));
    // points_obs4.push_back(VisiLibity::Point(19.5, 1));
    // points_obs4.push_back(VisiLibity::Point(19.5, 0));
    // Polygon obs4 = Polygon(points_obs4);

    vector<VisiLibity::Point> points_obs5;
    points_obs5.push_back(VisiLibity::Point(0, -3.5));
    points_obs5.push_back(VisiLibity::Point(0, -2));
    points_obs5.push_back(VisiLibity::Point(1.5, -2));
    points_obs5.push_back(VisiLibity::Point(1.5, -3.5));
    Polygon obs5 = Polygon(points_obs5);

    Environment poly_env = Environment(points_env);

    poly_env.add_hole(obs1);
    poly_env.add_hole(obs2);
    poly_env.add_hole(obs3);
    // poly_env.add_hole(obs4);
    poly_env.add_hole(obs5);

    return poly_env;
}

Environment get_maze_env(){

    vector<VisiLibity::Point> points_obs1;

    // points_obs1.push_back(VisiLibity::Point(-4, -3));
    // points_obs1.push_back(VisiLibity::Point(-4, -1.5));
    // points_obs1.push_back(VisiLibity::Point(3, -1.5));
    // points_obs1.push_back(VisiLibity::Point(3, -3));

    // points_obs1.push_back(VisiLibity::Point(-6, 2));
    // points_obs1.push_back(VisiLibity::Point(-6, 6));
    // points_obs1.push_back(VisiLibity::Point(0, 6));

    points_obs1.push_back(VisiLibity::Point(-6.8,  -7.1));
    points_obs1.push_back(VisiLibity::Point(6, -1.6));
    points_obs1.push_back(VisiLibity::Point(6, -7.1));

    Polygon obs1 = Polygon(points_obs1);

    vector<VisiLibity::Point> points_obs2;
    // points_obs2.push_back(VisiLibity::Point(-2, 1));
    // points_obs2.push_back(VisiLibity::Point(-2, 3));
    // points_obs2.push_back(VisiLibity::Point(4, 3));
    // points_obs2.push_back(VisiLibity::Point(4, 1));

    // points_obs2.push_back(VisiLibity::Point(2, -3.15));
    // points_obs2.push_back(VisiLibity::Point(2, 4));
    // points_obs2.push_back(VisiLibity::Point(6, 2));
    // points_obs2.push_back(VisiLibity::Point(6, -3.15));

    points_obs2.push_back(VisiLibity::Point(-4, 0));
    points_obs2.push_back(VisiLibity::Point(-7.3, 2));
    points_obs2.push_back(VisiLibity::Point(-7.6, 4));
    points_obs2.push_back(VisiLibity::Point(-2, 4));
    points_obs2.push_back(VisiLibity::Point(-0.5, 2));
    
    Polygon obs2 = Polygon(points_obs2);

    vector<VisiLibity::Point> points_obs3;
    // points_obs3.push_back(VisiLibity::Point(-4, 1));
    // points_obs3.push_back(VisiLibity::Point(-4, 5));
    // points_obs3.push_back(VisiLibity::Point(-1.5, 2));

    // points_obs3.push_back(VisiLibity::Point(-6, -3.2));
    // points_obs3.push_back(VisiLibity::Point(-4, 2.8));
    // points_obs3.push_back(VisiLibity::Point(-2, -3.2));

    points_obs3.push_back(VisiLibity::Point(6, 2));
    points_obs3.push_back(VisiLibity::Point(0.5, 6.8));
    points_obs3.push_back(VisiLibity::Point(6.8, 6.9));
    points_obs3.push_back(VisiLibity::Point(4.4, 6));
    
    Polygon obs3 = Polygon(points_obs3);

    vector<VisiLibity::Point> points_env;
    points_env.push_back(VisiLibity::Point(-8, -8));
    points_env.push_back(VisiLibity::Point(8, -8));
    points_env.push_back(VisiLibity::Point(8, 8));
    points_env.push_back(VisiLibity::Point(-8, 8));

    Environment poly_env = Environment(points_env);

    poly_env.add_hole(obs1);
    poly_env.add_hole(obs2);
    poly_env.add_hole(obs3);
    //prova
    return poly_env;
}

Environment get_env_offset(Environment env, double minR, double minH){
    //For printing purposes
    FillRule fr = FillRule::EvenOdd;
    SvgWriter svg;

    //Save all the polygon in this variable
    vector<PathsD> polygons;

    //------------------MAP------------------
    cout << "...CREATING THE MAP...\n";
    int map_pos = 0;
    vector<VisiLibity::Point> boundary_points;
    Polygon boundary = env[0];

    //Extrapolating the points
    for(int j = 0; j<boundary.n(); j++){
        VisiLibity::Point p = boundary[j];
        boundary_points.push_back(p);
    }

    PathsD b_boundary = createPolygon(boundary_points);
    cout << "Points of the normal boundary: " << b_boundary << '\n';

    PathsD off_boundary = offsetPolygon(b_boundary, -0.5, true);
    cout << "Point of offsetted boundary: " << off_boundary << '\n';

    polygons.push_back(off_boundary);   

    //-----------------OBSTACLES----------------
    cout << "...CREATING THE OBSTACLES...\n";
    for(int i = 1; i<=env.h(); i++){
        
        Polygon poly = env[i];
        vector<VisiLibity::Point> poly_points;
        
        //Extrapolating the points
        for(int j = 0; j<poly.n(); j++){
            VisiLibity::Point p = poly[j];
            poly_points.push_back(p);
        }

        //Computing the right offset for the polygon
        double min_angle = find_min_angle(poly);
        float off_value = (float) offset_calculator(min_angle, minR, minH);

        PathsD b_poly = createPolygon(poly_points);
        cout << "\nPoints of the "<< i << "-th normal polygon: " << b_poly << '\n';
        PathsD off_b_poly = offsetPolygon(b_poly, off_value, false);
        cout << "Points of the "<< i << "-th offsetted polygon: " << off_b_poly << '\n';

        //Check whether there are intersection between the different polygons
        cout << "Checking intersection of " << i << "-th offsetted polygon" << '\n';
        checkIntersections(off_b_poly, polygons, 0);
        
        cout << "Checking all the polygons within the vector at i-th iteration:\n";
        for (int k=0; k<polygons.size(); k++){
            cout << polygons[k] << '\n';
        }
    }

    cout << "SIZE OF POLYGONS: " << polygons.size() << '\n';
    //For printing purposes
    for(int i=0; i<polygons.size(); i++){
        cout << polygons[i] << '\n';
        svg.AddPaths(polygons[i], false, fr, 0x10AA66FF, 0xAA0066FF, 1, false);
    }   
    svg.SaveToFile("sample_map.svg", 800, 600, 0);

    //-----------------TRANSLATION----------------
    cout << "...TRANSLATING THE OBSTACLES...\n";
    vector<Polygon> translated_polygons;

    for(int i = 0; i<polygons.size(); i++){
        //The first has to be necessarily the map
        if(i==0){
            PathsD map = polygons[0]; 
            vector<VisiLibity::Point> translated_map_points;

            for (PointD point : map[1]){
                translated_map_points.push_back(VisiLibity::Point(point.x, point.y));
            }

            Polygon translated_map = Polygon(translated_map_points);
            translated_polygons.push_back(translated_map);
            cout << "Point of translated offsetted boundary: " << translated_map << '\n';
        }
        else{
            vector<VisiLibity::Point> poly_points = translatePolygon(polygons[i]);
            Polygon poly = Polygon(poly_points);
            translated_polygons.push_back(poly);
            cout << "Point of translated offsetted polygon: " << poly << '\n';     
        }
    }

    Environment off_env = Environment(translated_polygons);
    return off_env; 
}


Curve get_first_trait_dubins(Polyline shortest_path, double th0, double minR){

    //first 3 vertex (need them to do dubins)
    VisiLibity::Point start = shortest_path[0];
    VisiLibity::Point p0 = shortest_path[1];
    VisiLibity::Point p1 = shortest_path[2];
    //Find final angle of first trait
    double th1 = compute_angle(p0, p1);

    double Kmax = 1/minR;
    Curve first_trait = dubins_shortest_path(start.x(), start.y(), th0, p0.x(), p0.y(), th1, Kmax);

    return first_trait;
}


Curve get_last_trait_dubins(Polyline shortest_path, double thf, double minR){

    //BUILD DUBINS LAST TRAIT
    //last 3 vertex (need them to build dubins last trait)
    VisiLibity::Point pn_1 = shortest_path[shortest_path.size()-3];
    VisiLibity::Point pn = shortest_path[shortest_path.size()-2];
    VisiLibity::Point goal = shortest_path[shortest_path.size()-1];
    double th_n;

    th_n = compute_angle(pn_1, pn);
        
    double Kmax = 1/minR;

    Curve last_trait = dubins_shortest_path(pn.x(), pn.y(), th_n, goal.x(), goal.y(), thf, Kmax);

    return last_trait;
}

//Ashkan
double find_min_angle(VisiLibity::Polygon points){
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
    offset = std::max(minR * (1 - sin(minAngle/2)) + minH * sin(minAngle/2), minH);
    return offset;
}

int env_holes(VisiLibity::Environment env){
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

vector<Arc> get_arcs_interpolation(Polyline shortest_path, double minR){
    vector<Arc> arc_vect;

    for (int i = 1; i < shortest_path.size()-3; i++)
    {
        VisiLibity::Point a = shortest_path[i];
        VisiLibity::Point b = shortest_path[i+1];
        VisiLibity::Point c = shortest_path[i+2];

        double distance = find_distance(a,b,c, minR);

        VisiLibity::Point entrance = find_entrance(a, b, distance);
        VisiLibity::Point exit = find_exit(b, c, distance);

        cout<<"Entrance: "<<entrance<<endl;
        cout<<"Exit: "<<exit<<endl;

        double angle_entrance = compute_angle(a,b);
        double angle_exit = compute_angle(b,c);

        cout<<"Angle Entrance: "<<angle_entrance<<endl;
        cout<<"Angle Exit: "<<angle_exit<<endl;        
    
        Arc arc = get_arc(entrance, exit, angle_entrance, angle_exit, minR);
        arc_vect.push_back(arc);
    }
    return arc_vect;
}

Polyline interpolation(Polyline shortest_path, double th0, double thf, double minR){
    Polyline points_final_path;
    double kmax = 1/minR;

    if(shortest_path.size()==2){
        VisiLibity::Point start = shortest_path[0];
        VisiLibity::Point end = shortest_path[1];

        Curve dubins_path = dubins_shortest_path(start.x(), start.y(), th0, end.x(), end.y(), thf, kmax);
        points_final_path.append(get_points_from_curve(dubins_path, 200)); 
        return points_final_path;
    }

    else if(shortest_path.size()==3){
        VisiLibity::Point start = shortest_path[0];
        VisiLibity::Point middle = shortest_path[1];
        VisiLibity::Point end = shortest_path[2];
        
        double th_middle = compute_angle(middle, end);

        Curve first_trait = dubins_shortest_path(start.x(), start.y(), th0, middle.x(), middle.y(), th_middle, kmax);
        Curve second_trait = dubins_shortest_path(middle.x(), middle.y(), th_middle, end.x(), end.y(), thf, kmax);

        points_final_path.append(get_points_from_curve(first_trait, 100));
        points_final_path.append(get_points_from_curve(second_trait, 100));

        return points_final_path;
    }

    else if(shortest_path.size()==4){
        VisiLibity::Point start = shortest_path[0];
        VisiLibity::Point middle1 = shortest_path[1];
        VisiLibity::Point middle2 = shortest_path[2];
        VisiLibity::Point end = shortest_path[3];
        
        double th_m1_m2 = compute_angle(middle1, middle2);
        //double th_m2_end = compute_angle(middle2, end);

        Curve first_trait = dubins_shortest_path(start.x(), start.y(), th0, middle1.x(), middle1.y(), th_m1_m2, kmax);
        cout<<"Add LINE from "<<middle1<<" to "<<middle2<<endl;
        Polyline middle_line = get_points_line(middle1, middle2);
        Curve last_trait = dubins_shortest_path(middle2.x(), middle2.y(), th_m1_m2, end.x(), end.y(), thf, kmax);

        points_final_path.append(get_points_from_curve(first_trait, 100));
        points_final_path.append(middle_line);
        points_final_path.append(get_points_from_curve(last_trait, 100));

        return points_final_path;
    }

    else{

        Curve first_trait = get_first_trait_dubins(shortest_path, th0, minR);
        points_final_path.append(get_points_from_curve(first_trait, 100));

        vector<Arc> arc_vect = get_arcs_interpolation(shortest_path, minR);

        VisiLibity::Point s1 = shortest_path[1];
        Arc arc0 = arc_vect[0];
        VisiLibity::Point s2 = VisiLibity::Point(arc0.x0, arc0.y0);      

        //cout<<"arc0 x0,y0: "<<s2<<endl;  
        //cout<<"arc0 xf,yf: "<<arc0.xf<<" "<<arc0.yf <<endl;  
                
        points_final_path.append(get_points_line(s1, s2));
        points_final_path.append(get_points_from_arc(arc0, 100));

        for(int i = 2; i < shortest_path.size()-3; i++){
            Arc first_arc = arc_vect[i-2];
            Arc second_arc = arc_vect[i-1];

            VisiLibity::Point first_point = VisiLibity::Point(first_arc.xf, first_arc.yf);
            VisiLibity::Point second_point = VisiLibity::Point(second_arc.x0, second_arc.y0);
            
            points_final_path.append(get_points_line(first_point, second_point));
            points_final_path.append(get_points_from_arc(second_arc, 100));
        }

        Arc last_arc = arc_vect[arc_vect.size()-1];
        VisiLibity::Point sn_1 = VisiLibity::Point(last_arc.xf, last_arc.yf);
        VisiLibity::Point sn_2 = shortest_path[shortest_path.size() - 2];

        cout<<"Add LINE from "<<sn_2<<" to "<<sn_1<<endl;
        points_final_path.append(get_points_line(sn_2, sn_1));

        Curve last_trait = get_last_trait_dubins(shortest_path, thf, minR);
        points_final_path.append(get_points_from_curve(last_trait, 100));

        //cout<<"POINTS: "<<endl<<points_final_path<<endl;
        return points_final_path;

    }

    return points_final_path;
}