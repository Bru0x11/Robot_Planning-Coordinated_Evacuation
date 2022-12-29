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

//friend ostream & operator << (ostream &out, Line l);

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

Line Line::find_perpendicular(Point p){

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

Line Line::get_line_from_m_and_p(Point p, double m){
    if(m==DBL_MAX){
        return Line(1, 0, -p.x());
    }
    if(m==0){
        return Line(0, 1, -p.y());
    }

    double q = p.y() - m*p.x();
    return Line(m,q);
}


Line Line::get_line_from_points(Point p1, Point p2){
    double m,q;
    if(p1.x() == p2.x()){
        m = DBL_MAX;
        q = p1.y();
        return Line(m, q);
    }

    m = compute_m(p1, p2);

    //q = get_q_from_point(p1, m);
    return get_line_from_m_and_p(p1,m);
}




double Line::get_q_from_point(Point p, double m){
    if(m==DBL_MAX){
        return p.x();
    }
    if(m==0){
        return p.y();
    }

    return p.y() - m*p.x();

}

double Line::compute_m(Point p0, Point p1){
    double x1 = p1.x();
    double x0 = p0.x();

    if ((x1-x0) != 0)
    {
        double y1,y0;
        y1 = p1.y();
        y0 = p0.y();
        //cout<<"y0: "<<y0<<endl;
        //cout<<"y1: "<<y1<<endl;
        return (y1-y0) / (x1-x0);
    }
    return DBL_MAX;
}


ostream & operator << (ostream &out, Line l){
    out<<l.a<<"x+"<<l.b<<"y+"<<l.c<<"=0"<<endl;
    return out;
}


Point find_lines_intersection(Line l1, Line l2){
    
    double a1 = l1.a;
    double b1 = l1.b; 
    double c1 = l1.c;
    double a2 = l2.a;
    double b2 = l2.b;
    double c2 = l2.c;
    double x = (b1*c2 - b2*c1)/(a1*b2 - a2*b1);
    double y = (c1*a2 - c2*a1)/(a1*b2 - a2*b1);

    return Point(x,y);
}




double find_distance(Point p0, Point p1, Point p2, double minR)
{
    double m1, m2;
    // Compute m
    m1 = Line::compute_m(p0,p1);
    m2 = Line::compute_m(p1,p2);

    cout<<"m1: "<<m1<<" m2: "<<m2<<endl;

    //double tan_angle = (m1 - m2) / (1 + (m1 * m2));
    double angle = M_PI - abs(atan(m1) -  atan(m2));

    if (angle>M_PI){
        cout<<"IM HEEEEEEEEEEEREEEEE";
        angle = 2*M_PI - angle;
    }
    //double angle = atan(tan_angle);

    cout<<"angle between 2 lines: "<<angle<<endl;

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

    double x_entrance = x1 - distance * unit_v.x;
    double y_entrance = y1 - distance * unit_v.y;
    Point entrance = Point(x_entrance, y_entrance);

    return entrance;
};

Point find_exit(Point p0, Point p1, double distance)
{
    Point exit = find_entrance(p1, p0, distance);
    return exit;
}



// Point find_entrance(Point a, Point b, Point c, double minR){
//     cout<<"a: "<<a<<endl<<"b: "<<b<<endl<<"c: "<<c<<endl;
//     double a1,a2,b1,b2,c1,c2,c1_paral,c2_paral;
//     double m1, m2, q1, q2;
//     m1 = Line::compute_m(a,b);
//     m2 = Line::compute_m(b,c);

//     Line l1 = Line::get_line_from_m_and_p(a,m1);
//     Line l2 = Line::get_line_from_m_and_p(b,m2);

//     // q1 = Line::get_q_from_point(a, m1);
//     // q2 = Line::get_q_from_point(b, m2);
//     // Line l1 = Line(m2, q2);
//     // Line l2 = Line(m2, q2);

//     Line l1_paral = l1.find_parallel(minR);
//     Line l2_paral = l2.find_parallel(minR);

//     //find intersection of the parallels (center of circle)
//     Point circle_center = find_lines_intersection(l1_paral, l2_paral);

//     cout<<"circle center: "<<circle_center<<endl;

//     //find perpendicular lines
//     Line l1_perp = l1.find_perpendicular(circle_center);
//     Line l2_perp = l2.find_perpendicular(circle_center);

//     cout<<endl;
//     cout<<"RETTA 1: ";
//     cout<<l1;
//     cout<<"RETTA 1 paral: ";
//     cout<<l1_paral;
//     cout<<"RETTA 1 perp: ";
//     cout<<l1_perp;
//     cout<<"RETTA 2: ";
//     cout<<l2;
//     cout<<"RETTA 2 paral: ";
//     cout<<l2_paral;
//     cout<<"RETTA 2 perp: ";
//     cout<<l2_perp;    


//     Point entrance = find_lines_intersection(l1_perp, l1);
//     cout<<"entrance: "<<entrance<<endl;

//     return entrance;    
// }

// Point find_exit(Point a, Point b, Point c, double minR){
//    return find_entrance(c, b, a, minR);
// }

Polyline get_points_from_arc(Arc a, int npts){
    
    Polyline points;
    Point point;
    Pos pose;

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


//TODO: IF X1>X0
Polyline get_points_line(Point p0, Point p1){
    
    Polyline points;
    Line l = Line::get_line_from_points(p0, p1);

    
    cout<<"costruisco segment da punto "<<p0<<endl;
    cout<<"a punto "<<p1<<endl;

    //cout<<"m: "<<m<<endl;
    //cout<<"q: "<<q<<endl;
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

            Point p = Point(temp_x, temp_y);

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

            Point p = Point(temp_x, temp_y);

            points.push_back(p);

            break;
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
            Point p = Point(temp_x, temp_y);

            points.push_back(p);
        }
    }

    return points;

}


double compute_arc_length(Point a, Point b, double minR){
    
    double points_distance = sqrt(pow(b.x() - a.x(),2) + pow(b.y() - a.y(),2));
    //cout<<"COMPUTE ARC LENGHT of points: "<<a<<" and "<<b<<endl;
    //cout<<"points distance: "<<points_distance<<endl;
    double central_angle = acos(1 - (pow(points_distance,2)/(2*pow(minR, 2))));
    //cout<<"central angle: "<<central_angle<<endl;
    double L = minR * central_angle;

    return L;
}


double compute_angle(Point a, Point b){
    double m = Line::compute_m(a, b);
    return atan(m);
}

Arc get_arc(Point entrance, Point exit, double angle_entrance, double angle_exit, double minR){
    
    Arc arc;

    arc.x0 = entrance.x();
    arc.y0 = entrance.y();
    arc.th0 = angle_entrance;
    arc.xf = exit.x();
    arc.yf = exit.y();
    arc.thf = angle_exit;
    arc.k = 1/minR;
    arc.L = compute_arc_length(entrance, exit, minR);

    //cout<<"L arc: "<<arc.L<<endl;
    return arc;

}

//Environment get_environment(std::vector<Point>& map, std::vector<std::vector<Point>>& list_of_polygons){
    
//}

Environment get_environment1(){

    vector<Point> points_obs1;
    points_obs1.push_back(Point(4,4));
    points_obs1.push_back(Point(4, 8));
    points_obs1.push_back(Point(11, 8));
    points_obs1.push_back(Point(11, 4));
    Polygon obs1 = Polygon(points_obs1);

    vector<Point> points_obs2;
    points_obs2.push_back(Point(8, 10));
    points_obs2.push_back(Point(7, 12));
    points_obs2.push_back(Point(14, 12));
    points_obs2.push_back(Point(14, 10));
    Polygon obs2 = Polygon(points_obs2);

    vector<Point> points_obs3;
    points_obs3.push_back(Point(13, 6));
    points_obs3.push_back(Point(13, 8));
    points_obs3.push_back(Point(16, 8));
    points_obs3.push_back(Point(16, 6));
    Polygon obs3 = Polygon(points_obs3);

    vector<Point> points_env;
    points_env.push_back(Point(-20.0, -20.0));
    points_env.push_back(Point(20.0, -20.0));
    points_env.push_back(Point(20.0, 20.0));
    points_env.push_back(Point(-20.0, 20.0));

    Environment poly_env = Environment(points_env);

    poly_env.add_hole(obs1);
    poly_env.add_hole(obs2);
    poly_env.add_hole(obs3);

    // vector<Polygon> obstacles;
    // obstacles.push_back(poly_env);
    // obstacles.push_back(obs1);
    // obstacles.push_back(obs2);
    // obstacles.push_back(obs3);

    return poly_env;
}

Environment get_environment2(){

    vector<Point> points_obs1;
    points_obs1.push_back(Point(-3, 2));
    points_obs1.push_back(Point(-3, 3));
    points_obs1.push_back(Point(-2, 3));
    points_obs1.push_back(Point(-2, 2));
    Polygon obs1 = Polygon(points_obs1);

    vector<Point> points_obs2;
    points_obs2.push_back(Point(-5, 4));
    points_obs2.push_back(Point(-5, 5));
    points_obs2.push_back(Point(1, 5));
    points_obs2.push_back(Point(1, 4));
    Polygon obs2 = Polygon(points_obs2);

    // vector<Point> points_obs3;
    // points_obs3.push_back(Point(13, 6));
    // points_obs3.push_back(Point(13, 8));
    // points_obs3.push_back(Point(16, 8));
    // points_obs3.push_back(Point(16, 6));
    // Polygon obs3 = Polygon(points_obs3);

    vector<Point> points_env;
    points_env.push_back(Point(-20.0, -20.0));
    points_env.push_back(Point(20.0, -20.0));
    points_env.push_back(Point(20.0, 20.0));
    points_env.push_back(Point(-20.0, 20.0));

    Environment poly_env = Environment(points_env);

    poly_env.add_hole(obs1);
    poly_env.add_hole(obs2);
    // poly_env.add_hole(obs3);

    // vector<Polygon> obstacles;
    // obstacles.push_back(poly_env);
    // obstacles.push_back(obs1);
    // obstacles.push_back(obs2);
    // obstacles.push_back(obs3);

    return poly_env;
}


Curve get_first_trait_dubins(Polyline shortest_path, double th0, double minR){

    //first 3 vertex (need them to do dubins)
    Point start = shortest_path[0];
    Point p0 = shortest_path[1];
    Point p1 = shortest_path[2];
    //Find final angle of first trait
    double th1 = compute_angle(p0, p1);

    cout<<"ANGLE BEFORE: "<<th1<<endl;
    cout<<"I'm doing the opposite"<<endl;
    th1 = th1 + M_PI;
    cout<<"ANGLE AFTER: "<<th1<<endl;

    double Kmax = 1/minR;
    Curve first_trait = dubins_shortest_path(start.x(), start.y(), th0, p0.x(), p0.y(), th1, Kmax);

    return first_trait;
}


//TO DO: TH0
Curve get_last_trait_dubins(Polyline shortest_path, double thf, double minR){


    //BUILD DUBINS LAST TRAIT
    //last 3 vertex (need them to build dubins last trait)
    Point pn_1 = shortest_path[shortest_path.size()-3];
    Point pn = shortest_path[shortest_path.size()-2];
    Point goal = shortest_path[shortest_path.size()-1];
    double th_n;
    if(shortest_path.size()==3){
        th_n = compute_angle(pn, goal);
    }
    else{
        th_n = compute_angle(pn_1, pn);
    }

    double Kmax = 1/minR;

    Curve last_trait = dubins_shortest_path(pn.x(), pn.y(), th_n, goal.x(), goal.y(), thf, Kmax);

    return last_trait;
}


Polyline interpolation(Polyline shortest_path, double th0, double thf, double minR){

    Polyline points_final_path;

    //BUILD DUBINS FIRST TRAIT
    Curve first_trait = get_first_trait_dubins(shortest_path, th0, minR);
    Polyline points_first_trait = get_points_from_curve(first_trait, 300);

    //cout<<"i did first trait"<<endl;
    //DO INTERPOLATION

    // if(shortest_path.size()==3){
    //     Polyline points_line = get_points_line(shortest_path[1], shortest_path[2]);
    //     points_final_path.append(points_line);
    // }
    vector<Line> lines_vect;
    vector<Arc> arc_vect;

    for (int i = 1; i < shortest_path.size()-3; i=i+1)
    {
        Point a = shortest_path[i];
        Point b = shortest_path[i+1];
        Point c = shortest_path[i+2];

        double distance = find_distance(a,b,c,minR);

        Point entrance = find_entrance(a, b, distance);
        Point exit = find_exit(b, c, distance);

        cout<<"Entrance: "<<entrance<<endl;
        cout<<"Exit: "<<exit<<endl;

        //FOR PLOTTING
        //Polyline points_first = get_points_line(a, entrance);

        //cout<<"add path from "<<a<<" to "<<entrance<<endl; 

        //Polyline points_second = get_points_line(exit, c);

        //cout<<"add path from "<<exit<<" to "<<c<<endl; 

        double angle_entrance = compute_angle(a,b);
        double angle_exit = compute_angle(b,c);
    
        Arc arc = get_arc(entrance, exit, angle_entrance, angle_exit, minR);

        arc_vect.push_back(arc);
        
        //Polyline arc_points = get_points_from_arc(arc, 100);

        //points_final_path.append(points_first);
        //points_final_path.append(arc_points); 
        //points_final_path.append(points_second);
    }

    Curve last_trait = get_last_trait_dubins(shortest_path, thf, minR);
    Polyline points_last_trait = get_points_from_curve(last_trait, 300);
    points_final_path.append(points_first_trait);

    if (shortest_path.size() == 4){ 
        Point p1 = shortest_path[1];
        Point p2 = shortest_path[2];
        Polyline line = get_points_line(p1, p2);
        points_final_path.append(line);
    }

    else if (shortest_path.size() != 3){
        for(int i=1; i<shortest_path.size()-2;i++){
            if(i==1){
            
                Arc arc = arc_vect[0];

                Point p1 = shortest_path[1];
                Point p2 = Point(arc.x0, arc.y0);

                Polyline line_points = get_points_line(p1, p2);
                Polyline arc_points = get_points_from_arc(arc, 100);
                points_final_path.append(line_points);
            }
            
            else if(i==shortest_path.size()-3){

                Arc arc = arc_vect[i-2];
                Point p1 = Point(arc.xf, arc.yf);
                Point p2 = shortest_path[i+1];

                Polyline line = get_points_line(p1, p2);
                //Polyline arc_points = get_points_from_arc(arc, 100);
                //points_final_path.append(arc_points);
                points_final_path.append(line);            
            }

            else{
                Arc arc_first = arc_vect[i-2];
                Arc arc_second = arc_vect[i-1];
                Point p1 = Point(arc_first.xf, arc_first.yf);
                Point p2 = Point(arc_second.x0, arc_second.y0);

                Polyline arc_points = get_points_from_arc(arc_second, 100);
                Polyline line = get_points_line(p1, p2);
                points_final_path.append(line);     
                points_final_path.append(arc_points);
            }
            
        }
    }

    points_final_path.append(points_last_trait);

    return points_final_path;
}


// Polyline interpolation(Polyline shortest_path, double th0, double thf, double minR){

//     Polyline points_final_path;

//     //BUILD DUBINS FIRST TRAIT
//     Curve first_trait = get_first_trait_dubins(shortest_path, th0, minR);
//     Polyline points_first_trait = get_points_from_curve(first_trait, 300);

//     //DO INTERPOLATION
//     vector<Arc> arc_vect;
//     for (int i = 1; i < shortest_path.size()-3; i=i+1)
//     {
//         Point a = shortest_path[i];
//         Point b = shortest_path[i+1];
//         Point c = shortest_path[i+2];

//         Point entrance = find_entrance(a, b, c, minR);
//         Point exit = find_exit(a, b, c, minR);

//         //FOR PLOTTING
//         //Polyline points_first = get_points_line(a, entrance);

//         cout<<"add path from "<<a<<" to "<<entrance<<endl; 

//         //Polyline points_second = get_points_line(exit, c);

//         cout<<"add path from "<<exit<<" to "<<c<<endl; 

//         double angle_entrance = compute_angle(a,b);
//         double angle_exit = compute_angle(b,c);
    
//         Arc arc = get_arc(entrance, exit, angle_entrance, angle_exit, minR);

//         arc_vect.push_back(arc);
        
//         //Polyline arc_points = get_points_from_arc(arc, 100);

//         //points_final_path.append(points_first);
//         //points_final_path.append(arc_points); 
//         //points_final_path.append(points_second);
//     }

//     Curve last_trait = get_last_trait_dubins(shortest_path, thf, minR);
//     Polyline points_last_trait = get_points_from_curve(last_trait, 300);

//     points_final_path.append(points_first_trait);

//     for(int i=1; i<shortest_path.size()-2;i++){
//         if(i==1){
//             Point p1 = shortest_path[1];
//             Point p2 = Point(arc_vect[0].x0, arc_vect[0].y0);

//             Polyline line = get_points_line(p1, p2);
//             points_final_path.append(line);
//         }
//         else if(i==shortest_path.size()-3){
//             Point p1 = Point(arc_vect[i-2].xf, arc_vect[i-2].yf);
//             Point p2 = shortest_path[i+1];

//             Polyline line = get_points_line(p1, p2);
//             Polyline arc_points = get_points_from_arc(arc_vect[i-2], 100);
//             points_final_path.append(arc_points);
//             points_final_path.append(line);            
//         }
//         else{
//             Point p1 = Point(arc_vect[i-2].xf, arc_vect[i-2].yf);
//             Point p2 = Point(arc_vect[i-1].x0, arc_vect[i-1].y0);

//             Polyline arc_points = get_points_from_arc(arc_vect[i-2], 100);
//             Polyline line = get_points_line(p1, p2);
//             points_final_path.append(arc_points);
//             points_final_path.append(line);        
//         }
//     }

//     points_final_path.append(points_last_trait);

//     return points_final_path;
// }