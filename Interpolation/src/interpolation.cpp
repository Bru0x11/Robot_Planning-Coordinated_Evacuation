#include "include/interpolation.h"

/*
Defining the methods for the class Vector
*/
//Constructor
Vector::Vector(double x, double y){
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


/*
Defining the methods for the class Line
*/
Line::Line(double a, double b, double c){
    
    if(b == 0){
        this->vertical = true;
        this->a = a;
        this->b = b;
        this->c = c;
        this->m = DBL_MAX;
        this->q = -c/a;
    }
    else{
        if(a == 0){
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


    if(m == DBL_MAX){
        this->vertical = true;
        this->a=1;
        this->c=-q;
        this->b=0;
    }
    else{
        if(m == 0){
            this->horizontal = true;
        }

        this->a = -m;
        this->b = 1;
        this->c = -q;
    }

}

Line Line::findParallel(double distance){
    double c_new;

    if(this->vertical){
        c_new = this->c + distance;
        return Line(this->a, this->b, c_new);
    }

    c_new = this->c - ((sqrt(pow(this->a,2) + pow(this->b,2)))*distance);
    return Line(this->a, this->b, c_new);
}

Line Line::findPerpendicular(VisiLibity::Point point){
    if(this->vertical){
        return Line(0, point.y());
    }

    if(this->horizontal){
        return Line(1, 0, -point.x());
    }
    
    double mp = -1/(this->m);
    //double qp = Line::get_q_from_point(p,mp);

    return Line::getLineFromMAndP(point, mp);
}

Line Line::getLineFromMAndP(VisiLibity::Point point, double m){
    if(m == DBL_MAX){
        return Line(1, 0, -point.x());
    }

    if(m == 0){
        return Line(0, 1, -point.y());
    }

    double q = point.y() - m*point.x();
    return Line(m, q);
}

Line Line::getLineFromPoints(VisiLibity::Point point1, VisiLibity::Point point2){
    double m,q;

    if(point1.x() == point2.x()){
        m = DBL_MAX;
        q = point1.y();
        return Line(m, q);
    }

    m = computeM(point1, point2);

    return getLineFromMAndP(point1, m);
}

double Line::getQFromPoint(VisiLibity::Point point, double m){
    if(m == DBL_MAX){
        return point.x();
    }

    if(m == 0){
        return point.y();
    }

    return point.y() - m*point.x();
}

double Line::computeM(VisiLibity::Point point0, VisiLibity::Point point1){
    double x1 = point1.x();
    double x0 = point0.x();

    if ((x1-x0) != 0){
        double y1,y0;

        y1 = point1.y();
        y0 = point0.y();

        return (y1-y0) / (x1-x0);
    }

    return DBL_MAX;
}


ostream & operator << (ostream &out, Line l){
    out<<l.a<<"x+"<<l.b<<"y+"<<l.c<<"=0"<<endl;
    return out;
}


VisiLibity::Point findLinesIntersection(Line line1, Line line2){
    double a1 = line1.a;
    double b1 = line1.b; 
    double c1 = line1.c;
    double a2 = line2.a;
    double b2 = line2.b;
    double c2 = line2.c;
    double x = (b1*c2 - b2*c1)/(a1*b2 - a2*b1);
    double y = (c1*a2 - c2*a1)/(a1*b2 - a2*b1);

    return VisiLibity::Point(x,y);
}

double findDistance(VisiLibity::Point point0,VisiLibity::Point point1, VisiLibity::Point point2, double minimumCurvatureRadius){

    Vector vector1 = Vector(point1.x() - point0.x(), point1.y() - point0.y());
    Vector vector2 = Vector(point2.x() - point1.x(), point2.y() - point1.y());

    double angle = Vector::angle(vector1, vector2);

    angle = M_PI - angle; 

    double distance = minimumCurvatureRadius * (1 / tan(angle / 2));

    return abs(distance);
}

VisiLibity::Point findEntrance(VisiLibity::Point point0, VisiLibity::Point point1, double minimumCurvatureRadius){
    double x0 = point0.x();
    double y0 = point0.y();
    double x1 = point1.x();
    double y1 = point1.y();

    Vector vector = Vector(x1 - x0, y1 - y0);
    double vectorNorm = vector.norm();

    double unit_v_x = vector.x / vectorNorm;
    double unit_v_y = vector.y / vectorNorm;
    Vector unit_v = Vector(unit_v_x, unit_v_y);

    double x_entrance = x1 - minimumCurvatureRadius * unit_v.x;
    double y_entrance = y1 - minimumCurvatureRadius * unit_v.y;
    VisiLibity::Point entrance = VisiLibity::Point(x_entrance, y_entrance);

    return entrance;
}

VisiLibity::Point findExit(VisiLibity::Point point0, VisiLibity::Point point1, double minimumCurvatureRadius){
    VisiLibity::Point exit = findEntrance(point1, point0, minimumCurvatureRadius);
    return exit;
}

Polyline getPointsFromArc(Arc arc, int npts){
    
    Polyline points;
    VisiLibity::Point point;
    Pos pose;

    double k = arc.k;
    double true_x = arc.xf;
    double true_y = arc.yf;

    Pos temp = circline(arc.L, arc.x0, arc.y0, arc.th0, arc.k);

    double epsilon = 0.01;

    if((abs(temp.x - true_x)>epsilon) || (abs(temp.y - true_y) > epsilon)){
        arc.k = -k;
    }

    for (int j = 0; j < npts; j++){

        double s = arc.L/npts * j;

        pose = circline(s, arc.x0, arc.y0, arc.th0, arc.k);

        point.set_x(pose.x);
        point.set_y(pose.y);

        points.push_back(point);
     }

    return points; 
}

Polyline getPointsFromCurve(Curve curve, int npts){

    int nptsArc = (int)(npts/3); 

    Arc arc1 = curve.a1;
    Arc arc2 = curve.a2;
    Arc arc3 = curve.a3;

    Polyline line1 = getPointsFromArc(arc1, nptsArc);
    Polyline line2 = getPointsFromArc(arc2, nptsArc);
    Polyline line3 = getPointsFromArc(arc3, nptsArc);
    
    Polyline totLine;
    totLine.append(line1);
    totLine.append(line2);
    totLine.append(line3);

    return totLine;
}


Polyline getPointsLine(VisiLibity::Point point0, VisiLibity::Point point1){
    
    Polyline points;
    Line line = Line::getLineFromPoints(point0, point1);

    double x0 = point0.x();
    double x1 = point1.x();

    if(x1 > x0){
        for(int i = 1; i < 100; i++){
            double gamma = i*0.01;
            double temp_x = (1-gamma)*x0 + (gamma*x1);
            double temp_y = line.m*temp_x + line.q;

            VisiLibity::Point p =VisiLibity::Point(temp_x, temp_y);

            points.push_back(p);
        }
    }else if(x1 < x0){
        for(int i = 1; i < 100; i++){
            double gamma = i*0.01;
            double temp_x = (1-gamma)*x0 + (gamma*x1);
            double temp_y = line.m*temp_x + line.q;

            VisiLibity::Point p = VisiLibity::Point(temp_x, temp_y);

            points.push_back(p);
        }
    }else{
        double temp_x= point0.x();
        for(int i = 1; i < 100; i++){
            double gamma = i*0.01;
            double temp_y = (1-gamma)*point0.y() + (gamma*point1.y());
            VisiLibity::Point p = VisiLibity::Point(temp_x, temp_y);

            points.push_back(p);
        }
    }

    return points;
}


double computeArcLength(VisiLibity::Point point0, VisiLibity::Point point1, double minimumCurvatureRadius){
    
    double pointsDistance = sqrt(pow(point1.x() - point0.x(), 2) + pow(point1.y() - point0.y(), 2));

    double centralAngle = acos(1 - (pow(pointsDistance, 2)/(2*pow(minimumCurvatureRadius, 2))));
    double length = minimumCurvatureRadius * centralAngle;

    return length;
}


double computeAngle(VisiLibity::Point point0, VisiLibity::Point point1){

    Vector vector1 = Vector(point1.x() - point0.x(), point1.y() - point0.y());
    Vector vector2 = Vector(1, 0);

    double angle = Vector::angle(vector1, vector2);

    if(vector1.y < 0){
        angle = -angle;
    }

    return angle;
}

Arc getArc(VisiLibity::Point entrance, VisiLibity::Point exit, double angleEntrance, double angleExit, double minimumCurvatureRadius){
    
    Arc arc;

    arc.x0 = entrance.x();
    arc.y0 = entrance.y();
    arc.th0 = angleEntrance;
    arc.xf = exit.x();
    arc.yf = exit.y();
    arc.thf = angleExit;
    arc.k = 1/minimumCurvatureRadius;
    arc.L = computeArcLength(entrance, exit, minimumCurvatureRadius);

    return arc;
}

Environment getEnvironment(){

    vector<VisiLibity::Point> points_obs1;

    //Example 1
    // points_obs1.push_back(VisiLibity::Point(-4, -3));
    // points_obs1.push_back(VisiLibity::Point(-4, -1.5));
    // points_obs1.push_back(VisiLibity::Point(3, -1.5));
    // points_obs1.push_back(VisiLibity::Point(3, -3));

    //Example 2
    // points_obs1.push_back(VisiLibity::Point(-6, 2));
    // points_obs1.push_back(VisiLibity::Point(-6, 6));
    // points_obs1.push_back(VisiLibity::Point(0, 6));

    //Example 3
    points_obs1.push_back(VisiLibity::Point(-6.8,  -7.1));
    points_obs1.push_back(VisiLibity::Point(6, -1.6));
    points_obs1.push_back(VisiLibity::Point(6, -7.1));

    Polygon obs1 = Polygon(points_obs1);

    vector<VisiLibity::Point> points_obs2;

    //Example 1
    // points_obs2.push_back(VisiLibity::Point(-2, 1));
    // points_obs2.push_back(VisiLibity::Point(-2, 3));
    // points_obs2.push_back(VisiLibity::Point(4, 3));
    // points_obs2.push_back(VisiLibity::Point(4, 1));

    //Example 2
    // points_obs2.push_back(VisiLibity::Point(2, -3.15));
    // points_obs2.push_back(VisiLibity::Point(2, 4));
    // points_obs2.push_back(VisiLibity::Point(6, 2));
    // points_obs2.push_back(VisiLibity::Point(6, -3.15));

    //Example 3
    points_obs2.push_back(VisiLibity::Point(-4, 0));
    points_obs2.push_back(VisiLibity::Point(-7.3, 2));
    points_obs2.push_back(VisiLibity::Point(-7.6, 4));
    points_obs2.push_back(VisiLibity::Point(-2, 4));
    points_obs2.push_back(VisiLibity::Point(-0.5, 2));
    
    Polygon obs2 = Polygon(points_obs2);

    vector<VisiLibity::Point> points_obs3;

    //Example 1
    // points_obs3.push_back(VisiLibity::Point(-4, 1));
    // points_obs3.push_back(VisiLibity::Point(-4, 5));
    // points_obs3.push_back(VisiLibity::Point(-1.5, 2));

    //Example 2
    // points_obs3.push_back(VisiLibity::Point(-6, -3.2));
    // points_obs3.push_back(VisiLibity::Point(-4, 2.8));
    // points_obs3.push_back(VisiLibity::Point(-2, -3.2));

    //Example 3
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
    poly_env.add_hole(obs3);
    poly_env.add_hole(obs2);

    //prova
    return poly_env;
}

Environment getOffsettedEnvironment(Environment environment, double minimumCurvatureRadius, double robotSize){
    //For printing purposes
    FillRule fr = FillRule::EvenOdd;
    SvgWriter svg;

    //Save all the polygon in this variable
    vector<PathsD> polygons;

    //------------------MAP------------------
    vector<VisiLibity::Point> boundaryPoints;
    Polygon boundary = environment[0];

    //Extrapolating the points
    for(int j = 0; j < boundary.n(); j++){
        VisiLibity::Point point = boundary[j];
        boundaryPoints.push_back(point);
    }

    PathsD polygon = createPolygon(boundaryPoints);

    PathsD offsettedPolygon = offsetPolygon(polygon, -0.5, true);

    polygons.push_back(offsettedPolygon);   

    //-----------------OBSTACLES----------------
    for(int i = 1; i <= environment.h(); i++){
        
        Polygon poly = environment[i];
        vector<VisiLibity::Point> polygonPoints;
        
        //Extrapolating the points
        for(int j = 0; j < poly.n(); j++){
            VisiLibity::Point point = poly[j];
            polygonPoints.push_back(point);
        }

        //Computing the right offset for the polygon
        double minAngle = findMinAngle(poly);
        float offsetValue = (float)offsetCalculator(minAngle, minimumCurvatureRadius, robotSize);

        PathsD polygon = createPolygon(polygonPoints);
        PathsD offsettedPolygon = offsetPolygon(polygon, offsetValue, false);

        checkIntersections(offsettedPolygon, polygons);
    }

    //Putting the map in the first spot. This is necessary for the translation part
    int mapPosition = 0;
    for(int i = 0; i < polygons.size(); i++){
        if (polygons[i].size() == 2){
            mapPosition = i;
        }
    }
    iter_swap(polygons.begin(), polygons.begin() + mapPosition);

    //Printing useful informations
    cout << "SIZE OF POLYGONS: " << polygons.size() << '\n';
    cout << "LISTING ALL THE POLYGONS WE HAVE:\n";
    for(int i = 0; i < polygons.size(); i++){
        cout << polygons[i] << '\n';
        svg.AddPaths(polygons[i], false, fr, 0x10AA66FF, 0xAA0066FF, 1, false);
    }   
    svg.SaveToFile("sample_map.svg", 800, 600, 0);

    //-----------------TRANSLATION----------------
    vector<Polygon> translatedPolygons;

    for(int i = 0; i < polygons.size(); i++){
        //The first has to be necessarily the map
        if(i == 0){
            PathsD map = polygons[0]; 
            vector<VisiLibity::Point> translatedMapPoints;

            for (PointD point : map[1]){
                translatedMapPoints.push_back(VisiLibity::Point(point.x, point.y));
            }

            Polygon translatedMap = Polygon(translatedMapPoints);
            translatedPolygons.push_back(translatedMap);
        }else{
            vector<VisiLibity::Point> polygonPoints = translatePolygon(polygons[i]);
            Polygon poly = Polygon(polygonPoints);
            translatedPolygons.push_back(poly);
        }
    }

    Environment translatedEnvironment = Environment(translatedPolygons);
    return translatedEnvironment; 
}

Curve getFirstTraitDubins(Polyline shortestPath, double theta0, double minimumCurvatureRadius){
    //Take the first three point to compute dubins
    VisiLibity::Point start = shortestPath[0];
    VisiLibity::Point point0 = shortestPath[1];
    VisiLibity::Point point1 = shortestPath[2];
    //Find final angle of first trait
    double theta1 = computeAngle(point0, point1);

    double Kmax = 1/minimumCurvatureRadius;
    Curve firstTrait = dubins_shortest_path(start.x(), start.y(), theta0, point0.x(), point0.y(), theta1, Kmax);

    return firstTrait;
}


Curve getLastTraitDubins(Polyline shortestPath, double thetaF, double minimumCurvatureRadius){
    //Take the first three point to compute dubins
    VisiLibity::Point finalpoint0 = shortestPath[shortestPath.size()-3];
    VisiLibity::Point finalpoint1 = shortestPath[shortestPath.size()-2];
    VisiLibity::Point goal = shortestPath[shortestPath.size()-1];
    double thetaN;

    thetaN = computeAngle(finalpoint0, finalpoint1);
        
    double Kmax = 1/minimumCurvatureRadius;

    Curve lastTrait = dubins_shortest_path(finalpoint1.x(), finalpoint1.y(), thetaN, goal.x(), goal.y(), thetaF, Kmax);

    return lastTrait;
}

double findMinAngle(VisiLibity::Polygon points){
    double minAngle = 5;
    int index1, index2, index3;

    for (int i = 0; i < points.n(); i++) {
        if(i == points.n() - 1){
            index1 = i-1;
            index2 = i;
            index3 = 0;
        } if (i == 0){
            index1 = points.n() - 1;
            index2 = i;
            index3 = i+1;
        }

        Vector vector1 = Vector(points[index2].x() - points[index1].x(), points[index2].y() - points[index1].y());
        Vector vector2 = Vector(points[index2].x() - points[index3].x(), points[index2].y() - points[index3].y());
        double angle = Vector::angle(vector1,vector2);
        if (minAngle > angle){
            minAngle = angle;
        }
    }
    return minAngle;
}

double offsetCalculator(double minAngle, double minimumCurvatureRadius, double robotSize){
    double offset;
    offset = std::max(minimumCurvatureRadius * (1 - sin(minAngle/2)) + robotSize * sin(minAngle/2), robotSize);
    return offset;
}

int environmentHoles(VisiLibity::Environment environment){
    double angle;
    double offset;

    for(int i = 1 ; i < environment.h() + 1 ; i++){
        angle = findMinAngle(environment[i]);
        offset = offsetCalculator(angle, 1.5, 0.5);
        cout<< "hole "<< i << " min angle: " << angle <<": \n";
        cout<< "hole "<< i << " offset: " << offset <<": \n";
    }
    return 0;
}

vector<Arc> getArcsInterpolation(Polyline shortestPath, double minimumCurvatureRadius){
    vector<Arc> arcVector;

    for (int i = 1; i < shortestPath.size()-3; i++)
    {
        VisiLibity::Point point0 = shortestPath[i];
        VisiLibity::Point point1 = shortestPath[i+1];
        VisiLibity::Point point2 = shortestPath[i+2];

        double distance = findDistance(point0, point1, point2, minimumCurvatureRadius);

        VisiLibity::Point entrance = findEntrance(point0, point1, distance);
        VisiLibity::Point exit = findExit(point1, point2, distance);

        double angleEntrance = computeAngle(point0,point1);
        double angleExit = computeAngle(point1,point2);    
    
        Arc arc = getArc(entrance, exit, angleEntrance, angleExit, minimumCurvatureRadius);
        arcVector.push_back(arc);
    }
    return arcVector;
}

Polyline interpolation(Polyline shortestPath, double theta0, double thetaF, double minimumCurvatureRadius){
    Polyline pointsFinalPath;
    double kmax = 1/minimumCurvatureRadius;

    if(shortestPath.size() == 2){
        VisiLibity::Point start = shortestPath[0];
        VisiLibity::Point end = shortestPath[1];

        Curve dubinsPath = dubins_shortest_path(start.x(), start.y(), theta0, end.x(), end.y(), thetaF, kmax);
        pointsFinalPath.append(getPointsFromCurve(dubinsPath, 200));

        return pointsFinalPath;
    }else if(shortestPath.size() == 3){
        VisiLibity::Point start = shortestPath[0];
        VisiLibity::Point middle = shortestPath[1];
        VisiLibity::Point end = shortestPath[2];
        
        double thetaMiddle = computeAngle(middle, end);

        Curve firstTrait = dubins_shortest_path(start.x(), start.y(), theta0, middle.x(), middle.y(), thetaMiddle, kmax);
        Curve secondTrait = dubins_shortest_path(middle.x(), middle.y(), thetaMiddle, end.x(), end.y(), thetaF, kmax);

        pointsFinalPath.append(getPointsFromCurve(firstTrait, 100));
        pointsFinalPath.append(getPointsFromCurve(secondTrait, 100));

        return pointsFinalPath;
    }else if(shortestPath.size() == 4){
        VisiLibity::Point start = shortestPath[0];
        VisiLibity::Point middle1 = shortestPath[1];
        VisiLibity::Point middle2 = shortestPath[2];
        VisiLibity::Point end = shortestPath[3];
        
        double thetaM1M2 = computeAngle(middle1, middle2);
        Curve firstTrait = dubins_shortest_path(start.x(), start.y(), theta0, middle1.x(), middle1.y(), thetaM1M2, kmax);

        Polyline middleLine = getPointsLine(middle1, middle2);
        Curve lastTrait = dubins_shortest_path(middle2.x(), middle2.y(), thetaM1M2, end.x(), end.y(), thetaF, kmax);

        pointsFinalPath.append(getPointsFromCurve(firstTrait, 100));
        pointsFinalPath.append(middleLine);
        pointsFinalPath.append(getPointsFromCurve(lastTrait, 100));

        return pointsFinalPath;
    }else{
        Curve firstTrait = getFirstTraitDubins(shortestPath, theta0, minimumCurvatureRadius);
        pointsFinalPath.append(getPointsFromCurve(firstTrait, 100));

        vector<Arc> arcVector = getArcsInterpolation(shortestPath, minimumCurvatureRadius);

        VisiLibity::Point point1 = shortestPath[1];
        Arc arc0 = arcVector[0];
        VisiLibity::Point point2 = VisiLibity::Point(arc0.x0, arc0.y0);     
                
        pointsFinalPath.append(getPointsLine(point1, point2));
        pointsFinalPath.append(getPointsFromArc(arc0, 100));

        for(int i = 2; i < shortestPath.size()-3; i++){
            Arc firstArc = arcVector[i-2];
            Arc secondArc = arcVector[i-1];

            VisiLibity::Point firstPoint = VisiLibity::Point(firstArc.xf, firstArc.yf);
            VisiLibity::Point secondPoint = VisiLibity::Point(secondArc.x0, secondArc.y0);
            
            pointsFinalPath.append(getPointsLine(firstPoint, secondPoint));
            pointsFinalPath.append(getPointsFromArc(secondArc, 100));
        }

        Arc lastArc = arcVector[arcVector.size()-1];
        VisiLibity::Point finalPoint1 = VisiLibity::Point(lastArc.xf, lastArc.yf);
        VisiLibity::Point finalPoint2 = shortestPath[shortestPath.size() - 2];

        pointsFinalPath.append(getPointsLine(finalPoint2, finalPoint1));

        Curve lastTrait = getLastTraitDubins(shortestPath, thetaF, minimumCurvatureRadius);
        pointsFinalPath.append(getPointsFromCurve(lastTrait, 100));

        return pointsFinalPath;
    }

    return pointsFinalPath;
}