#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
using namespace std::chrono_literals;

#include <iostream>
#include <cstdlib>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include "dubins.h"


#include "visilibity.hpp"

using namespace std;


//Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t
//otherwise

double sinc(double t){

    double s;
    if(abs(t) < 0.002){
        s = 1 - pow(t,2)/6 * (1 - pow(t,2)/20);
    }
    else{
        s = sin(t)/t;
    }

    return s;
}


//Normalize an angle (in range [0,2*pi))
double mod2pi(double ang){
    double out = ang;
    while(out<0){
        out = out + (2 * M_PI);
    }

    while(out >= (2 * M_PI)){
        out = out - (2 * M_PI); 
    }

    return out;
}

//Normalize an angular difference (range (-pi, pi])
double rangeSymm(double ang){
  double out = ang;
  while (out <= - M_PI){
    out = out + 2 * M_PI;
  }
  while (out > M_PI){
    out = out - 2 * M_PI;
  }

  return out;
}


//Check validity of a solution by evaluating explicitly the 3 equations 
//defining a Dubins problem (in standard form)
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf){
  double x0 = -1;
  double y0 = 0;
  double xf = 1;
  double yf = 0;

  double eq1 = x0 + s1 * sinc((1/2.) * k0 * s1) * cos(th0 + (1/2.) * k0 * s1) + s2 * sinc((1/2.) * k1 * s2) * cos(th0 + k0 * s1 + (1/2.) * k1 * s2) + s3 * sinc((1/2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - xf;
  double eq2 = y0 + s1 * sinc((1/2.) * k0 * s1) * sin(th0 + (1/2.) * k0 * s1) + s2 * sinc((1/2.) * k1 * s2) * sin(th0 + k0 * s1 + (1/2.) * k1 * s2) + s3 * sinc((1/2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - yf;
  double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

  bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
  bool bol = (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;

  return bol;
}


/////////////////////////////////////////////////////////////////////
//Functions to scale and solve Dubins problems
/////////////////////////////////////////////////////////////////////

//Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
//return a double[] array containing [sc_th0, sc_thf, sc_Kmax, lambda]
double* scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax){
  //find transform parameters
  double dx = xf - x0;
  double dy = yf - y0;
  double phi = atan2(dy, dx);
  double lambda = hypot(dx, dy)/2;

  //scale and normalize angles and curvature
  double sc_th0 = mod2pi(th0 - phi);
  double sc_thf = mod2pi(thf - phi);
  double sc_Kmax = Kmax * lambda;

  static double standard[4];
  standard[0] = sc_th0;
  standard[1] = sc_thf;
  standard[2] = sc_Kmax;
  standard[3] = lambda;
  return standard;

}


//Scale the solution to the standard problem back to the original problem
//return a pointer of double containing {s1, s2, s3}
double* scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3){
  //cout<<endl<<"Before scaling: "<<sc_s1<<endl<<sc_s2<<endl<<sc_s3<<endl; 
  double s1 = sc_s1 * lambda;
  double s2 = sc_s2 * lambda;
  double s3 = sc_s3 * lambda;
  static double scaled[3];
  scaled[0] = s1;
  scaled[1] = s2;
  scaled[2] = s3;
  return scaled;
}

//LSL
Solution LSL(double sc_th0, double sc_thf, double sc_Kmax){
  double invK = 1 / sc_Kmax;
  double C = cos(sc_thf) - cos(sc_th0);
  double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(C, S);
  double sc_s1 = invK * mod2pi(temp1 - sc_th0);
  double temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
  Solution sol;
  if (temp2 < 0){
    sol.ok = false; 
    sol.sc_s1 = 0; 
    sol.sc_s2 = 0; 
    sol.sc_s3 = 0;
    return sol;
  }
  sol.sc_s1 = sc_s1;
  sol.sc_s2 = invK * sqrt(temp2);
  sol.sc_s3 = invK * mod2pi(sc_thf - temp1);
  sol.ok = true;
  return sol;
}

//RSR
Solution RSR(double sc_th0, double sc_thf, double sc_Kmax){
  double invK = 1 / sc_Kmax;
  double C = cos(sc_th0) - cos(sc_thf);
  double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(C, S);
  double sc_s1 = invK * mod2pi(sc_th0 - temp1);
  double temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
  Solution sol;
  if (temp2 < 0){
    sol.ok = false; 
    sol.sc_s1 = 0; 
    sol.sc_s2 = 0; 
    sol.sc_s3 = 0;
    return sol;
  }
  sol.sc_s1 = sc_s1;
  sol.sc_s2 = invK * sqrt(temp2);
  sol.sc_s3 = invK * mod2pi(temp1 - sc_thf);
  sol.ok = true;
  return sol;
}


//LSR
Solution LSR(double sc_th0, double sc_thf, double sc_Kmax){
  double invK = 1 / sc_Kmax;
  double C = cos(sc_th0) + cos(sc_thf);
  double S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(-C, S);
  double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
  Solution sol;
  if (temp3 < 0){
    sol.ok = false; 
    sol.sc_s1 = 0; 
    sol.sc_s2 = 0; 
    sol.sc_s3 = 0;
    return sol;
  }
  double sc_s2 = invK * sqrt(temp3);
  sol.sc_s2 = sc_s2;
  double temp2 = -atan2(-2, sc_s2 * sc_Kmax);
  sol.sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
  sol.sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
  sol.ok = true;
  return sol;
}


//RSL
Solution RSL(double sc_th0, double sc_thf, double sc_Kmax){
  double invK = 1 / sc_Kmax;
  double C = cos(sc_th0) + cos(sc_thf);
  double S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(C, S);
  double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
  Solution sol;
  if (temp3 < 0){
    sol.ok = false; 
    sol.sc_s1 = 0; 
    sol.sc_s2 = 0; 
    sol.sc_s3 = 0;
    return sol;
  }
  double sc_s2 = invK * sqrt(temp3);
  sol.sc_s2 = sc_s2;
  double temp2 = atan2(2, sc_s2 * sc_Kmax);
  sol.sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
  sol.sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
  sol.ok = true;
  return sol;
}


//RLR
Solution RLR(double sc_th0, double sc_thf, double sc_Kmax){
  double invK = 1 / sc_Kmax;
  double C = cos(sc_th0) - cos(sc_thf);
  double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
  double temp1 = atan2(C, S);
  double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
  Solution sol;
  if (abs(temp2) > 1){
    sol.ok = false; 
    sol.sc_s1 = 0; 
    sol.sc_s2 = 0; 
    sol.sc_s3 = 0;
    return sol;
  }
  double sc_s2 = invK * mod2pi((2 * M_PI) - acos(temp2));
  sol.sc_s2 = sc_s2;
  double sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
  sol.sc_s1 = sc_s1;
  sol.sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
  sol.ok = true;
  return sol;
}


//LRL
Solution LRL(double sc_th0, double sc_thf, double sc_Kmax){
  double invK = 1 / sc_Kmax;
  double C = cos(sc_thf) - cos(sc_th0);
  double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
  double temp1 = atan2(C, S);
  double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
  Solution sol;
  if (abs(temp2) > 1){
    sol.ok = false; 
    sol.sc_s1 = 0; 
    sol.sc_s2 = 0; 
    sol.sc_s3 = 0;
    return sol;
  }
  double sc_s2 = invK * mod2pi((2 * M_PI) - acos(temp2));
  sol.sc_s2 = sc_s2;
  double sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
  sol.sc_s1 = sc_s1;
  sol.sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));
  sol.ok = true;
  return sol;
}

//Evaluate an arc (circular or straight) composing a Dubins curve, at a 
//given arc-length s

Pos circline(double s, double x0, double y0, double th0, double k){
    Pos p;
    p.x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2);
    p.y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2);
    p.th = mod2pi(th0 + k * s);
    return p;
}



//Create a structure representing an arc of a Dubins curve (straight or
//circular)
Arc dubinsarc(double x0, double y0, double th0, double k, double L){
    Arc c;
    c.x0 = x0;
    c.y0 = y0;
    c.th0 = th0;
    c.k = k;
    c.L = L;
    Pos p = circline(L, x0, y0, th0, k);
    c.xf = p.x;
    c.yf = p.y;
    c.thf = p.th;

    return c;
}


Curve dubinscurve(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2){
        Curve c;
        c.a1 = dubinsarc(x0,y0,th0,k0,s1);
        c.a2 = dubinsarc(c.a1.xf, c.a1.yf, c.a1.thf, k1, s2);
        c.a3 = dubinsarc(c.a2.xf, c.a2.yf, c.a2.thf, k2, s3);
        c.L = c.a1.L + c.a2.L + c.a3.L;
        return c;
}


//Solve the Dubins problem for the given input parameters.
//Return the type and the parameters of the optimal curve
Curve dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax){
  //Compute params of standard scaled problem
  double* standard;
  standard = scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax);

  double sc_th0 = standard[0];
  double sc_thf = standard[1];
  double sc_Kmax = standard[2];
  double lambda = standard[3];

  //cout<<"HERE1"<<endl<<sc_th0<<endl<<sc_thf<<endl<<sc_Kmax<<endl<<lambda<<endl;

  int ksigns[6][3] = {
              {1,  0,  1}, //  LSL
              {-1,  0, -1}, // RSR
              {1,  0, -1}, // LSR
              {-1,  0,  1}, // RSL
              {-1,  1, -1}, // RLR
              {1, -1,  1}, // LRL
              };

  Solution sol [6];
  sol[0] = LSL(sc_th0, sc_thf, sc_Kmax);
  sol[1] = RSR(sc_th0, sc_thf, sc_Kmax);
  sol[2] = LSR(sc_th0, sc_thf, sc_Kmax);
  sol[3] = RSL(sc_th0, sc_thf, sc_Kmax);
  sol[4] = RLR(sc_th0, sc_thf, sc_Kmax);
  sol[5] = LRL(sc_th0, sc_thf, sc_Kmax);

  //Try all the possible primitives, to find the optimal solution
  int pidx = -1;
    
  bool ok;
  double sc_s1_c = sol[0].sc_s1;
  double sc_s2_c = sol[0].sc_s2;
  double sc_s3_c = sol[0].sc_s3;

  double L = sc_s1_c + sc_s2_c + sc_s3_c;
  
  double Lcur;
  double sc_s1;
  double sc_s2;
  double sc_s3;

  for (int i = 0; i < 6; i++){
    Solution s = sol[i];
    ok = s.ok;
    sc_s1_c = s.sc_s1;
    sc_s2_c = s.sc_s2;
    sc_s3_c = s.sc_s3;
    Lcur = sc_s1_c + sc_s2_c + sc_s3_c;

    if (ok && Lcur<=L){
      L = Lcur;
      sc_s1 = sc_s1_c;
      sc_s2 = sc_s2_c;
      sc_s3 = sc_s3_c;
      pidx = i;
    }

  
   }

  Curve curve;
  if (pidx >= 0) {
    //Transform the solution to the problem in standard form to the 
    //solution of the original problem (scale the lengths)  
    double* original;

    original = scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3);
    double s1 = original[0];
    double s2 = original[1];
    double s3 = original[2];

    //cout<<ksigns[pidx][1];

    double k0 = ksigns[pidx][0]*Kmax;
    double k1 = ksigns[pidx][1]*Kmax;
    double k2 = ksigns[pidx][2]*Kmax;
    //Construct the Dubins curve object with the computed optimal parameters
    curve = dubinscurve(x0, y0, th0, s1, s2, s3, k0, k1, k2);
    
    //Check the correctness of the algorithm //TOO
    //assert(check(sc_s1, ksigns(pidx,1)*sc_Kmax, sc_s2, ksigns(pidx,2)*sc_Kmax, sc_s3, ksigns(pidx,3)*sc_Kmax, sc_th0, sc_thf));
  }

  Curve dubins_curve = curve;

  return dubins_curve;

}

vector<Curve> multipoint_dubins(VisiLibity::Polyline points, double Kmax){

  
  VisiLibity::Point start_point = points[0];
  
  double x0 = start_point.x();
  double y0 = start_point.y();
  double th0 = 0; //INITIAL ANGLE

  double xf;
  double yf;
  double thf;

  double x_next;
  double y_next;

  double m;

  vector<Curve> dubins_curves;

  int points_size = points.size();

  for(int i = 1; i < points_size; i++){
    xf = points[i].x();
    yf = points[i].y();
    
    if(i != (points.size() - 1)){
      x_next = points[i+1].x();
      y_next = points[i+1].y();
      m = (y_next - yf)/(x_next - xf);
      thf = atan(m);
    }
    else{
      thf = 0; //FINAL ANGLE
    }

    struct Curve c = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax);

    dubins_curves.push_back(c);

    x0 = xf;
    y0 = yf;
    th0 = thf;

  }

  return dubins_curves;
}
