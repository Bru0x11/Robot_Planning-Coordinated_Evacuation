#include "../include/coordination.h"

std::vector<RobotInitialization> coordination(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, VisiLibity::Polyline& robotPath3){

    std::vector<RobotInitialization> robotOrder {}; //final result

    std::vector<Intersection> allIntersections {};
    
    std::cout<<"Lenght of path robot 1: "<<robotPath1.length()<<std::endl;
    std::cout<<"Lenght of path robot 2: "<<robotPath2.length()<<std::endl;
    std::cout<<"Lenght of path robot 3: "<<robotPath3.length()<<std::endl;
    std::cout<<"Time of path robot 1: "<<robotPath1.length()/0.3<<std::endl;
    std::cout<<"Time of path robot 2: "<<robotPath2.length()/0.3<<std::endl;
    std::cout<<"Time of path robot 3: "<<robotPath3.length()/0.3<<std::endl;
    std::cout<<"check intersection "<<std::endl;
    allIntersections.push_back(getPathIntersection(robotPath1, robotPath2)); //Intersection between robot 1 and 2
    allIntersections.push_back(getPathIntersection(robotPath1, robotPath3)); //Intersection between robot 1 and 3
    allIntersections.push_back(getPathIntersection(robotPath2, robotPath3)); //Intersection between robot 2 and 3

    std::vector<DeltaTime> allDeltas {};

    double robotSize = 0.5;
    double rho = 0.3;
    double costantVelocity = 0.3;
    double epsilon = (robotSize + rho)/costantVelocity; //size of the robot divided by costant velocity + a costant for safety

    DeltaTime d; 
    double diff;

    //delta betweeen robot 1 and 2
    diff = (allIntersections[0].timeRobot1 - allIntersections[0].timeRobot2);
    d.delta = myAbs((diff));
    std::cout<<"delta betweeen robot 1 and 2: "<<d.delta<<std::endl;
    d.fasterRobot = (((allIntersections[0].timeRobot1 - allIntersections[0].timeRobot2) > 0) ? 2 : 1); 
    allDeltas.push_back(d); 

    //delta betweeen robot 1 and 3
    d.delta = myAbs(1.0*(allIntersections[1].timeRobot1 - allIntersections[1].timeRobot2));
    d.fasterRobot = (((allIntersections[1].timeRobot1 - allIntersections[1].timeRobot2) > 0) ? 3 : 1);
    std::cout<<"delta betweeen robot 1 and 3: "<<d.delta<<std::endl;
    allDeltas.push_back(d); 
    
    //delta betweeen robot 2 and 3
    d.delta = myAbs(1.0*(allIntersections[2].timeRobot1 - allIntersections[2].timeRobot2));
    d.fasterRobot = (((allIntersections[2].timeRobot1 - allIntersections[2].timeRobot2) > 0) ? 3 : 2); 
    std::cout<<"delta betweeen robot 2 and 3: "<<d.delta<<std::endl;
    allDeltas.push_back(d);

    RobotInitialization robotInit;

    if (allDeltas[0].delta <= epsilon && allDeltas[1].delta <=epsilon && allDeltas[2].delta <=epsilon){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, epsilon};
        robotOrder.push_back(robotInit);
        robotInit = {3, epsilon};
        robotOrder.push_back(robotInit);
    }else if ((allDeltas[0].delta <= epsilon && allDeltas[1].delta <= epsilon && allDeltas[2].delta > epsilon)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, epsilon};
        robotOrder.push_back(robotInit);
        robotInit = {3, 0};
        robotOrder.push_back(robotInit);       
    }else if((allDeltas[0].delta <= epsilon && allDeltas[1].delta > epsilon && allDeltas[2].delta <= epsilon)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, epsilon};
        robotOrder.push_back(robotInit);

    }else if((allDeltas[0].delta > epsilon && allDeltas[1].delta <=epsilon && allDeltas[2].delta <= epsilon)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, epsilon};
        robotOrder.push_back(robotInit);
    }
    else if((allDeltas[0].delta >= epsilon && allDeltas[1].delta <= epsilon && allDeltas[2].delta >=epsilon)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, epsilon};
        robotOrder.push_back(robotInit);
    }
    else if(allDeltas[0].delta >= epsilon && allDeltas[1].delta >= epsilon && allDeltas[2].delta <= epsilon){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, epsilon};
        robotOrder.push_back(robotInit);
    }

    else{
        //allDeltas[0].delta > epsilon && allDeltas[1].delta > epsilon && allDeltas[2].delta > epsilon) //all at the same time
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, 0};
        robotOrder.push_back(robotInit);
    }

    std::vector<int> timeOrder {}; //order of robots wrt time 
    return robotOrder;
}