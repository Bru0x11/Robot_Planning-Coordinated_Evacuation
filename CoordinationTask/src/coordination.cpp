#include "../include/coordination.h"

std::vector<RobotInitialization> coordination(VisiLibity::Polyline& robotPath1, VisiLibity::Polyline& robotPath2, VisiLibity::Polyline& robotPath3){

    std::vector<RobotInitialization> robotOrder {}; //final result

    std::vector<Intersection> allIntersections {};
    allIntersections.push_back(getPathIntersection(robotPath1, robotPath2)); //Intersection between robot 1 and 2
    allIntersections.push_back(getPathIntersection(robotPath1, robotPath3)); //Intersection between robot 1 and 3
    allIntersections.push_back(getPathIntersection(robotPath2, robotPath3)); //Intersection between robot 2 and 3

    std::vector<DeltaTime> allDeltas {};

    DeltaTime d; 
    d.delta = fabs(allIntersections[0].timeRobot1 - allIntersections[0].timeRobot2);
    d.fasterRobot = (((allIntersections[0].timeRobot1 - allIntersections[0].timeRobot2) > 0) ? 2 : 1); //TODO: cambiare > 0 in qualcosa di più coarse (uguale al granularity che sceglierò)

    allDeltas.push_back(d); //delta betweeen robot 1 and 2

    d.delta = fabs(allIntersections[1].timeRobot1 - allIntersections[1].timeRobot2);
    d.fasterRobot = (((allIntersections[1].timeRobot1 - allIntersections[1].timeRobot2) > 0) ? 3 : 1);//TODO: cambiare > 0 in qualcosa di più coarse);
    
    allDeltas.push_back(d); //delta betweeen robot 1 and 3
    
    d.delta = (fabs(allIntersections[2].timeRobot1 - allIntersections[2].timeRobot2));
    d.fasterRobot = (((allIntersections[2].timeRobot1 - allIntersections[2].timeRobot2) > 0) ? 3 : 2); //TODO: cambiare > 0 in qualcosa di più coarse;

    allDeltas.push_back(d); //delta betweeen robot 2 and 3

    RobotInitialization robotInit;
    if (allDeltas[0].delta > 0 && allDeltas[1].delta > 0 && allDeltas[2].delta > 0){ //all at the same time
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, 0};
        robotOrder.push_back(robotInit);
    }

    else if (allDeltas[0].delta == 0 && allDeltas[1].delta == 0 && allDeltas[2].delta == 0){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 1};
        robotOrder.push_back(robotInit);
        robotInit = {3, 1};
        robotOrder.push_back(robotInit);
    }else if ((allDeltas[0].delta == 0 && allDeltas[1].delta == 0 && allDeltas[2].delta != 0)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 1};
        robotOrder.push_back(robotInit);
        robotInit = {3, 0};
        robotOrder.push_back(robotInit);       
    }else if((allDeltas[0].delta == 0 && allDeltas[1].delta != 0 && allDeltas[2].delta == 0)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 1};
        robotOrder.push_back(robotInit);

    }else if((allDeltas[0].delta != 0 && allDeltas[1].delta == 0 && allDeltas[2].delta == 0)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, 1};
        robotOrder.push_back(robotInit);
    }else if((allDeltas[0].delta == 0 && allDeltas[1].delta != 0 && allDeltas[2].delta != 0)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, 0};
        robotOrder.push_back(robotInit);
        
        if (allDeltas[2].delta != 1){
            robotInit = {2, 1};
            robotOrder.push_back(robotInit);
        }else if(allDeltas[2].delta == 1){
            robotInit = {2, 2};
            robotOrder.push_back(robotInit);
        }

    }else if((allDeltas[0].delta != 0 && allDeltas[1].delta == 0 && allDeltas[3].delta != 0)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, 1};
        robotOrder.push_back(robotInit);
    }else if((allDeltas[0].delta != 0 && allDeltas[1].delta != 0 && allDeltas[3].delta == 0)){
        robotInit = {1, 0};
        robotOrder.push_back(robotInit);
        robotInit = {2, 0};
        robotOrder.push_back(robotInit);
        robotInit = {3, 1};
        robotOrder.push_back(robotInit);
    }

    return robotOrder;
}