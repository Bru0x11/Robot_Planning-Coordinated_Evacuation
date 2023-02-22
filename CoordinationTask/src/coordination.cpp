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

    std::cout<<"SIZE: "<<allIntersections.size();
    std::vector<DeltaTime> allDeltas {};

    double robotSize = 0.5;
    double rho = 0.1;
    double costantVelocity = 0.3;
    double epsilon = (robotSize + rho)/costantVelocity; //size of the robot divided by costant velocity + a delta for safety

    DeltaTime d; 
    double diff;
    diff = (allIntersections[0].timeRobot1 - allIntersections[0].timeRobot2);
    d.delta = myAbs((allIntersections[0].timeRobot1 - allIntersections[0].timeRobot2));
    std::cout<<"delta betweeen robot 1 and 2: "<<d.delta<<std::endl;
    d.fasterRobot = (((allIntersections[0].timeRobot1 - allIntersections[0].timeRobot2) > 0) ? 2 : 1); 
    std::cout<<"fasterRobot betweeen robot 1 and 2: "<<d.fasterRobot<<std::endl;
    allDeltas.push_back(d); //delta betweeen robot 1 and 2

    d.delta = myAbs(1.0*(allIntersections[1].timeRobot1 - allIntersections[1].timeRobot2));
    d.fasterRobot = (((allIntersections[1].timeRobot1 - allIntersections[1].timeRobot2) > 0) ? 3 : 1);
    
    allDeltas.push_back(d); //delta betweeen robot 1 and 3
    
    d.delta = myAbs(1.0*(allIntersections[2].timeRobot1 - allIntersections[2].timeRobot2));
    d.fasterRobot = (((allIntersections[2].timeRobot1 - allIntersections[2].timeRobot2) > 0) ? 3 : 2); 

    allDeltas.push_back(d); //delta betweeen robot 2 and 3

    for(int i=0; i<allDeltas.size(); i++){
        std::cout<<"allDeltas["<<i<<"]: "<<allDeltas[i].delta<<std::endl;
    }

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
    // else if((allDeltas[0].delta <=epsilon && allDeltas[1].delta >= epsilon && allDeltas[2].delta >=epsilon)){
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 0};
    //     robotOrder.push_back(robotInit);
        
    //     if (allDeltas[2].delta >= epsilon){
    //         robotInit = {2, 1};
    //         robotOrder.push_back(robotInit);
    //     }else if(allDeltas[2].delta <= 1){
    //         robotInit = {2, 2};
    //         robotOrder.push_back(robotInit);
    //     }

    //}
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



    // if (allDeltas[0].delta > 0 && allDeltas[1].delta > 0 && allDeltas[2].delta > 0){ //all at the same time
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {2, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 0};
    //     robotOrder.push_back(robotInit);
    // }

    // else if (allDeltas[0].delta == 0 && allDeltas[1].delta == 0 && a llDeltas[2].delta == 0){
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {2, 1};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 1};
    //     robotOrder.push_back(robotInit);
    // }else if ((allDeltas[0].delta == 0 && allDeltas[1].delta == 0 && allDeltas[2].delta != 0)){
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {2, 1};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 0};
    //     robotOrder.push_back(robotInit);       
    // }else if((allDeltas[0].delta == 0 && allDeltas[1].delta != 0 && allDeltas[2].delta == 0)){
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {2, 1};
    //     robotOrder.push_back(robotInit);

    // }else if((allDeltas[0].delta != 0 && allDeltas[1].delta == 0 && allDeltas[2].delta == 0)){
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {2, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 1};
    //     robotOrder.push_back(robotInit);
    // }else if((allDeltas[0].delta == 0 && allDeltas[1].delta != 0 && allDeltas[2].delta != 0)){
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 0};
    //     robotOrder.push_back(robotInit);
        
    //     if (allDeltas[2].delta != 1){
    //         robotInit = {2, 1};
    //         robotOrder.push_back(robotInit);
    //     }else if(allDeltas[2].delta == 1){
    //         robotInit = {2, 2};
    //         robotOrder.push_back(robotInit);
    //     }

    // }else if((allDeltas[0].delta != 0 && allDeltas[1].delta == 0 && allDeltas[3].delta != 0)){
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {2, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 1};
    //     robotOrder.push_back(robotInit);
    // }
    // //else if(allDeltas[0].delta != 0 && allDeltas[1].delta != 0 && allDeltas[3].delta == 0){
    // //     robotInit = {1, 0};
    // //     robotOrder.push_back(robotInit);
    // //     robotInit = {2, 0};
    // //     robotOrder.push_back(robotInit);
    // //     robotInit = {3, 1};
    // //     robotOrder.push_back(robotInit);
    // // }

    // else{
    //     robotInit = {1, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {2, 0};
    //     robotOrder.push_back(robotInit);
    //     robotInit = {3, 1};
    //     robotOrder.push_back(robotInit);
    // }

    std::vector<int> timeOrder {}; //order of robots wrt time 





    return robotOrder;
}