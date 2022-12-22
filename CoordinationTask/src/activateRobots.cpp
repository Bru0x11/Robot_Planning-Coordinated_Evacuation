#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void activateRobots(std::vector<RobotInitialization> robotOrder){

    int pid1, pid2, pid3;

    pid1 = fork();

    if (pid1 == 0){
        sleep(robotOrder[0].delay);
        std::cout << "\nI'm activating robot 1...\n";
    }else{
        pid2 = fork();
        
        if (pid2 == 0){
            sleep(robotOrder[1].delay);
            std::cout << "\nI'm activating robot 2...\n";
        }else{
            pid3 = fork();
            if (pid3 == 0){
                sleep(robotOrder[2].delay);
                std::cout << "\nI'm activating robot 3...\n";
            }
        }
    }

}


int main(){

    std::vector<RobotInitialization> robotOrder {};
    robotOrder.push_back(RobotInitialization(1, 0));
    robotOrder.push_back(RobotInitialization(2, 0));
    robotOrder.push_back(RobotInitialization(3, 1));

    activateRobots(robotOrder);

    return 0;
}
