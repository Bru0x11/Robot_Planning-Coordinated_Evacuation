#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <sys/types.h>
#include <sys/wait.h>

int sendMessage(std::vector<int> robotOrder){  //std::vector<RobotInitialization> robotOrder

    pid_t pid1, pid2, pid3, wpid;
    int status = 0;

    pid1 = fork();

    if (pid1 == 0){
        sleep(robotOrder[0]); //.delay
        std::cout << "\nI'm activating robot 1...\n";
        exit(0);
    }else{
        pid2 = fork();
        
        if (pid2 == 0){
            sleep(robotOrder[1]); //.delay
            std::cout << "\nI'm activating robot 2...\n";
            exit(0);
        }else{
            pid3 = fork();
            if (pid3 == 0){
                sleep(robotOrder[2]); //.delay
                std::cout << "\nI'm activating robot 3...\n";
                exit(0);
            }
        }
    }

while ((wpid = wait(&status)) > 0);
std::cout << "\nAll processes have ended... quitting program...\n";
return 0;
}


int main(){
    std::vector<int> robotOrder {1, 0, 0};
    sendMessage(robotOrder);
    return 0;
}
