#ifndef ACTIVATEROBOTS_H
#define ACTIVATEROBOTS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include "coordination.h"

void sendMessage(std::vector<RobotInitialization> robotOrder);

#endif