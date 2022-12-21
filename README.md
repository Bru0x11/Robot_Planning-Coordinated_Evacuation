# Robot_Planning
Project Robot Planning and its application

How the program will be executed:
Get list of points through the API -> pass them to OffsetTools in order to create the offsetted environment, then translate them in the correct format (list of lists of VisiLibityPoints) -> pass this list to Interpolation.cpp where we convert this list of all the obstacles into the correct format (we create the class Environment) -> pass this element into the code that creates the fast_map -> add initial and goal point and complete the map -> find the fastest path + interpolation -> return list of points to the robot

# TODO:
  - create executable to test a example map on docker 

  - EDO -> create executable to test on rviz2 -> print on rviz2
  - BRU -> create an example to obtain points
  - ASH -> adaptive offset  

# TO ASK
