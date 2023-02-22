#include "path_planning.h"



class MinimalPublisher : public rclcpp::Node{

  //PUBLIC METHODS
  public:
    MinimalPublisher(): Node("barba_node"){

      sim = false; 

      bordersFlag = false;
      gateFlag = false; 
      obstaclesFlag = false;

      topicFlag = false;

      t1Flag = false;
      t2Flag = false; 
      t3Flag = false;

      publisherR1_ = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan", 10);
      publisherR2_ = this->create_publisher<nav_msgs::msg::Path>("shelfino2/plan", 10);
      publisherR3_ = this->create_publisher<nav_msgs::msg::Path>("shelfino3/plan", 10);

      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);


      //Tranform the frame
      if(sim){
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        std::string toFrameRel = "map";

        try {
          rclcpp::Time now = this->get_clock()->now();
          t1 = tf_buffer->lookupTransform(toFrameRel, "shelfino1/base_link", tf2::TimePointZero, 30s);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), "shelfino1/base_link", ex.what());
          return;
        }

        try {
          rclcpp::Time now = this->get_clock()->now();
          t2 = tf_buffer->lookupTransform(toFrameRel, "shelfino2/base_link", tf2::TimePointZero, 30s);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), "shelfino2/base_link", ex.what());
          return;
        }

        try {
          rclcpp::Time now = this->get_clock()->now();
          t3 = tf_buffer->lookupTransform(toFrameRel, "shelfino3/base_link", tf2::TimePointZero, 30s);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), "shelfino3/base_link", ex.what());
          return;
        }
      }


      // auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

      //TOPIC SUBSCRIPTION TO READ BORDERS
      bordersSubscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "map_borders", qos, std::bind(&MinimalPublisher::border_topic_callback, this, _1));

      //TOPIC SUBSCRIPTION TO READ GATE
      gateSubscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "gate_position", qos, std::bind(&MinimalPublisher::gate_topic_callback, this, _1));
      
      // //TOPIC SUBSCRIPTION TO READ OBSTACLES
      subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", qos, std::bind(&MinimalPublisher::topic_callback, this, _1));

      /*
      //ACTIVATE THE MOTORS
      rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
      client_ = this->create_client<std_srvs::srv::SetBool>("shelfino2/power");
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      request->data = true;

      client_->async_send_request(request);


      //DEACTIVATE THE MOTORS
      client_ = this->create_client<std_srvs::srv::SetBool>("shelfino2/power");
      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      request->data = false;
      client_->async_send_request(request);
      */
      
    }

  void pathPlan(){

      //BORDER 
      vector<VisiLibity::Point> points_border;
      for(int i=0; i<bordersMsg.points.size(); i++){
        points_border.push_back(VisiLibity::Point(bordersMsg.points[i].x, bordersMsg.points[i].y));
      }

      VisiLibity::Environment new_env = VisiLibity::Environment(points_border);


      //OBSTACLES
      //obstacles_msgs::msg::ObstacleMsg arrayObstacles = obstacles_msgs::msg::ObstacleArrayMsg
      geometry_msgs::msg::Polygon polygon_obs;
      geometry_msgs::msg::Point32 vertex;

      for(int i = 0; i<obstaclesMsg.obstacles.size(); i++){
        polygon_obs = obstaclesMsg.obstacles[i].polygon;
        vector<VisiLibity::Point> points_obs;

        for(int j = 0; j<polygon_obs.points.size(); j++){
          vertex = polygon_obs.points[j];
          points_obs.push_back(VisiLibity::Point(vertex.x, vertex.y));
        }
        VisiLibity::Polygon new_obs = Polygon(points_obs);
        new_env.add_hole(new_obs);
      }

      //cout<<"bordersMsg.points.size(): "<<bordersMsg.points.size()<<endl;

      //GATE
      geometry_msgs::msg::Pose gate = gateMsg.poses[0];
      VisiLibity::Point gatePosition = VisiLibity::Point(gate.position.x, gate.position.y);
      double thGate = gate.orientation.z;

      //Hyperparameters
      double minimumCurvatureRadius = 0.5;
      double robotSize = 0.3; //We divide its size by 2

      //Defining the environment
      //Environment environment = getEnvironment();
      Environment environment = new_env;
      cout<<"Environment isValid: "<<new_env.is_valid(0.1)<<endl;
      Environment offsettedEnvironment = getOffsettedEnvironment(environment, minimumCurvatureRadius, robotSize);

      cout<<"offsettedEnvironment isValid: "<<offsettedEnvironment.is_valid(0.1)<<endl;

      //Creating the roadmap
      Visibility_Graph roadmapGraph = Visibility_Graph(offsettedEnvironment, 0.1);

      //DEFINE ROBOT MIN_CURVATURE_RADIUS

      double startingPointXR1;
      double startingPointYR1;

      double startingPointXR2;
      double startingPointYR2;

      double startingPointXR3;
      double startingPointYR3;

      //Defining start angles
      double thetaStartingPointR1;
      double thetaStartingPointR2;
      double thetaStartingPointR3;

      if(sim){
        cout<<"POSIZIONE ROBOT 1 : X: "<<t1.transform.translation.x<<" Y: "<<t1.transform.translation.y<<endl;
        cout<<"POSIZIONE ROBOT 2 : X: "<<t2.transform.translation.x<<" Y: "<<t2.transform.translation.y<<endl;
        cout<<"POSIZIONE ROBOT 3 : X: "<<t3.transform.translation.x<<" Y: "<<t3.transform.translation.y<<endl;

        startingPointXR1 = t1.transform.translation.x;
        startingPointYR1 = t1.transform.translation.y;

        startingPointXR2 = t2.transform.translation.x;
        startingPointYR2 = t2.transform.translation.y;

        startingPointXR3 = t3.transform.translation.x;
        startingPointYR3 = t3.transform.translation.y;

        //Defining start angles
        thetaStartingPointR1 = t1.transform.rotation.z;
        thetaStartingPointR2 = t2.transform.rotation.z;
        thetaStartingPointR3 = t3.transform.rotation.z;
      }
      else{

        startingPointXR1 = -4;
        startingPointYR1 = -3;

        startingPointXR2 = -4;
        startingPointYR2 = 0;

        startingPointXR3 = -4;
        startingPointYR3 = 3;

        thetaStartingPointR1 = 0;
        thetaStartingPointR2 = 0;
        thetaStartingPointR3 = 0;
      }

      VisiLibity::Point startingPointR1 = VisiLibity::Point(startingPointXR1, startingPointYR1);
      VisiLibity::Point startingPointR2 = VisiLibity::Point(startingPointXR2, startingPointYR2);
      VisiLibity::Point startingPointR3 = VisiLibity::Point(startingPointXR3, startingPointYR3);

      VisiLibity::Point endingPoint = VisiLibity::Point(4, 0);
      // VisiLibity::Point endingPoint = gatePosition;

      double thetaEndingPoint = 0;
      //double thetaEndingPoint = thGate;

      //Finding the shortest path in the map
      cout << "Compute shortest path R1 " << '\n';
      Polyline shortestPathR1 = offsettedEnvironment.shortest_path(startingPointR1, endingPoint, roadmapGraph, 0.1);
      cout << "Compute shortest path R2 " << '\n';
      Polyline shortestPathR2 = offsettedEnvironment.shortest_path(startingPointR2, endingPoint, roadmapGraph, 0.1);
      cout << "Compute shortest path R3 " << '\n';
      Polyline shortestPathR3 = offsettedEnvironment.shortest_path(startingPointR3, endingPoint, roadmapGraph, 0.1);

      cout << "Offsetted environment: \n" << offsettedEnvironment << '\n';
      cout << "Shortest path R1 in the map: " << shortestPathR1 << '\n';  
      cout << "Shortest path R2 in the map: " << shortestPathR2 << '\n';
      cout << "Shortest path R3 in the map: " << shortestPathR3 << '\n';    

      Polyline finalPathR1 = interpolation(shortestPathR1, thetaStartingPointR1, thetaEndingPoint, minimumCurvatureRadius);
      Polyline finalPathR2 = interpolation(shortestPathR2, thetaStartingPointR2, thetaEndingPoint, minimumCurvatureRadius);
      Polyline finalPathR3 = interpolation(shortestPathR3, thetaStartingPointR3, thetaEndingPoint, minimumCurvatureRadius);

      std::vector<RobotInitialization> robotOrder = coordination(finalPathR1, finalPathR2, finalPathR3);

      cout<<"Robot 1 activates in "<< robotOrder[0].delay <<" seconds"<<endl;
      cout<<"Robot 2 activates in "<< robotOrder[1].delay +  robotOrder[0].delay<<" seconds"<<endl;
      cout<<"Robot 3 activates in "<< robotOrder[2].delay + robotOrder[1].delay + robotOrder[0].delay <<" seconds"<<endl;

      nav_msgs::msg::Path pathMsgR1 = getPathMsg(finalPathR1);
      nav_msgs::msg::Path pathMsgR2 = getPathMsg(finalPathR2);
      nav_msgs::msg::Path pathMsgR3 = getPathMsg(finalPathR3);

      //sendMessage(robotOrder, pathMsgR1, pathMsgR2, pathMsgR3);

      sleep(5);

      publisherR1_->publish(pathMsgR1);
      sleep(1);
      publisherR2_->publish(pathMsgR2);
      sleep(1);
      publisherR3_->publish(pathMsgR3);

      sleep(5);




      // sleep(robotOrder[0].delay); //.delay
      // std::cout << "\nI'm activating robot 1...\n";

      // client_ptrR1 = rclcpp_action::create_client<FollowPath>(this,"shelfino1/follow_path");
      // if (!client_ptrR1->wait_for_action_server()) {
      //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      //   rclcpp::shutdown();
      // }
      // auto goalMsgR1 = FollowPath::Goal();
      // goalMsgR1.path = pathMsgR1;
      // goalMsgR1.controller_id = "FollowPath";
      // RCLCPP_INFO(this->get_logger(), "Sending goal");
      // //auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
      // //send_goal_options.result_callback = std::bind(&MinimalPublisher::resultCallback, this, _1);
      // client_ptrR1->async_send_goal(goalMsgR1); 


      
      // sleep(robotOrder[1].delay);
      // std::cout << "\nI'm activating robot 2...\n";

      // client_ptrR2 = rclcpp_action::create_client<FollowPath>(this,"shelfino2/follow_path");
      // if (!client_ptrR2->wait_for_action_server()) {
      //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      //   rclcpp::shutdown();
      // }
      // auto goalMsgR2 = FollowPath::Goal();
      // goalMsgR2.path = pathMsgR2;
      // goalMsgR2.controller_id = "FollowPath";
      // RCLCPP_INFO(this->get_logger(), "Sending goal");

      // //auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
      // //send_goal_options.result_callback = std::bind(&MinimalPublisher::resultCallback, this, _1);
      // client_ptrR2->async_send_goal(goalMsgR2); 

      
      // sleep(robotOrder[2].delay);
      // std::cout << "\nI'm activating robot 3...\n";
  
      // client_ptrR3 = rclcpp_action::create_client<FollowPath>(this,"shelfino3/follow_path");
      // if (!client_ptrR3->wait_for_action_server()) {
      //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      //   rclcpp::shutdown();
      // }
      // auto goalMsgR3 = FollowPath::Goal();
      // goalMsgR3.path = pathMsgR3;
      // goalMsgR3.controller_id = "FollowPath";
      // RCLCPP_INFO(this->get_logger(), "Sending goal");

      // //auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
      // //send_goal_options.result_callback = std::bind(&MinimalPublisher::resultCallback, this, _1);
      // client_ptrR3->async_send_goal(goalMsgR3); 


  }

  nav_msgs::msg::Path getPathMsg(Polyline finalPath){

      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "map";
      std::vector<geometry_msgs::msg::PoseStamped> posesTemp;
      geometry_msgs::msg::Pose poseTemp;
      geometry_msgs::msg::Point positionTemp;
      geometry_msgs::msg::Quaternion quaternionTemp;
      geometry_msgs::msg::PoseStamped poseStampedTemp;

      for(int i=0; i < finalPath.size(); i++){
          double x = finalPath[i].x();
          double y = finalPath[i].y();

          positionTemp.x = x;
          positionTemp.y = y;
          positionTemp.z = 0;

          quaternionTemp.x = 0;
          quaternionTemp.y = 0;
          quaternionTemp.z = 0;
          quaternionTemp.w = 0;

          poseTemp.position = positionTemp;
          poseTemp.orientation = quaternionTemp;

          poseStampedTemp.pose = poseTemp;
          poseStampedTemp.header.stamp = this->get_clock()->now();
          poseStampedTemp.header.frame_id = "base_link";

          posesTemp.push_back(poseStampedTemp);
      }

      path.poses = posesTemp;

      return path;
  }

//   int sendMessage(std::vector<RobotInitialization> robotOrder, nav_msgs::msg::Path pathMsgR1, nav_msgs::msg::Path pathMsgR2, nav_msgs::msg::Path pathMsgR3){  //std::vector<RobotInitialization> robotOrder
    
//     publisherR1_ = this->create_publisher<nav_msgs::msg::Path>("plan1", 10);
//     publisherR2_ = this->create_publisher<nav_msgs::msg::Path>("plan2", 10);
//     publisherR3_ = this->create_publisher<nav_msgs::msg::Path>("plan3", 10);

//     pid_t pid1, pid2, pid3, wpid;
//     int status = 0;

//     pid1 = fork();

//     if (pid1 == 0){
//         sleep(robotOrder[0].delay); //.delay
//         std::cout << "\nI'm activating robot 1...\n";
//         publisherR1_->publish(pathMsgR1);
//         sleep(3);
//         exit(0);
//     }else{
//         pid2 = fork();
        
//         if (pid2 == 0){
//             sleep(robotOrder[1].delay); //.delay
//             std::cout << "\nI'm activating robot 2...\n";
//             publisherR2_->publish(pathMsgR2);
//             exit(0);
//         }else{
//             pid3 = fork();
//             if (pid3 == 0){
//                 sleep(robotOrder[2].delay); //.delay
//                 std::cout << "\nI'm activating robot 3...\n";
//                 publisherR3_->publish(pathMsgR3);
//                 exit(0);
//             }
//          }
//     }

//     while ((wpid = wait(&status)) > 0);
//     std::cout << "\nAll processes have ended... turning off the engines... quitting program...\n";
//     return 0;
// }

  //Receiving the result from the robot
  void resultCallback(const GoalHandle::WrappedResult & result){
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!!!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
  }

  //PRIVATE METHODS
  private:

  void topic_callback(obstacles_msgs::msg::ObstacleArrayMsg msg){
    RCLCPP_INFO(this->get_logger(), "I heard '%lu' obstacles", msg.obstacles.size());
    obstaclesFlag = true;
    // if(bordersFlag && gateFlag && !topicFlag){
      //topicFlag = true
    //   pathPlan();
    // }
    obstaclesMsg = msg;
    obstaclesFlag = true;
    
    if(bordersFlag && gateFlag){
      if(!topicFlag){
        topicFlag = true;
        pathPlan();
      }
    }
  }


  void gate_topic_callback(geometry_msgs::msg::PoseArray msg){
    RCLCPP_INFO(this->get_logger(), "I heard gate ");
    gateFlag = true;
    gateMsg = msg;

    if(obstaclesFlag && bordersFlag){
      if(!topicFlag){
        topicFlag = true;
        pathPlan();
      }
    }
  }

  void border_topic_callback(geometry_msgs::msg::Polygon msg){
    RCLCPP_INFO(this->get_logger(), "I heard borders ");
    bordersFlag = true;
    bordersMsg = msg; 

    if(obstaclesFlag && gateFlag){
      if(!topicFlag){
        topicFlag = true;
        pathPlan();
      }
    }
  }

  


  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gateSubscription_;
  geometry_msgs::msg::PoseArray gateMsg; 

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherR1_;  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherR2_;  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherR3_;  

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr bordersSubscription_;
  geometry_msgs::msg::Polygon bordersMsg;

  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
  obstacles_msgs::msg::ObstacleArrayMsg obstaclesMsg;

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr t1sub;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr t2sub;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr t3sub;

  rclcpp_action::Client<FollowPath>::SharedPtr client_ptrR1;
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptrR2;
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptrR3;

  // rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tfsub;

  geometry_msgs::msg::TransformStamped t1 ;
  geometry_msgs::msg::TransformStamped t2 ;
  geometry_msgs::msg::TransformStamped t3 ;


  // vector<geometry_msgs::msg::TransformStamped> tf;

  bool bordersFlag;
  bool gateFlag;
  bool obstaclesFlag;

  bool topicFlag;

  bool t1Flag;
  bool t2Flag;
  bool t3Flag;

  bool sim = true;

  bool tfFlag;

};





// class MinimalSubscriber : public rclcpp::Node
// {
//   public:
//     MinimalSubscriber()
//     : Node("minimal_subscriber")
//     {
//       //TOPIC SUBSCRIPTION TO READ BORDERS
//       auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
//       bordersSubscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
//       "map_borders", qos, std::bind(&MinimalSubscriber::border_topic_callback, this, _1));
//     }

//   private:
//     void border_topic_callback(geometry_msgs::msg::Polygon msg){
//       RCLCPP_INFO(this->get_logger(), "I heard borders ");
//       bordersMsg = msg;
//       bordersFlag = true;
//   }
// };


// void start_ros_node(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
// }

// void start_border_read(int argc, char * argv[]){
  
//   while(!bordersFlag){

//   }
//   cout<<"READ!!!!"<<endl;

// }

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);
  // std::thread border_listener(start_border_read, argc, argv);
  // std::thread path_planning(start_ros_node, argc, argv);

  // border_listener.join();
  // path_planning.join();

  // rclcpp::spin(std::make_shared<MinimalSubscriber>());
  // rclcpp::shutdown();

  // sleep(10);
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());

  // std::thread first (start_ros_node, argc, argv);     // spawn new thread that calls foo()
  // std::thread second (start_border_read, argc, argv);

  // first.join();                // pauses until first finishes
  // second.join();   

  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  
  return 0;
}