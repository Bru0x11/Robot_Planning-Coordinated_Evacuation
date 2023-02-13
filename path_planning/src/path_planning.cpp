#include "path_planning.h"

class MinimalPublisher : public rclcpp::Node{

  //PUBLIC METHODS
  public:
    MinimalPublisher(): Node("barba_node"){
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);

      // //Tranform the frame
      // std::string target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
      // std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
      // std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      // tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      // tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      // std::string fromFrameRel = target_frame_.c_str();
      // std::string toFrameRel = "map"; //map
      // geometry_msgs::msg::TransformStamped t;

      // try {
      //   rclcpp::Time now = this->get_clock()->now();
      //   t = tf_buffer->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, 30s);
      // } 
      // catch (const tf2::TransformException & ex) {
      //   RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      //   return;
      // }

      // tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
      // tf2::Matrix3x3 m(q);
      // double roll, pitch, yaw;
      // m.getRPY(roll, pitch, yaw);

      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "map";

      //TOPIC SUBSCRIPTION TO READ BORDERS
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
      bordersSubscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "map_borders", qos, std::bind(&MinimalPublisher::border_topic_callback, this, _1));

      //TOPIC SUBSCRIPTION TO READ GATE
      gateSubscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "gate_position", qos, std::bind(&MinimalPublisher::gate_topic_callback, this, _1));
      
      //TOPIC SUBSCRIPTION TO READ OBSTACLES
      subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "obstacles", qos, std::bind(&MinimalPublisher::topic_callback, this, _1));

      // //GATE
      // geometry_msgs::msg::Pose gate;
      // VisiLibity::Point gate_position = VisiLibity::Point(gate.position.x, gate.position.y);
      // double th_gate = gate.orientation.z;

      // //BORDER 
      // geometry_msgs::msg::Polygon border;
      // vector<VisiLibity::Point> points_border;
      // for(int i=0; i<border.points.size(); i++){
      //   points_border.push_back(VisiLibity::Point(border.points[i].x, border.points[i].y));
      // }

      // VisiLibity::Environment new_env = VisiLibity::Environment(points_border);

      // //OBSTACLES
      // std::vector<obstacles_msgs::msg::ObstacleMsg> obstacles = obs_msg.obstacles; 
      // cout<<"ci sono "<<obstacles.size()<<" ostacoli "<<endl;

      // geometry_msgs::msg::Polygon polygon_obs;
      // geometry_msgs::msg::Point32 vertex;

      // for(int i = 0; i<obstacles.size(); i++){
      //   polygon_obs = obstacles[i].polygon;
      //   vector<VisiLibity::Point> points_obs;

      //   for(int j = 0; j<polygon_obs.points.size(); j++){
      //     vertex = polygon_obs.points[j];
      //     points_obs.push_back(VisiLibity::Point(vertex.x, vertex.y));
      //   }
      //   VisiLibity::Polygon new_obs = Polygon(points_obs);
      //   new_env.add_hole(new_obs);
      // }


      //Hyperparameters
      double minimumCurvatureRadius = 0.5;
      double robotSize = 0.3; //We divide its size by 2

      //Defining the environment
      Environment environment = getEnvironment();
      Environment offsettedEnvironment = getOffsettedEnvironment(environment, minimumCurvatureRadius, robotSize);

      //Creating the roadmap
      Visibility_Graph roadmapGraph = Visibility_Graph(offsettedEnvironment, 0.1);

      //DEFINE ROBOT MIN_CURVATURE_RADIUS

      //Defining the start and end point
      // double x0 = t.transform.translation.x;
      // double y0 = t.transform.translation.y;
      double startingPointX = 0;
      double startingPointY = 0;

      VisiLibity::Point startingPoint = VisiLibity::Point(startingPointX, startingPointY);
      VisiLibity::Point endingPoint = VisiLibity::Point(0, 5);

      //Defining start and end angles
      // double th0 = t.transform.rotation.z;
      double thetaStartingPoint = 0;
      double thetaEndingPoint = 0;

      //Finding the shortest path in the map
      Polyline shortestPath = offsettedEnvironment.shortest_path(startingPoint, endingPoint, roadmapGraph, 0.1);

      cout << "Offsetted environment: " << offsettedEnvironment << '\n';
      cout << "Shortest path in the map: " << shortestPath << '\n';      

      Polyline finalPath = interpolation(shortestPath, thetaStartingPoint, thetaEndingPoint, minimumCurvatureRadius);

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

      // publisher_->publish(path);

      //ACTION 

      // rclcpp_action::Client<FollowPath>::SharedPtr client_ptr;
      // client_ptr = rclcpp_action::create_client<FollowPath>(this,"shelfino2/follow_path");
      // if (!client_ptr->wait_for_action_server()) {
      //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      //   rclcpp::shutdown();
      // }
      // auto goal_msg = FollowPath::Goal();
      // goal_msg.path = path;
      // goal_msg.controller_id = "FollowPath";
      // RCLCPP_INFO(this->get_logger(), "Sending goal");

      // auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
      // send_goal_options.result_callback = std::bind(&MinimalPublisher::resultCallback, this, _1);
      // client_ptr->async_send_goal(goal_msg);

    }

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
    RCLCPP_INFO(this->get_logger(), "I heard obstacles ");
  }

  void border_topic_callback(geometry_msgs::msg::Polygon msg){
  RCLCPP_INFO(this->get_logger(), "I heard borders ");
  }

  void gate_topic_callback(geometry_msgs::msg::Pose msg){
  RCLCPP_INFO(this->get_logger(), "I heard gate ");
  }
  
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr bordersSubscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr gateSubscription_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;  
  obstacles_msgs::msg::ObstacleArrayMsg obs_msg;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}