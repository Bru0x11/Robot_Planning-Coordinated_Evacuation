#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "nav2_msgs/action/follow_path.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

#include <iostream>
#include <cstdlib>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

#include "include/interpolation.h"
#include "include/dubins.h"
#include "src/visilibity.hpp"

using namespace std;
using namespace VisiLibity;


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("barba_node")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);

      //Tranform the frame
      std::string target_frame_ = this->declare_parameter<std::string>("target_frame", "shelfino1/base_link");
      std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      std::string fromFrameRel = target_frame_.c_str();
      std::string toFrameRel = "gazebo/base_link"; //map
      geometry_msgs::msg::TransformStamped t;

      try {
        t = tf_buffer->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, 30s);
      } 
      catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
      }

      tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "gazebo/base_link";


      Environment env = get_environment1();

      //ROAD MAP
      Visibility_Graph graph = Visibility_Graph(env, 0);

      //DEFINE ROBOT MIN_CURVATURE_RADIUS
      double minR = 1;
      //DEFINE START AND END POINTS
      double x0 = t.transform.translation.x;
      double y0 = t.transform.translation.y;

      Point start_test = Point(x0, y0);
      Point end = Point(13, 13);
      //DEFINE START AND END ANGLES 
      double th0 = t.transform.rotation.z;
      double thf = M_PI/2;

      //FIND SHORTES PATH
      Polyline shortest_path = env.shortest_path(start_test, end, graph, 0.1);

      cout << "Enviroment is valid: " << env.is_valid() << endl;
      cout << "Shortest_path: " << endl;
      cout << shortest_path << endl;

      cout << "env: "<<endl;
      cout<<env<<endl;

      //cout << "graph: "<<endl<<graph<<endl;
      //sleep(5000); 

      cout<<"faccio interpolation"<<endl;

      Polyline points_final_path = interpolation(shortest_path, th0, thf, minR);

      std::vector<geometry_msgs::msg::PoseStamped> poses_temp;

      geometry_msgs::msg::Pose pose_temp;
      geometry_msgs::msg::Point position_temp;
      geometry_msgs::msg::Quaternion quaternion_temp;
      geometry_msgs::msg::PoseStamped pose_stamped_temp;

      for(int i=0; i<points_final_path.size(); i++){
          double x = points_final_path[i].x();
          double y = points_final_path[i].y();

          position_temp.x = x;
          position_temp.y = y;
          position_temp.z = 0;

          quaternion_temp.x = 0;
          quaternion_temp.y = 0;
          quaternion_temp.z = 0;
          quaternion_temp.w = 0;

          pose_temp.position = position_temp;
          pose_temp.orientation = quaternion_temp;

          pose_stamped_temp.pose = pose_temp;
          pose_stamped_temp.header.stamp = this->get_clock()->now();
          pose_stamped_temp.header.frame_id = "gazebo/base_link";

          poses_temp.push_back(pose_stamped_temp);
      }

      path.poses = poses_temp;
      


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

      //ACTION 
      // using FollowPath = nav2_msgs::action::FollowPath;
      // rclcpp_action::Client<FollowPath>::SharedPtr client_ptr;
      // client_ptr = rclcpp_action::create_client<FollowPath>(this,"shelfino1/follow_path");
      // if (!client_ptr->wait_for_action_server()) {
      //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      //   rclcpp::shutdown();
      // }
      // auto goal_msg = FollowPath::Goal();
      // goal_msg.path = path;
      // goal_msg.controller_id = "FollowPath";
      // RCLCPP_INFO(this->get_logger(), "Sending goal");
      // client_ptr->async_send_goal(goal_msg);

      publisher_->publish(path);
      
    }
  
  private:

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;  

  
};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;

}