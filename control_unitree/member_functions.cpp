// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



#include "unitree_ros2_msgs/msg/high_cmd.hpp"
#include "unitree_ros2_msgs/msg/high_state.hpp"
/*#include "unitree_legged_sdk/unitree_legged_sdk.h"
*/
#include <functional>
#include <memory>
#include <thread>
#include <time.h>
// #include "action_msg/action/fibonacci.hpp"
#include "action_msg/action/control.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
//#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <chrono>
#include <functional>
#include <string>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <nav_msgs/msg/odometry.hpp> 
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
using namespace std::chrono_literals;


void stop_mode();
void motion_cmd();
void get_path_errors();
void path_following_controller();
void motion_to_avoid_collision_at_first_rotation();
bool fault_detected();
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double v_real);
double get_yaw(geometry_msgs::msg::Quaternion q);

class ControlUnitree : public rclcpp::Node
{
public:
  using Control = action_msg::action::Control;
  using GoalHandleControl = rclcpp_action::ServerGoalHandle<Control>;

  explicit ControlUnitree(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("control_unitree_b1", options)
  {
    tf_buffer_ =   std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Call on_timer function every second
    timer_ = this->create_wall_timer(0.1s, std::bind(&ControlUnitree::on_timer, this));
    publisher_ = this->create_publisher<unitree_ros2_msgs::msg::HighCmd>("high_cmd", 1);
    publisher_check_collision = this->create_publisher<geometry_msgs::msg::Point>("check_collision", 1);
    using std::placeholders::_1;
    theta_s_path_sub = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlUnitree::theta_star_path_callback, this, _1));
    mpc_path_sub = this->create_subscription<nav_msgs::msg::Path>(
      "/mpc_planner/local_path", 10, std::bind(&ControlUnitree::path_mpc_callback, this, _1));
    grid_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/occupancy_grid_map/grid_map", 10, std::bind(&ControlUnitree::grid_map_callback, this, _1));
    grid_local_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/occupancy_grid_local_map/grid_map", 10, std::bind(&ControlUnitree::grid_local_map_callback, this, _1));
    flag_orient_sub = this->create_subscription<std_msgs::msg::Bool>(
      "flag_orient", 10, std::bind(&ControlUnitree::flag_orientation_callback, this, _1));
    status_collision_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/collision_rotation", 10, std::bind(&ControlUnitree::collision_rotation_back, this, _1));
    backward_collision_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/collision_backward", 10, std::bind(&ControlUnitree::collision_backward_back, this, _1));
    position_collision_sub = this->create_subscription<std_msgs::msg::Bool>(
      "/collision_position", 10, std::bind(&ControlUnitree::collision_position_back, this, _1));
  //  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
  //    "path", 10, std::bind(&ControlUnitree::odom_callback, this, _1));

    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Control>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "control",
      std::bind(&ControlUnitree::handle_goal, this, _1, _2),
      std::bind(&ControlUnitree::handle_cancel, this, _1),
      std::bind(&ControlUnitree::handle_accepted, this, _1));
  }

private:
  // timers , action , tf
  rclcpp_action::Server<Control>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  //  publishers,subscriberr
  rclcpp::Publisher<unitree_ros2_msgs::msg::HighCmd>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_check_collision;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr theta_s_path_sub, mpc_path_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_local_map_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_orient_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_collision_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr backward_collision_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr position_collision_sub;
 // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  // messages
  unitree_ros2_msgs::msg::HighCmd high_cmd;
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Point point_collision;
  // parameters
  std::string global_frame = "local_map_lidar";
  std::string local_frame = "base";
    
  double  x1_path , y1_path,
          final_orientation,
          odom_x, odom_y, odom_theta,
          v_robot , v_cmd , w_cmd,
          max_v , min_v , acc_v , max_w,
          dist_error , angle_error , 
          dt = 0.1, dist_limit = 0.5,
          x_goal , y_goal,
          time_squat = 0 , time_stand = 0;

  double  x_path[100], y_path[100];

  int32_t t_g_map , t_l_map, t_mpc;

  int     size_path , k_v=0;

  bool    flag_orientation = false , StartMotion = false,
          path_stored , final_theta_corrected,
          robot_reached, simulation_node_is_active = false,
          rotation_collision = true , position_collision = true,
          backward_collision = true,
          StartSquat = false , StartStand = false,
          squat_reached = false , stand_reached = false,
          start_simulate_collision = false;
  
  //////// [action server] /////////
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Control::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->goal.c_str());
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->goal == "StartMotion") {
        robot_reached = false;
        start_simulate_collision = true;
        StartMotion = true;
        path_stored = false;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else if (goal->goal == "Squat") {
        squat_reached = false;
        StartSquat = true;
        time_squat = 0;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else if (goal->goal == "Stand") {
        stand_reached = false;
        StartStand = true;
        time_stand = 0;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleControl> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleControl> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    if(StartMotion)
    {
      rclcpp::Rate loop_rate(1);
      auto result = std::make_shared<Control::Result>();
      while(!robot_reached) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          v_cmd = 0;
          w_cmd = 0;
          high_cmd.velocity[0] = v_cmd;
          high_cmd.yaw_speed = w_cmd;
          publisher_->publish(high_cmd);
          StartMotion = false;
          result->status = "canceled";
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal Canceled");
          return;
        }
        loop_rate.sleep();
      }

      // Check if goal is done
      if (robot_reached) 
      {
        v_cmd = 0;
        w_cmd = 0;
        high_cmd.velocity[0] = v_cmd;
        high_cmd.yaw_speed = w_cmd;
        publisher_->publish(high_cmd);
        StartMotion = false;
        RCLCPP_INFO(this->get_logger(), "Robot Reached to the goal %f %f", x_goal , y_goal);
      //  reached_status_msg.data = "reached";
        result->status = "done";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
      }
    }
    if(StartSquat)
    {
      rclcpp::Rate loop_rate(1);
      auto result = std::make_shared<Control::Result>();
      while(!squat_reached) {

        loop_rate.sleep();
      }

      if(squat_reached)
      {
        result->status = "done";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Squat Succeeded");
        StartSquat = false;
      }
    }
    if(StartStand)
    {
      rclcpp::Rate loop_rate(1);
      auto result = std::make_shared<Control::Result>();
      while(!stand_reached) {

        loop_rate.sleep();
      }
      if(stand_reached)
      {
        result->status = "done";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Stand Succeeded");
        StartStand = false;
      }
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleControl> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ControlUnitree::execute, this, _1), goal_handle}.detach();
  }
  
  //////// [callbacks] /////////////
  void theta_star_path_callback(const nav_msgs::msg::Path& msg)
  {
  //  RCLCPP_INFO(this->get_logger(), "I heard: '%i' points path", msg.poses.size());
    int size_ = msg.poses.size();
    if(size_!=0)
    {
      final_orientation = get_yaw(msg.poses[size_-1].pose.orientation);
      x1_path = msg.poses[0].pose.position.x;
      y1_path = msg.poses[0].pose.position.y;
      x_goal = msg.poses[size_-1].pose.position.x;
      y_goal = msg.poses[size_-1].pose.position.y;
      for(int i=0; i<size_; i++)
      {
        x_path[i] = msg.poses[i].pose.position.x;
        y_path[i] = msg.poses[i].pose.position.y;
      }

      path_stored = true;    
      k_v=0;
      get_path_errors();
      stop_mode();
      final_theta_corrected = false;
    }
    else
    {
      path_stored = false;
      v_cmd = 0;
      w_cmd = 0;
    }


  }
  void path_mpc_callback(const nav_msgs::msg::Path& msg)
  {
    /*int size = msg.poses.size();
    if(size!=0)
    {         
      for(int i=0; i<size; i++)
      {
        x_path[i] = msg.poses[i].pose.position.x;
        y_path[i] = msg.poses[i].pose.position.y;
      }
      size_path = size;
      path_stored = true;
      robot_reached = false;
      k_v=0;
      get_path_errors();
      stop_mode();
      final_theta_corrected = false;
    }
    else
    {
        path_stored = false;
        v_cmd = 0;
        w_cmd = 0;
    }*/
  }
  void flag_orientation_callback(const std_msgs::msg::Bool& msg) 
  {
    flag_orientation = msg.data;
  }


  /*void odom_callback(const nav_msgs::msg::Odometry& msg) 
  {
    v_robot = abs(msg.twist.twist.linear.x);
  }*/
  void grid_map_callback(const nav_msgs::msg::OccupancyGrid& msg)
  {
    t_g_map = msg.header.stamp.sec;
  }
  void grid_local_map_callback(const nav_msgs::msg::OccupancyGrid& msg)
  {
    t_l_map = msg.header.stamp.sec;
  }

  void collision_rotation_back(const std_msgs::msg::Bool msg) 
{  
    if (msg.data)
    {
        v_cmd = 0;
        rotation_collision = true;
        RCLCPP_INFO(this->get_logger(),"Rotation motion can lead to collide!");
    }
    else 
    {
        rotation_collision = false;
        RCLCPP_INFO(this->get_logger(),"Robot is rotating safety!");
    }
    simulation_node_is_active = true;
}

void collision_position_back(const std_msgs::msg::Bool msg) 
{  
    if (msg.data)
    {
        v_cmd = 0;
        position_collision = true;
        RCLCPP_INFO(this->get_logger(),"Forward motion can lead to collide!");
    }
    else 
    {
        position_collision = false;
        RCLCPP_INFO(this->get_logger(),"Forward motion safety!");
    }
    simulation_node_is_active = true;
}

void collision_backward_back(const std_msgs::msg::Bool msg) 
{
    if(msg.data)
    {
        backward_collision = true;
        v_cmd = 0;
        RCLCPP_INFO(this->get_logger(),"Backward motion can lead to collision! Stop Motion");
    }
    else
    {
        backward_collision = false;
        
    }
    simulation_node_is_active = true;
}

  
  //////// [custom methods] /////////
  bool fault_detected()
  {
    int32_t t_now=this->now().seconds();

    if(((t_now - t_g_map)>2 || (t_now - t_l_map)>2 || (t_now - t_mpc)>2) && StartMotion)
    {
     //   ROS_ERROR("Occupancy Grid Map, Local map or MPC is Failed, Stop Motion");
        return true;
    }
    else
    {
        return false;
    }
  }

  void path_following_controller()
{
  max_v = 0.15;
  acc_v = 0.04;
  min_v = 0;
  max_w = 0.3;
  //  DECREASING
  if (k_v == size_path-1 && dist_error<1 && v_cmd>(max_v/2))
  {
    double max_v_local = v_cmd;
    if (max_v_local>(max_v/2))
    {
        max_v_local=max_v_local-0.04;
    }
    v_cmd = sat_linear_velocity(max_v_local,min_v,acc_v ,dist_error, v_cmd);
  }
  else
  {
    // Motion 
    if(abs(angle_error)<0.1)  // case error angle very small
    {
        v_cmd = sat_linear_velocity(max_v,min_v,acc_v,dist_error , v_cmd);
        w_cmd = angle_error;
    }
    else if(abs(angle_error)>=0.1 && abs(angle_error)<0.38)  // case error angle small
    {
        v_cmd = sat_linear_velocity(max_v-(max_v/6),min_v,acc_v,dist_error , v_cmd);
        w_cmd = 0.9*angle_error;
    }
    else if(abs(angle_error)>=0.38 && abs(angle_error)<0.7)  // case error angle medium
    {
        v_cmd = sat_linear_velocity(max_v-(max_v/3),min_v,acc_v,dist_error , v_cmd);
        w_cmd = 0.85*angle_error;
    }
    else if(abs(angle_error)>=0.7 && abs(angle_error)<1)  // case error angle big
    {
        v_cmd = sat_linear_velocity(max_v-(max_v/2),min_v,acc_v,dist_error , v_cmd);
        w_cmd = 0.8*angle_error;
    }
    else if(abs(angle_error)>=1 && abs(angle_error)<1.25)  // case angle error very big
    {
        v_cmd = sat_linear_velocity(max_v-(max_v/1.5),min_v,acc_v,dist_error , v_cmd);
        w_cmd = 0.75*angle_error;
    }
    else if(abs(angle_error)>=1.25)
    {
        v_cmd = 0;
        w_cmd = 1*angle_error;
    }     
  }
  
  // to gurantee that robot will correct orientation before first motion
  if(abs(angle_error)>=0.1 && v_robot<=(max_v/3) && k_v<=5) // v_cmd instead of v_robot for simulation
  {
    v_cmd = 0;
    w_cmd = 1*angle_error;
  }

  // to start smoothly in case emergency stop
  if (abs(v_robot)<0.06 && abs(v_cmd-v_robot)>0.4)
  {
      v_cmd = 0;
  }

  if(w_cmd>max_w)
  {
    w_cmd = max_w;
  }
  else if(w_cmd<-max_w)
  {
    w_cmd = -max_w;
  }

  if(v_cmd>max_v)
  {
    v_cmd = max_v;
  }
  if(v_cmd < 0)
  v_cmd = 0;
  RCLCPP_INFO(this->get_logger(), "velocity commands %f %f", v_cmd, w_cmd);
  high_cmd.velocity[0] = v_cmd;
  high_cmd.yaw_speed = w_cmd;
}

void get_path_errors()
{
  double angle_segment = 0 , err_x , err_y , err_theta , compensate_angle = 0;;
  if(start_simulate_collision)
  {
    point_collision.x = x1_path;
    point_collision.y = y1_path;
    RCLCPP_INFO(this->get_logger(), "publish check collision");
    publisher_check_collision->publish(point_collision);
    start_simulate_collision = false;
    simulation_node_is_active = false;
  }

  //Calculate the erros between the robot and the current segement
  err_x = (x_path[k_v]-odom_x); 
  err_y =  (y_path[k_v]-odom_y);
  dist_error = sqrt(pow(err_x, 2) + pow(err_y, 2));
  angle_segment=atan2(err_y,err_x);
  err_theta =  angle_segment - odom_theta - compensate_angle;
  angle_error = atan2(sin(err_theta ),cos(err_theta ));

  // accordingly to these errors move to the next segment      
  while (dist_error<dist_limit && k_v <size_path-1)
  {
    err_x = (x_path[k_v]-odom_x);
    err_y =  (y_path[k_v]-odom_y);
    dist_error = sqrt(pow(err_x, 2) + pow(err_y, 2));
    angle_segment=atan2(err_y,err_x);
    err_theta =  angle_segment - odom_theta - compensate_angle;
    angle_error = atan2(sin(err_theta ),cos(err_theta ));
    // prevent vibration robot at critical angle
    if(abs(angle_error)>3.04)
    {
      if(angle_segment>odom_theta)
        angle_error = abs(angle_error);
      else
        angle_error = -abs(angle_error);           
    }
    k_v++;
  }  
}

void stop_mode()
{
  double dist_stop = sqrt(pow((x_goal-odom_x), 2) + pow((y_goal-odom_y), 2));
  RCLCPP_INFO(this->get_logger(), "dist_stop %f ", dist_stop);
  if (dist_stop<=0.5)
  {
    path_stored=false;
    if(!flag_orientation)
    {
      robot_reached=true;
    }
  }
}

void correction_final_theta()
{
  double err_theta =  final_orientation - odom_theta;
  angle_error = atan2(sin(err_theta),cos(err_theta));
  if (abs(err_theta)>=0.05 && !final_theta_corrected)
  {
    v_cmd = 0.0;
    w_cmd = 1.2*angle_error;
    if (angle_error > 0)
    {
        w_cmd = 0.25;
    }
    else if(angle_error<0)
    {
        w_cmd = -0.25;
    }
  }
  if (abs(err_theta)<0.06 && !final_theta_corrected)
  {
    robot_reached=true;
    final_theta_corrected = true;
  }
}

double sat_linear_velocity(double max , double min ,double accel, double v_ref , double v_real)
{
  double direction = 1;
  double v_diff = v_ref - v_real;
  // saturations on accelerations
  if(v_diff>=0)
  {
      v_diff = v_real + direction * accel*dt;
  }
  else if(v_diff<0)
  {
      v_diff = v_real - direction * accel*dt;
  }

  if (v_diff<min)
  {
      return min;
  }
  else if (v_diff>max)
  {
      return max;
  }
  else
  {
      return v_diff;
  }
}

void motion_cmd()
{
  if (path_stored) 
  {
    get_path_errors();
    path_following_controller();
    stop_mode(); 
   // motion_to_avoid_collision_at_first_rotation();
  }
  else
  {
    high_cmd.velocity[0] = 0.0;
    high_cmd.yaw_speed = 0.0;
    v_cmd = 0;
    w_cmd = 0;
    if (flag_orientation)
    {
      correction_final_theta();
      high_cmd.yaw_speed = w_cmd;
    }
  }
  /*if(fault_detected())
  {
    v_cmd = 0;
    w_cmd = 0;
    high_cmd.velocity[0] = 0.0;
    high_cmd.velocity[1] = 0.0;
    high_cmd.yaw_speed = 0.0;
    RCLCPP_INFO(this->get_logger(), "EMERGENCY STOP,  Error in RTABMAP or MPC , Fix it and restart the control node");
  }*/
}

void motion_to_avoid_collision_at_first_rotation()
{
  if(rotation_collision)
  {
    if(!position_collision)
    {
      v_cmd = 0;
      w_cmd = 0;
      high_cmd.velocity[0] = 0.1;
      high_cmd.yaw_speed = 0;
      RCLCPP_INFO(this->get_logger(), "Forward motion to a free-obstacle position");
    }
    else if(!backward_collision)
    {
      v_cmd = 0;
      w_cmd = 0;
      high_cmd.velocity[0] = -0.1;
      high_cmd.yaw_speed = 0;
      RCLCPP_INFO(this->get_logger(), "Backward motion! Searching free-obstacle rotation");
    }
    else if(backward_collision)
    {
      v_cmd = 0;
      w_cmd = 0;
      high_cmd.velocity[0] = v_cmd;
      high_cmd.yaw_speed = w_cmd;
      RCLCPP_INFO(this->get_logger(), "Robot can't rotate saftey, Stop Motion!");
    }
  }
}
    /*high_cmd.mode = 2;
    high_cmd.body_height = 0.0;
    high_cmd.gait_type = 1;*/

  /////////// [loop method] //////////
  void on_timer()
  {
    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
      t = tf_buffer_->lookupTransform(
        global_frame, local_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not find transform %s", ex.what());
      return;
    }
    odom_x = t.transform.translation.x;
    odom_y = t.transform.translation.y;
    odom_theta = get_yaw(t.transform.rotation);
  //  RCLCPP_INFO(this->get_logger(), "x , y , theta %f %f %f", odom_x, odom_y , odom_theta);
    
    if(StartMotion)
    {
      motion_cmd();  
      if (!robot_reached) // && simulation_node_is_active 
      {
          high_cmd.head[0] = 0xFE;
          high_cmd.head[1] = 0xEF;
        //  high_cmd.level_flag = HIGHLEVEL;
          high_cmd.mode = 2;      
          high_cmd.gait_type = 1;
          high_cmd.speed_level = 0;
          high_cmd.foot_raise_height = 0;
          high_cmd.body_height = 0; //.56;  //body_height_;
          high_cmd.euler[0]  = 0;
          high_cmd.euler[1] = 0;
          high_cmd.euler[2] = 0;
          high_cmd.velocity[1] = 0;
          high_cmd.reserve = 0;
          publisher_->publish(high_cmd);
      }
    }
    else if (StartSquat && !squat_reached)
    {
      time_squat = time_squat + 0.1;
      high_cmd.head[0] = 0xFE;
      high_cmd.head[1] = 0xEF;
    //  high_cmd.level_flag = HIGHLEVEL;
      high_cmd.mode = 1;      
      high_cmd.gait_type = 1;
      high_cmd.speed_level = 0;
      high_cmd.foot_raise_height = 0;
      high_cmd.euler[0]  = 0;
      high_cmd.euler[1] = 0;
      high_cmd.euler[2] = 0;
      high_cmd.velocity[0] = 0;
      high_cmd.velocity[1] = 0;
      high_cmd.yaw_speed = 0;
      high_cmd.reserve = 0;
      if(time_squat < 1)
      {
        high_cmd.body_height = -0.15; //.56;  //body_height_;
        publisher_->publish(high_cmd);
      }
      else
      {
        high_cmd.body_height = -0.25; //.56;  //body_height_;
        publisher_->publish(high_cmd);
        squat_reached = true;
      }     
    }
    else if (StartStand && !stand_reached)
    {
      time_stand = time_stand + 0.1;
      high_cmd.head[0] = 0xFE;
      high_cmd.head[1] = 0xEF;
    //  high_cmd.level_flag = HIGHLEVEL;
      high_cmd.mode = 1;      
      high_cmd.gait_type = 1;
      high_cmd.speed_level = 0;
      high_cmd.foot_raise_height = 0;
      high_cmd.euler[0]  = 0;
      high_cmd.euler[1] = 0;
      high_cmd.euler[2] = 0;
      high_cmd.velocity[0] = 0;
      high_cmd.velocity[1] = 0;
      high_cmd.yaw_speed = 0;
      high_cmd.reserve = 0;
      if(time_stand < 1)
      {
        high_cmd.body_height = -0.12; //.56;  //body_height_;
        publisher_->publish(high_cmd);
      }
      else
      {
        high_cmd.body_height = 0.01; //.56;  //body_height_;
        publisher_->publish(high_cmd);
        stand_reached = true;
      } 
    }
  }

};  // class ControlUnitree

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ControlUnitree>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
double get_yaw(geometry_msgs::msg::Quaternion q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}