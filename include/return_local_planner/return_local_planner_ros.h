
#ifndef SIMPLE_LOCAL_PLANNER_ROS_H_
#define SIMPLE_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/local_planner_limits.h>
// msgs
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>
// boost classes  TODO do I need?
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
// other
#include <array>
#include <vector>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <angles/angles.h>

#include <dynamic_reconfigure/server.h>
#include <return_local_planner/ReturnLocalPlannerRosConfig.h>

using namespace std;
// definitions
#define D2R 0.0174532925      // = 3.14159265/180

namespace return_local_planner{

  class ReturnLocalPlannerROS : public nav_core::BaseLocalPlanner{

    public:
   
      ReturnLocalPlannerROS();
      ReturnLocalPlannerROS(std::string name, tf2_ros::Buffer* tf, 
                costmap_2d::Costmap2DROS* costmap_ros);
      ~ReturnLocalPlannerROS();

      void reconfigureCB(return_local_planner::ReturnLocalPlannerRosConfig &config, uint32_t level);

      void initialize(std::string name, tf2_ros::Buffer* tf, 
                costmap_2d::Costmap2DROS* costmap_ros);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();

      /**
       * @brief angular PID controller for robot sharp returning and follow global plan as pid controller
       * 
       * @param base_odometry robot odometry, in this code, come from odomHelper class(override)
       * @param start start pose of robot which pid calculate start to goal
       * @param goal goal(target) pose of robot which pid calculate start to goal
       * @return geometry_msgs::Twist velocity which must be publish /cmd_vel or vel topic
       */
      geometry_msgs::Twist AngularPID(nav_msgs::Odometry& base_odometry, 
                geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);
     
     /**
      * @brief once goal reached, use this function rotate to goal angle
      * without xy_goal tolerance. after you should use goal_reached_ = true
      * 
      * @param global_pose robot current pose
      * @param goal_th     goal yaw theta. i use end pose of global plan. (global_plan.back())
      * @return double     angular velocity 
      */
      double rotateToGoal(geometry_msgs::PoseStamped& global_pose, double goal_th);

    private:
      ros::NodeHandle nh_;

      base_local_planner::LocalPlannerUtil planner_util_;
      base_local_planner::OdometryHelperRos odom_helper_;
      base_local_planner::LatchedStopRotateController latchedStopRotateController_;
      dynamic_reconfigure::Server<return_local_planner::ReturnLocalPlannerRosConfig> *server_;

      // std::vector<geometry_msgs::PoseStamped> plan;
      geometry_msgs::PoseStamped current_pose_;   // come from costmap
      costmap_2d::Costmap2DROS* costmap_ros_;  
      tf2_ros::Buffer* tf_; 

      // CCONFIG PARAM ATTRIBUTE
      std::string odom_topic_;
      double linear_vel_;
      double rotate_to_goal_vel_;
      //-- PID --
      double k_p_, k_i_, k_d_; 
      double period_;

      // Flags
      bool goal_reached_, initialized_, latched_rotate_to_goal_;
  };
};

void base_local_planner::OdometryHelperRos::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	boost::mutex::scoped_lock lock(odom_mutex_);
	base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
 	base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
 	base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    base_odom_.child_frame_id = msg->child_frame_id;

	/**
	 * @brief override base_local_planner::OdometryHelperRos::odomCallback function
	 * odometryHelper induced only twist.twist message
	 * in override, pose.pose message add this function
	 */
	base_odom_.pose.pose.position.x = msg->pose.pose.position.x;
	base_odom_.pose.pose.position.y = msg->pose.pose.position.y;
	base_odom_.pose.pose.position.z = msg->pose.pose.position.z;

	base_odom_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
	base_odom_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
	base_odom_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
	base_odom_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
}

#endif

