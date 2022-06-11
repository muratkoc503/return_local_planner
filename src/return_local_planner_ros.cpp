#include "return_local_planner/return_local_planner_ros.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(return_local_planner::ReturnLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace return_local_planner{

	void ReturnLocalPlannerROS::reconfigureCB(return_local_planner::ReturnLocalPlannerRosConfig &config, uint32_t level){
		// set private attribute
		odom_topic_ = config.odom_topic;
		linear_vel_ = config.linear_vel;
		rotate_to_goal_vel_ = config.rotate_to_goal_vel;

		// set private PID variables
		k_p_ = config.kp;
		k_i_ = config.ki;
		k_d_ = config.kd;
		period_ = config.period;

		// update generic local planner params - limits
		base_local_planner::LocalPlannerLimits limits;
		limits.xy_goal_tolerance = config.xy_goal_tolerance;
		limits.yaw_goal_tolerance = config.yaw_tolerance;
		limits.max_vel_theta = config.max_vel_theta;
		planner_util_.reconfigureCB(limits, true);
	}

	ReturnLocalPlannerROS::ReturnLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false), odom_helper_("odom"),latched_rotate_to_goal_(false) {}

	ReturnLocalPlannerROS::ReturnLocalPlannerROS(std::string name, tf2_ros::Buffer* tf, 
                costmap_2d::Costmap2DROS* costmap_ros): costmap_ros_(NULL),
			    tf_(NULL), initialized_(false), odom_helper_("odom"), latched_rotate_to_goal_(false){
		initialize(name, tf, costmap_ros);
         }

	ReturnLocalPlannerROS::~ReturnLocalPlannerROS() {}

	void ReturnLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, 
                costmap_2d::Costmap2DROS* costmap_ros){

		// check if the plugin is already initialized
		if(!initialized_){	
			costmap_ros_ = costmap_ros;
			tf_ = tf;
			
			server_ = new dynamic_reconfigure::Server<return_local_planner::ReturnLocalPlannerRosConfig>(nh_);
			dynamic_reconfigure::Server<return_local_planner::ReturnLocalPlannerRosConfig>::CallbackType cb;
			cb = boost::bind(&ReturnLocalPlannerROS::reconfigureCB, this, _1, _2);
			server_->setCallback(cb);

			costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
			costmap_ros_->getRobotPose(current_pose_);

        	odom_helper_.setOdomTopic( odom_topic_ );
			planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

			initialized_ = true;
		}
		else
		{
			ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}

	bool ReturnLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{

		// check if plugin initialized
		if(!initialized_){
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

		latchedStopRotateController_.resetLatching();
		goal_reached_ = false;
		
		ROS_INFO("Got new plan");

		// eğer planner_util_ kullanmayacaksan global planı aşağıdaki şekilde kullanabilirsin.
		// plan.clear();
		// plan = orig_global_plan; 

		planner_util_.setPlan(orig_global_plan);	

		return true;
	}

	bool ReturnLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{

		// check if plugin initialized
		if(!initialized_){
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

		std::vector<geometry_msgs::PoseStamped> transformed_plan;

		if ( ! costmap_ros_->getRobotPose(current_pose_)) {
     		ROS_ERROR("Could not get robot pose");
      	return false;
    	}
		
		if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      		ROS_ERROR("Could not get local plan");
      	return false;
    	}
		
		if(transformed_plan.empty()) {
			ROS_WARN_NAMED("return_local_planner", "Received an empty transformed plan.");
		return false;
		}

		std::cout << "transformed_plan_size: " << transformed_plan.size() << std::endl;
			
		nav_msgs::Odometry base_odom;
		geometry_msgs::PoseStamped vel;
  		odom_helper_.getOdom(base_odom);
		odom_helper_.getRobotVel(vel);

		if(latchedStopRotateController_.isPositionReached(&planner_util_,current_pose_)){
			latched_rotate_to_goal_ = true;
			std::cout << "---------- position reached --------------" << std::endl;
		}
		else if( ! latched_rotate_to_goal_){
			cmd_vel = AngularPID( base_odom, transformed_plan[0], transformed_plan[1] );
			std::cout << "------------ PID --------------" << std::endl;
		}
		else if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)){
			this->goal_reached_ = true;
			std::cout << "------------ goal reached --------------" << std::endl;
		}
		else if ( latched_rotate_to_goal_){
			std::cout << "------------ rotate --------------" << std::endl;
			double goal_theta = tf2::getYaw(transformed_plan.back().pose.orientation);
			double robot_theta = tf2::getYaw(current_pose_.pose.orientation);
			// rotateToGoal look angle difference for shortest for returning
			double rotate_to_goal = ReturnLocalPlannerROS::rotateToGoal(current_pose_, goal_theta);
			cmd_vel.angular.z = rotate_to_goal;
			cmd_vel.linear.x = 0.0;
			if( abs(goal_theta - robot_theta) < planner_util_.getCurrentLimits().yaw_goal_tolerance ){
				// latched_rotate_to_goal_ = false;
				std::cout << "------------ rotate reached --------------" << std::endl;
				this->goal_reached_ = true;
			}
		}
		return true;
	}

	bool ReturnLocalPlannerROS::isGoalReached()
	{
		if(!this->initialized_){
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}
		if(goal_reached_){
			ROS_INFO("goal reached!!");
			latched_rotate_to_goal_ = false;
		}
		// this info comes from compute velocity commands:
		return this->goal_reached_;

	}

	geometry_msgs::Twist ReturnLocalPlannerROS::AngularPID(nav_msgs::Odometry& base_odometry, 
						geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal){
		geometry_msgs::Twist velocity;
		// init for goal yaw from plan[i] to plan[j]; j=i+1: in this code
		double start_x = start.pose.position.x;
		double start_y = start.pose.position.y;
		double goal_x = goal.pose.position.x;
		double goal_y = goal.pose.position.y;
		double diff_x = goal_x - start_x;
		double diff_y = goal_y - start_y;
		double goal_yaw = atan2(diff_y, diff_x);
		
		// float th = tf2::getYaw(base_odometry.pose.pose.orientation);
		float angle = goal_yaw;
		float th = tf2::getYaw(current_pose_.pose.orientation);
		float zw = 0;
		float error = 0;      float error1 = 0;     float error2 = 0;     float error3 = 0;
		float th1 = 0;        
		int zwx = 0;		  float zw_old = 0;

		double error_angle = angle - th;
		if(error_angle > M_PI) error_angle -= (2* M_PI);
		if(error_angle < -M_PI) error_angle += (2* M_PI);

		double vel_ang = base_odometry.twist.twist.angular.z;
		error = error_angle - vel_ang;

		zw = k_p_*error + k_i_*period_*(angle-(th+th1)/2.0) +
		(k_d_/(6*period_))*((error-error3)+3*(error1-error2)) + zw_old;
		
		th1 = th;
		error3 = error2;
		error2 = error1;
		error1 = error;
		zw_old = zw;

		if(zw>planner_util_.getCurrentLimits().max_vel_theta)
			zw = planner_util_.getCurrentLimits().max_vel_theta;
		
		if(zw > 1)					// burasıyla sonra ilgilen
			velocity.linear.x = 0.0;	// zero linear velocity
		else
			velocity.linear.x = linear_vel_;

			velocity.angular.z = zw;

		return velocity;
}

	double ReturnLocalPlannerROS::rotateToGoal(geometry_msgs::PoseStamped& global_pose, double goal_th){
		double yaw = tf2::getYaw(global_pose.pose.orientation);
		double ang_diff = angles::shortest_angular_distance(yaw, goal_th);
		double vel = 0;

		if(ang_diff < 0)
			vel = -rotate_to_goal_vel_;
		else
			vel = rotate_to_goal_vel_;

		return vel;
}

}
