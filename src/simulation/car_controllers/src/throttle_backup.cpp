/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <car_controllers/throttle_controller.h>
#include <pluginlib/class_list_macros.h>

#define MAX_SPEED_F 2
#define MAX_SPEED_R -1

namespace controller_ns {

	ThrottleController::ThrottleController(): command_(0), loop_count_(0){}

	ThrottleController::~ThrottleController(){
		sub_command_.shutdown();
	}

	bool ThrottleController::init(hardware_interface::EffortJointInterface *robot, const std::string &joint_name, const control_toolbox::Pid &pid){
		pid_controller_ = pid;
		joint_ = robot->getHandle(joint_name);	// Get joint handle from hardware interface
		return true;
	}

	bool ThrottleController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n){
		std::string joint_name; 	// Get joint name from parameter server
		if (!n.getParam("joint", joint_name)) {
			ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
			return false;
		}
		joint_ = robot->getHandle(joint_name);	// Get joint handle from hardware interface
		if (!pid_controller_.init(ros::NodeHandle(n, "pid"))){ 	// Load PID Controller using gains set on parameter server
			return false;
		}
		// Start realtime state publisher
		controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));
		// Start command subscriber
		sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &ThrottleController::setCommandCB, this);
		return true;
	}

	void ThrottleController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup){
		pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
	}

	void ThrottleController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup){
		pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
	}

	void ThrottleController::getGains(double &p, double &i, double &d, double &i_max, double &i_min){
		bool dummy;
		pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
	}

	void ThrottleController::printDebug(){
		pid_controller_.printValues();
	}

	std::string ThrottleController::getJointName(){
		return joint_.getName();
	}

	// Set the joint velocity command
	void ThrottleController::setCommand(double cmd){
		if(cmd > MAX_SPEED_F){
			cmd = MAX_SPEED_F;
		}
		else if(cmd < MAX_SPEED_R){
			cmd = MAX_SPEED_R;
		}
		else{
			command_ = cmd;
		}	
	}


	// Return the current velocity command
	void ThrottleController::getCommand(double& cmd){
		cmd = command_;
	}

	void ThrottleController::starting(const ros::Time& time){
		command_ = 0.0;
		pid_controller_.reset();
	}

	void ThrottleController::update(const ros::Time& time, const ros::Duration& period){
		double error = command_ - joint_.getVelocity();
		// Set the PID error and compute the PID command with nonuniform time step size.
		// The derivative error is computed from the change in the error and the timestep dt.
		double commanded_effort = pid_controller_.computeCommand(error, period);
		joint_.setCommand(commanded_effort);

		if(loop_count_ % 10 == 0){
			if(controller_state_publisher_ && controller_state_publisher_->trylock()){
				controller_state_publisher_->msg_.header.stamp = time;
				controller_state_publisher_->msg_.set_point = command_;
				controller_state_publisher_->msg_.process_value = joint_.getVelocity();
				controller_state_publisher_->msg_.error = error;
				controller_state_publisher_->msg_.time_step = period.toSec();
				controller_state_publisher_->msg_.command = commanded_effort;
				double dummy;
				bool antiwindup;
				getGains(controller_state_publisher_->msg_.p, controller_state_publisher_->msg_.i, controller_state_publisher_->msg_.d, controller_state_publisher_->msg_.i_clamp, dummy, antiwindup);
				controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
				controller_state_publisher_->unlockAndPublish();
			}
		}
		loop_count_++;
	}

	void ThrottleController::setCommandCB(const std_msgs::Float64ConstPtr& msg){
		command_ = msg->data;
	}


} // namespace

PLUGINLIB_EXPORT_CLASS(controller_ns::ThrottleController, controller_interface::ControllerBase)
