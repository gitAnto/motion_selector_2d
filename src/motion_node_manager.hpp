/*********************************************************************
 *
 *  Copyright (c) 2014
 *  All rights reserved.
 *
 *  Software License Agreement (BSD License)
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
 *   * Neither the name of the Authors nor the names of its
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
 *
 *  Author: Antonio Petitti, Donato Di Paola on Apr, 2015
 *********************************************************************/
#ifndef MOTION_MANAGER_NODE_HPP
#define MOTION_MANAGER_NODE_HPP

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#include <string>
#include "BaitahCmd.h"
//#include "motion_personfollower.hpp"


	namespace motion {

		/**	
		* @class MotionManagerNode
		* @brief Implements the ROS node for the motion manager.
		*/
		class MotionManagerNode{

		public:
	 		/**
			* @brief Constructs a motion manager node
			*/
			MotionManagerNode();

	 		/**
			* @brief Destrucs a motion manager node
			*/
			~MotionManagerNode();

			void kitchenCallback(const geometry_msgs::Pose& msg);
			void standbyCallback(const geometry_msgs::Pose& msg);
			void rechargeCallback(const geometry_msgs::Pose& msg);

	 		/**
			* @brief Tracked target position callback function
			*/
			void positionTrackerCallback(const geometry_msgs::PoseStamped& target);

                        /**
                        * @brief robot position callback function
                        */
                        void robotPoseCallback(const geometry_msgs::PoseStamped& robot);

	 		/**
			* @brief Goal callback function
			*/
			void positionGoalCallback(const move_base_msgs::MoveBaseGoal& goal);

	 		/**
			* @brief Motion manager status callback function
			*/
			void statusCallback(const baitah_msgs::BaitahCmd& status);

                        /**
                        * @brief Stop motion: send zero velocities refs to the motors
                        */
			void stopMotion();

                        /**
                        * @brief waiting for exploration commands
                        */
//                        void exploration();

	 		/**
			* @brief Switch between motion functions
			*/
//			void motionFunctionSwitch(const std::string motionFunction);

			/**
			* @brief Send a goal to the action server
			*/
			void goToGoal();

                        /**
                        * @brief Get the status of the motion manager node
                        */
                        int getStatus();

                        /**
                        * @brief Get the param of the received command
                        */
			std::string getCmdParam();

                        /**
                        * @brief Get the status of the motion manager node
                        */
                        int getOldStatus();

                        /**
                        * @brief Get the status of the motion manager node
                        */
                        void setOldStatus(int);

                        /**
                        * @brief TODO
                        */
	                void setCbFlag(int flag);
                        
			/**
                        * @brief TODO
                        */
        	        int getCbFlag();

			/**
			* @brief Check the node status 
			*/
			bool isOk();

			/**
			* @brief does the robot reach the goal? 
			*/
			bool goal_reached();

			/**
			* @brief is the robot reaching the goal? 
			*/
			bool goal_active();

			/**
			* @brief control the heading of the robot
			*/
			void robot_control(const std::string motion_str);

			/**
			* @brief tell to the supervisor if the goal has been reached or not
			*/
			void result2supervisor(bool goal_success, const std::string motion_state);

			geometry_msgs::PoseStamped m_target_pose;
			geometry_msgs::PoseStamped m_robot_pose;

			move_base_msgs::MoveBaseGoal m_target_goal;
			move_base_msgs::MoveBaseGoal m_standby_goal;
                        move_base_msgs::MoveBaseGoal m_kitchen_goal;
                        move_base_msgs::MoveBaseGoal m_recharge_goal;


		private:

			ros::NodeHandle nh_;
			
			//publishers
			ros::Publisher m_vel_pub;
			ros::Publisher m_motion_status_pub;
			ros::Publisher m_motion_string_state_pub;
		
			//subscribers
			ros::Subscriber m_pose_tracker_sub;
			ros::Subscriber m_pose_robot_sub;
			ros::Subscriber m_pose_goal_sub;
			ros::Subscriber m_status_sub;

			ros::Subscriber m_kitchen_marker; 
			ros::Subscriber m_recharge_marker; 
			ros::Subscriber m_standby_marker;
				

			MoveBaseClient m_ac;
/*
			geometry_msgs::PoseStamped m_target_pose;
			geometry_msgs::PoseStamped m_robot_pose;

			move_base_msgs::MoveBaseGoal m_target_goal;
			move_base_msgs::MoveBaseGoal m_standby_goal;
                        move_base_msgs::MoveBaseGoal m_kitchen_goal;
                        move_base_msgs::MoveBaseGoal m_recharge_goal;
*/

			baitah_msgs::BaitahCmd m_status;
			int m_old_status;
			int m_cb_flag;

			motion::PoseFollower m_pose_follower_controller;

		};
	}


#endif // MOTION_MANAGER_NODE_HPP
