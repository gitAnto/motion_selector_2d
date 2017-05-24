/*********************************************************************
 *
 *  Copyright (c) 2014.
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
 *   * Neither the name of the AuthorsA nor the names of its
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

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include "motion_posefollower.hpp"



	namespace motion {

		PoseFollower::PoseFollower():m_tf(ros::Duration(10)),m_costmap("local_costmap", m_tf),m_circumscribed_radius(0.46),m_clearing_radius(0.46){
			m_tp.initialize("my_trajectory_planner", &m_tf, &m_costmap);
			m_costmap.start();
			m_invalid_plan = false;

			m_dist_min = 1.5;
			m_dist_max = 2.5;
		}

		PoseFollower::~PoseFollower(){
			m_costmap.stop();
		}

		bool PoseFollower::trajectoryControlLoop(const geometry_msgs::PoseStamped& target, geometry_msgs::Twist& cmd_vel){

			double d = sqrt(pow(target.pose.position.x,2) + pow(target.pose.position.y,2));

			if( d > m_dist_max || d < m_dist_min || m_invalid_plan) {

				headingControlLoop(target, cmd_vel);

				m_invalid_plan = false;
				}	
			else {
				// Trajectory Planner
				
				 ROS_INFO("Trajectory Planner...");
		
				double target_x, target_y, T;
				double target_d, d_t, step;
				geometry_msgs::PoseStamped real_target;
				std::vector<geometry_msgs::PoseStamped> plan;
	
				real_target = target;			

				T = atan2(target.pose.position.y, target.pose.position.x);
				d_t = 0.30;
				
				real_target.pose.position.x = d_t*cos(T);
				real_target.pose.position.y = d_t*sin(T);
				real_target.pose.orientation = tf::createQuaternionMsgFromYaw(T);

				real_target.header.frame_id = "/base_link_203"; 
				real_target.header.stamp = ros::Time::now();

				plan.push_back(real_target);
		
				
				m_invalid_plan = false;
	
				ROS_INFO("Plan setting");
	
				if(!m_tp.setPlan(plan)) {
					ROS_ERROR("Failed to set the plan!");
					return false;			
				} 
				
				if(!m_tp.computeVelocityCommands(cmd_vel)){
					m_invalid_plan = true;
					ROS_ERROR("The planner could not find a valid plan!");
					return false;
				}			
			}
			return true;					
		}
	
		bool PoseFollower::headingControlLoop(const geometry_msgs::PoseStamped& target, geometry_msgs::Twist& cmd_vel){
	
	 	   	// Rotation Controller - [P]
	 	   	double e_k, theta_star, theta_m;     
	 	     	double Kp;
	 	     	double omega;

			ROS_INFO("Heading control");

			theta_star = 0;
	 	     	theta_m = atan2(target.pose.position.y, target.pose.position.x);
	 	     	e_k = theta_star - theta_m; // position error
	 
			Kp = 1.0;

			omega = - (Kp*e_k);  // controller equation
		        
		     	cmd_vel.angular.z = omega;
	
			return true;					
		}

		void PoseFollower::clearCostmapWindows(double size_x, double size_y){
	     	
			tf::Stamped<tf::Pose> global_pose;
	 
	 		std::vector<geometry_msgs::Point> clear_poly;
	     		geometry_msgs::Point pt;
	 
	     		//clear the costmap
	     		m_costmap.getRobotPose(global_pose);
	 
	     		clear_poly.clear();
	    	 	double x = global_pose.getOrigin().x();
	     		double y = global_pose.getOrigin().y();
	 
	     		pt.x = x - size_x / 2;
	    		pt.y = y - size_x / 2;
	     		clear_poly.push_back(pt);
	 
	     		pt.x = x + size_x / 2;
	     		pt.y = y - size_x / 2;
	     		clear_poly.push_back(pt);
	 
	     		pt.x = x + size_x / 2;
	     		pt.y = y + size_x / 2;
	     		clear_poly.push_back(pt);
	 
	     		pt.x = x - size_x / 2;
	     		pt.y = y + size_x / 2;
	     		clear_poly.push_back(pt);
	 
	     		m_costmap.setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
		}
	}
	
