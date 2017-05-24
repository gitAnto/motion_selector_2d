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
#ifndef MOTION_POSEFOLLOWER_HPP
#define MOTION_POSEFOLLOWER_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>



	namespace motion {
		/**	
		* @class PoseFollower
		* @brief Computes control velocities for a robot given the target pose, a costmap, and the robot's position.
		*/
		class PoseFollower{

		public:
	 		/**
			* @brief Constructs a pose follower controller
			*/
			PoseFollower();
			  
	 		/**
			* @brief  Destructs a pose follower controller
			*/
			~PoseFollower();

	 		/**
			* @brief  pose trajectory following control
			*/
	  		bool trajectoryControlLoop(const geometry_msgs::PoseStamped& target, geometry_msgs::Twist& cmd_vel);

	 		/**
			* @brief  heading control
			*/
			bool headingControlLoop(const geometry_msgs::PoseStamped& target, geometry_msgs::Twist& cmd_vel);		

	 		/**
			* @brief  clear cost map
			*/
			void clearCostmapWindows(double size_x, double size_y);

		private:
			base_local_planner::TrajectoryPlannerROS m_tp;	
			tf::TransformListener m_tf;
			costmap_2d::Costmap2DROS m_costmap;

			bool m_invalid_plan;
			double m_circumscribed_radius;
			double m_clearing_radius;
			
			double m_dist_min;
			double m_dist_max;

		};
	}


#endif // MOTION_POSEFOLLOWER_HPP
