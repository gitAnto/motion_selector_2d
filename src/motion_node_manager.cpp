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

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <baitah_msgs/BaitahCmd.h>

#include "motion_posefollower.hpp"
#include "motion_node_manager.hpp"

#include <math.h>

//#include "read_txt_file.hpp"
#include <fstream>
#include <sstream>

#define M_PI 		3.14159265358979323846  /* pi */

// global var
move_base_msgs::MoveBaseGoal goal;
int 			     goal_reached_ = 0;
int 			     goal_sent     = 0;
int 			     cb_sent       = 0;
//


	namespace motion {

		MotionManagerNode::MotionManagerNode(): m_ac("move_base", true) //tell the action client that we want to spin a thread by default
			{
			m_vel_pub 		  = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
			m_motion_status_pub 	  = nh_.advertise<baitah_msgs::BaitahCmd>("/motion_manager_state",1);
			m_motion_string_state_pub = nh_.advertise<std_msgs::String>("/motion_status",1); 

			m_pose_tracker_sub = nh_.subscribe("/filter_node/person_pose_filtered", 1, &MotionManagerNode::positionTrackerCallback, this); 
//		m_pose_goal_sub = nh_.subscribe("/goal", 1, &MotionManagerNode::positionGoalCallback, this); 
			m_status_sub = nh_.subscribe("/motion_cmd", 1, &MotionManagerNode::statusCallback, this); 
    	m_pose_robot_sub = nh_.subscribe("/robot_pose", 1, &MotionManagerNode::robotPoseCallback, this); 

			m_kitchen_marker  = nh_.subscribe("/kitchen",  1, &MotionManagerNode::kitchenCallback,  this); 
			m_recharge_marker = nh_.subscribe("/recharge", 1, &MotionManagerNode::rechargeCallback, this); 
			m_standby_marker  = nh_.subscribe("/standby",  1, &MotionManagerNode::standbyCallback,  this); 
			
			//wait for the action server to come up
			while(!m_ac.waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			//readGoalsFromFile("goals.txt");
			std::ifstream goalsFile;
			std::string path = ros::package::getPath("motion_manager");
			path = path + "/src/goals.txt";
			ROS_INFO("Opening file %s",path.c_str());
      goalsFile.open(path.c_str());
			std::string frame_id, x_goal, y_goal, theta_goal; 

			//standby goal
			goalsFile >> frame_id;
			goalsFile >> x_goal;
			goalsFile >> y_goal;
                        goalsFile >> theta_goal;
			m_standby_goal.target_pose.pose.position.x = std::atof(x_goal.c_str());
                        m_standby_goal.target_pose.pose.position.y = std::atof(y_goal.c_str());
			m_standby_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::atof(theta_goal.c_str())*M_PI/180.0);
	                m_standby_goal.target_pose.header.frame_id = "map";
			               

			//ROS_INFO("Stanby position in: x=%f y=%f theta=%f",m_standby_goal.target_pose.pose.position.x,m_standby_goal.target_pose.pose.position.y,tf::getYaw(m_standby_goal.target_pose.pose.orientation));


                        //standby goal
                        goalsFile >> frame_id;
                        goalsFile >> x_goal;
                        goalsFile >> y_goal;
                        goalsFile >> theta_goal;
                        m_kitchen_goal.target_pose.pose.position.x = std::atof(x_goal.c_str());
                        m_kitchen_goal.target_pose.pose.position.y = std::atof(y_goal.c_str());
			m_kitchen_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::atof(theta_goal.c_str())*M_PI/180.0); 
			m_kitchen_goal.target_pose.header.frame_id = "map";

//			DEBUG INFO
//			ROS_INFO("theta from file = %f", std::atof(theta_goal.c_str())*M_PI/180.0);
//			std::cout<<"Info from file: xgoal " << x_goal << "ygoal " << y_goal << "tgoal " << theta_goal << std::endl;
//			ROS_INFO("Quaternion = ");
//			std::cout << tf::createQuaternionMsgFromYaw(M_PI) << std::endl;
                        //ROS_INFO("Kitchen position in: x=%f y=%f theta=%f",m_kitchen_goal.target_pose.pose.position.x,m_kitchen_goal.target_pose.pose.position.y,tf::getYaw(m_kitchen_goal.target_pose.pose.orientation));


                        //recharge goal
                        goalsFile >> frame_id;
                        goalsFile >> x_goal;
                        goalsFile >> y_goal;
                        goalsFile >> theta_goal;
                        m_recharge_goal.target_pose.pose.position.x = std::atof(x_goal.c_str());
                        m_recharge_goal.target_pose.pose.position.y = std::atof(y_goal.c_str());
			m_recharge_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::atof(theta_goal.c_str())*M_PI/180.0); 
			m_recharge_goal.target_pose.header.frame_id = "map";

                        std::cout<<"Info from file: xgoal " << x_goal << "ygoal " << y_goal << "tgoal " << theta_goal << std::endl;
                        //ROS_INFO("Recharge position in: x=%f y=%f theta=%f",m_recharge_goal.target_pose.pose.position.x,m_recharge_goal.target_pose.pose.position.y,tf::getYaw(m_recharge_goal.target_pose.pose.orientation));

			ROS_INFO("File loaded");
			goalsFile.close();

			//target goal initialization
			m_target_goal.target_pose.pose.position.x = 0.0;
                        m_target_goal.target_pose.pose.position.y = 0.0;
			m_target_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0); 
			m_target_goal.target_pose.header.frame_id = "map";

			 m_target_pose.pose.position.x  = 0.0;
			 m_target_pose.pose.position.y  = 0.0;
			 m_target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    			 // robot initialization
			 m_robot_pose.pose.position.x  = 0.0;
			 m_robot_pose.pose.position.y  = 0.0;
			 m_robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

			m_status.cmd_type = 0;			
		}

		MotionManagerNode::~MotionManagerNode(){
		}


		void MotionManagerNode::kitchenCallback(const geometry_msgs::Pose& msg)
		{
			m_kitchen_goal.target_pose.pose.position.x = msg.position.x;
      m_kitchen_goal.target_pose.pose.position.y = msg.position.y;
			//m_kitchen_goal.target_pose.pose.orientation = msg.orientation; 
		}

		void MotionManagerNode::standbyCallback(const geometry_msgs::Pose& msg)
		{
			m_standby_goal.target_pose.pose.position.x = msg.position.x;
      m_standby_goal.target_pose.pose.position.y = msg.position.y;
			//m_standby_goal.target_pose.pose.orientation = msg.orientation; 
		}

		void MotionManagerNode::rechargeCallback(const geometry_msgs::Pose& msg)
		{
			m_recharge_goal.target_pose.pose.position.x = msg.position.x;
      m_recharge_goal.target_pose.pose.position.y = msg.position.y;
			//m_recharge_goal.target_pose.pose.orientation = msg.orientation; 
		}

    void MotionManagerNode::robotPoseCallback(const geometry_msgs::PoseStamped& robot_msg){
                       // save inoformation coming from the topic
//			ROS_INFO("robot pose callback");
			
                        m_robot_pose = robot_msg;
                }

		void MotionManagerNode::positionTrackerCallback(const geometry_msgs::PoseStamped& target_msg){
//			ROS_INFO("tracker callback");
			// save inoformation coming from the topic
			m_target_pose = target_msg;
		}

    void MotionManagerNode::setCbFlag(int flag){
            m_cb_flag = flag;
    }

    int MotionManagerNode::getCbFlag(){
            return m_cb_flag;
    }

/*		void MotionManagerNode::positionGoalCallback(const move_base_msgs::MoveBaseGoal& goal_msg){
			ROS_INFO("position goal  callback");
			// save information coming from the topic
			m_target_goal = goal_msg;
		}*/

		void MotionManagerNode::statusCallback(const baitah_msgs::BaitahCmd& status_msg){

			ROS_INFO("status callback");
			int status_, current_status;

			// save information coming from the topic
			status_ = status_msg.cmd_type;

			current_status = getStatus();

			//if status is changed
			//if (status_ != current_status)// && current_status!=baitah_msgs::BaitahCmd::battery_low_cmd)
			//{
				//m_ac.cancelAllGoals();
				goal_reached_ = 0;
				goal_sent     = 0;
				cb_sent       = 0;
//				m_status = status_msg;
			//}

       m_status = status_msg;

			if (status_ == baitah_msgs::BaitahCmd::stop_cmd)
      {
				ROS_INFO("Stopping the robot..");
				stopMotion();
				m_status.cmd_type = baitah_msgs::BaitahCmd::dismissed_cmd;
				//setOldStatus(m_status.cmd_type);//robot in standby, but no moving
				goal_reached_ = 1;
			}

			ROS_INFO("Status: %d",m_status.cmd_type);
			
			setCbFlag(1);//probably useless --> verify!
		}

     void MotionManagerNode::stopMotion(){

//				m_ac.stopTrackingGoal();
//				m_ac.cancelGoal();
//				m_ac.cancelAllGoals();

			        geometry_msgs::Twist cmd_vel;

							cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;

              m_vel_pub.publish(cmd_vel);
				
							// stop!
							ros::Duration(2.0).sleep();
        };



      void MotionManagerNode::goToGoal(){

				m_ac.sendGoal(goal);

      };


      void MotionManagerNode::robot_control(const std::string motion_str){

							geometry_msgs::PoseStamped target_pose_local;			

							double x_local, y_local;
							double x_diff, y_diff;
							double theta_robot, theta_target_local;

							x_diff = m_target_pose.pose.position.x - m_robot_pose.pose.position.x;	
							y_diff = m_target_pose.pose.position.y - m_robot_pose.pose.position.y;
							//ROS_INFO("x robot = %f y robot = %f",m_robot_pose.pose.position.x, m_robot_pose.pose.position.y);
							//ROS_INFO("x target = %f y target = %f",m_target_pose.pose.position.x, m_target_pose.pose.position.y);
							//ROS_INFO("x_diff: %f  y_diff: %f",x_diff,y_diff);

							theta_robot = tf::getYaw(m_robot_pose.pose.orientation);
							//ROS_INFO("theta_robot = %f",theta_robot);

							//difference vector in robot coordinate frame --> Rot(-theta)
							x_local =   cos(theta_robot)*x_diff + sin(theta_robot)*y_diff;
							y_local =  -sin(theta_robot)*x_diff + cos(theta_robot)*y_diff; 			
							//ROS_INFO("x_local = %f and y_local = %f",x_local, y_local);

							target_pose_local.pose.position.x = x_local;
							target_pose_local.pose.position.y = y_local;

							theta_target_local = atan2(y_local, x_local);
							target_pose_local.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta_target_local);

							target_pose_local.header.stamp    = ros::Time::now();
							target_pose_local.header.frame_id = "base_link_203";

			
							//ROS_INFO("x_local: %f  y_local: %f",x_local,y_local);
							//ROS_INFO("theta_robot: %f",theta_robot);

							if(motion_str=="heading")
							{
								geometry_msgs::Twist cmd_vel;
								if(m_pose_follower_controller.headingControlLoop(target_pose_local, cmd_vel))
								 			m_vel_pub.publish(cmd_vel);
							}
							else
							{
								if(motion_str=="trajectory")
								{
									geometry_msgs::Twist cmd_vel;

									if(m_pose_follower_controller.trajectoryControlLoop(target_pose_local, cmd_vel))
							 		  			m_vel_pub.publish(cmd_vel);

								}
								else
								{
									ROS_WARN("Wrong robot control string in MotionManagerNode::robot_control");
								}
							}

        };

                
		void MotionManagerNode::result2supervisor(bool goal_success, const std::string motion_state){

			baitah_msgs::BaitahCmd mb_result;
			mb_result.cmd_object = baitah_msgs::BaitahCmd::robot;
		
			if(goal_success)
			{
				if(motion_state == "standby")
					mb_result.cmd_type = baitah_msgs::BaitahCmd::goal_reached_standby_cmd;
				else
				{
					if(motion_state == "interaction")
						mb_result.cmd_type = baitah_msgs::BaitahCmd::goal_reached_interaction_cmd;
					else
					{
						if(motion_state == "emergency")
							mb_result.cmd_type = baitah_msgs::BaitahCmd::goal_reached_emergency_cmd;
						else				
							ROS_WARN("Wrong message received in MotionManagerNode::result2supervisor");
					}
				}				
			}
			else
				mb_result.cmd_type = baitah_msgs::BaitahCmd::goal_notreached_cmd;

			m_motion_status_pub.publish(mb_result);
		}
                
		int MotionManagerNode::getStatus(){
			return m_status.cmd_type;
		}

		std::string MotionManagerNode::getCmdParam(){
			return m_status.param;
		}

                int MotionManagerNode::getOldStatus(){
                        return m_old_status;
                }

                void MotionManagerNode::setOldStatus(int old_status){
                        m_old_status = old_status;
                }

		bool MotionManagerNode::isOk(){
			return nh_.ok();
		}

		bool MotionManagerNode::goal_reached(){
			return m_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
		}

		bool MotionManagerNode::goal_active(){
			return m_ac.getState() == actionlib::SimpleClientGoalState::ACTIVE;
		}
	}





int main(int argc, char** argv){

	ros::init(argc, argv, "motion_node_manager");

	motion::MotionManagerNode motion_node;
 	ros::Rate loop_rate(50);

	while(motion_node.isOk())
	{

		int valid_str = 0;
		int status = motion_node.getStatus();

		if (status == baitah_msgs::BaitahCmd::stop_cmd)
		{

	//                ROS_INFO("motion_manager | node status: Stop");

		        //stop motion
		        motion_node.stopMotion();

			//robot in standby
	//		motion_node.setStatus();
	//		motion_node.setOldStatus(baitah_msgs::BaitahCmd::dismissed_cmd);

		        valid_str = 1;
		};


		if (status == baitah_msgs::BaitahCmd::emergency_cmd)
		{

			//ROS_INFO("motion_manager | node status: Interaction");

			if(!goal_reached_)
			{
				if(!goal_sent)
				{

					/*double x_diff, y_diff;

					x_diff = motion_node.m_target_pose.pose.position.x - motion_node.m_robot_pose.pose.position.x;	
					y_diff = motion_node.m_target_pose.pose.position.y - motion_node.m_robot_pose.pose.position.y;
					//ROS_INFO("x_diff = %f and y_diff = %f",x_diff, y_diff);

					//find position close to the target in /map
					double module_diff, phase_diff;
					double dist = 0.5;			
					double x,y;
					double yaw_target = tf::getYaw(motion_node.m_target_pose.pose.orientation);

					module_diff = sqrt(x_diff*x_diff+y_diff*y_diff);
					phase_diff  = atan2(y_diff, x_diff);

					//<dist> meters before
					module_diff = module_diff - dist;

					x = module_diff*cos(phase_diff) + motion_node.m_robot_pose.pose.position.x;
					y = module_diff*sin(phase_diff) + motion_node.m_robot_pose.pose.position.y;

			
				 	goal.target_pose.pose.position.x  = x;
					goal.target_pose.pose.position.y  = y;
					goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw_target + M_PI);//tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, phase_diff);
					*/

					double dist = 0.8;			
					double x = motion_node.m_target_pose.pose.position.x;
					double y = motion_node.m_target_pose.pose.position.y;
					double yaw_target = tf::getYaw(motion_node.m_target_pose.pose.orientation);

					x = x + dist*cos(yaw_target);
					y = y + dist*sin(yaw_target);

					goal.target_pose.pose.position.x = x;
					goal.target_pose.pose.position.y = y;
					goal.target_pose.pose.position.z = 0.0;

					goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_target + M_PI);

			                goal.target_pose.header.frame_id = "map";
			                goal.target_pose.header.stamp = ros::Time::now();

	//				m_target_goal.target_pose.pose.position.x = x;
	//				m_target_goal.target_pose.pose.position.y = y;

					ROS_INFO("Sending goal in x = %f [m], y = %f [m], w.r.t frame: /map", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);//,yaw*(180.0/3.14159));
					motion_node.goToGoal();
					goal_sent = 1;
		
				}
				else
				{
					if(!cb_sent)
					{
						if(motion_node.goal_reached())
						{
							goal_reached_ = 1;
							ROS_INFO("emergency goal reached");
							motion_node.result2supervisor(true, "emergency");
							cb_sent    = 1;
						}
						else
						{
							if(!motion_node.goal_active())
							{
								ROS_INFO("emergency goal not reached");
								motion_node.result2supervisor(false, "emergency");
								cb_sent    = 1;
							}
						}
					}
				}

			}


			valid_str = 1;
		};


		if (status == baitah_msgs::BaitahCmd::dismissed_cmd)
		{

			//ROS_INFO("motion_manager | node status: Standby");


			if(!goal_reached_)
			{
				if(!goal_sent)
				{
					goal = motion_node.m_standby_goal;
					motion_node.goToGoal();
					goal_sent = 1;
				}
				else
				{
					if(!cb_sent)
					{
						if(motion_node.goal_reached())
						{
							goal_reached_ = 1;
							ROS_INFO("standby goal reached");
							motion_node.result2supervisor(true, "standby");
							cb_sent    = 1;
						}
						else
						{
							if(!motion_node.goal_active())
							{
								ROS_INFO("standby goal not reached");
								motion_node.result2supervisor(false, "standby");
								cb_sent    = 1;
							}
						}
					}

				}
				
			}
			else 
			{
				motion_node.robot_control("heading");
			}

		
			valid_str = 1;
		};

		if (status == baitah_msgs::BaitahCmd::come_cmd)
		{

			//ROS_INFO("motion_manager | node status: Interaction");

			if(!goal_reached_)
			{
				if(!goal_sent)
				{

					/*double x_diff, y_diff;

					x_diff = motion_node.m_target_pose.pose.position.x - motion_node.m_robot_pose.pose.position.x;	
					y_diff = motion_node.m_target_pose.pose.position.y - motion_node.m_robot_pose.pose.position.y;
					//ROS_INFO("x_diff = %f and y_diff = %f",x_diff, y_diff);

					//find position close to the target in /map
					double module_diff, phase_diff;
					double dist = 0.5;			
					double x,y;

					module_diff = sqrt(x_diff*x_diff+y_diff*y_diff);
					phase_diff  = atan2(y_diff, x_diff);

					//<dist> meters before
					module_diff = module_diff - dist;

					x = module_diff*cos(phase_diff) + motion_node.m_robot_pose.pose.position.x;
					y = module_diff*sin(phase_diff) + motion_node.m_robot_pose.pose.position.y;

			
				 	goal.target_pose.pose.position.x  = x;
					goal.target_pose.pose.position.y  = y;*/
	
					double dist = 0.5;			
					double x = motion_node.m_target_pose.pose.position.x;
					double y = motion_node.m_target_pose.pose.position.y;
					double yaw_target = tf::getYaw(motion_node.m_target_pose.pose.orientation);

					x = x + dist*cos(yaw_target);
					y = y + dist*sin(yaw_target);

					goal.target_pose.pose.position.x = x;
					goal.target_pose.pose.position.y = y;
					goal.target_pose.pose.position.z = 0.0;

					goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_target + M_PI);

					//tf::Matrix3x3(goal.target_pose.pose.orientation).getRPY(roll, pitch, yaw); 

          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();

	//				m_target_goal.target_pose.pose.position.x = x;
	//				m_target_goal.target_pose.pose.position.y = y;

					ROS_INFO("Sending goal in x = %f [m], y = %f [m], w.r.t frame: /map", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);//,yaw*(180.0/3.14159));
					motion_node.goToGoal();
					goal_sent = 1;
		
				}
				else
				{
					if(!cb_sent)
					{
						if(motion_node.goal_reached())
						{
							goal_reached_ = 1;
							ROS_INFO("interaction goal reached");
							motion_node.result2supervisor(true, "interaction");
							cb_sent    = 1;
						}
						else
						{
							if(!motion_node.goal_active())
							{
								ROS_INFO("interaction goal not reached");
								motion_node.result2supervisor(false, "interaction");
								cb_sent    = 1;
							}
						}
					}
				}

			}


			valid_str = 1;
		};


		if (status == baitah_msgs::BaitahCmd::followme_cmd)
		{

			//ROS_INFO("motion_manager | node status: Active Monitoring");

			motion_node.robot_control("trajectory");

			valid_str = 1;
		};

		if (status == baitah_msgs::BaitahCmd::battery_low_cmd)
		{

			//ROS_INFO("motion_manager | node status: Recharge");

			if(!goal_reached_)
			{
				if(!goal_sent)
				{
					//go to the recharging station
					goal = motion_node.m_recharge_goal;
					goal.target_pose.header.stamp = ros::Time::now();
					motion_node.goToGoal();
					goal_sent = 1;
				
				}
				else
				{
					if(motion_node.goal_reached())
					{
						goal_reached_ = 1;
						ROS_INFO("recharge station reached");
						
					}
					else
					{
						if(!motion_node.goal_active())
						{
							ROS_INFO("recharge station cannot be reached");
							
						}
					}

				}
				
			}

			valid_str = 1;
		};

		if (status == baitah_msgs::BaitahCmd::goto_cmd || status == baitah_msgs::BaitahCmd::forward_cmd || status == baitah_msgs::BaitahCmd::rotate_cmd)
		{

			 //ROS_INFO("motion_manager | node status: Exploring");
			 //int old_status = motion_node.getOldStatus();

			 if(status==baitah_msgs::BaitahCmd::goto_cmd && motion_node.getCbFlag())
			 {
					//go to a given point in the environment
					if(!goal_reached_)
					{
						if(!goal_sent)
						{
							//go to kitchen
							goal = motion_node.m_kitchen_goal;
							goal.target_pose.header.stamp = ros::Time::now();
							motion_node.goToGoal();
							goal_sent = 1;
				
						}
						else
						{
							if(motion_node.goal_reached())
							{
								goal_reached_ = 1;
								ROS_INFO("kitchen reached");
						
							}
							else
							{
								if(!motion_node.goal_active())
								{
									ROS_INFO("kitchen cannot be reached");				
								}
							}

						}
				
					}
			  }
			  else //exploration
			  {			  
			  	//wait for exploration commands
				if ( (status == baitah_msgs::BaitahCmd::forward_cmd || status == baitah_msgs::BaitahCmd::rotate_cmd) && motion_node.getCbFlag())
	                        {
					//double x, y;
	                        	//double yaw;
					ROS_INFO("Command received..");

					if(status == baitah_msgs::BaitahCmd::forward_cmd)
					{

						//go to a given point in the environment
						if(!goal_reached_)
						{
							if(!goal_sent)
							{
								//go to the recharging station
							        goal.target_pose.pose.position.x    = 1.0;
							        goal.target_pose.pose.position.y    = 0.0;
					                        goal.target_pose.pose.orientation.x = 0.0;
					                        goal.target_pose.pose.orientation.y = 0.0;
					                        goal.target_pose.pose.orientation.z = 0.0;
								goal.target_pose.pose.orientation.w = 1.0;
							        //tf::Matrix3x3(goal.target_pose.pose.orientation).getRPY(roll, pitch, yaw); 

							        goal.target_pose.header.frame_id = "base_link_203";
							        goal.target_pose.header.stamp = ros::Time::now();

						        	ROS_INFO("Moving SARa 1 meter forward");//,yaw*(180.0/3.14159)$
	        	                	
								motion_node.goToGoal();
								goal_sent = 1;
							}
							else
							{
								if(!cb_sent)
								{
									if(motion_node.goal_reached())
									{
										goal_reached_ = 1;
										ROS_INFO("moved forward!!");
										motion_node.result2supervisor(true, "interaction");
										cb_sent    = 1;
						
									}
									else
									{
										if(!motion_node.goal_active())
										{
											ROS_INFO("there is not enough space for moving forward..");	
											motion_node.result2supervisor(false, "interaction");
											cb_sent    = 1;			
										}
									}
								}

							}
						}
					}
					else
					{
						//go to a given point in the environment
						if(!goal_reached_)
						{
							if(!goal_sent)
							{
								float rotation_angle_deg;
								rotation_angle_deg = std::atof(motion_node.getCmdParam().c_str());
								ROS_INFO("param: %s", motion_node.getCmdParam().c_str());

								goal.target_pose.pose.position.x = 0.0;
					                        goal.target_pose.pose.position.y = 0.0;
					                        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(rotation_angle_deg*M_PI/180.0);

					                        goal.target_pose.header.frame_id = "base_link_203";
					                        goal.target_pose.header.stamp = ros::Time::now();

					                        ROS_INFO("Sending to SARa rotation command of %f",rotation_angle_deg);//,yaw*(180.0/3.14159)$
								
								motion_node.goToGoal();
								goal_sent = 1;
							}
							else
							{
								if(!cb_sent)
								{
									if(motion_node.goal_reached())
									{
										goal_reached_ = 1;
										ROS_INFO("rotation completed!!");
										motion_node.result2supervisor(true, "interaction");
										cb_sent    = 1;
						
									}
									else
									{
										if(!motion_node.goal_active())
										{
											ROS_INFO("there is not enough space for rotating..");	
											motion_node.result2supervisor(false, "interaction");
											cb_sent    = 1;			
										}
									}
								}

							}
						}
					}

			
					motion_node.setCbFlag(0);

				}
			  }

			  valid_str = 1;
	
		};

		if (!valid_str){

			//ROS_ERROR("motion_manager | Status not valid!");
		};

			loop_rate.sleep();
			ros::spinOnce();

			motion_node.setOldStatus(status);
	}

	return 0;
};
