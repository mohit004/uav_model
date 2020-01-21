#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
// #include<mavros_msgs/ExtendedState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/Thrust.h>
#include <tf/tf.h>

#include <NMPC_PC.h>


//extern double sampleTime;

//extern NMPCworkspace nmpcWorkspace;
//extern NMPCvariables nmpcVariables;

// Subscribers
ros::Subscriber state_sub;
ros::Subscriber trajectory_start_sub;
ros::Subscriber nmpc_start_sub;
ros::Subscriber learn_switch_sub;

ros::Subscriber ref_trajectory_sub;
ros::Subscriber ref_velocity_sub;
ros::Subscriber local_pos_sub;
ros::Subscriber local_vel_rates_sub;
ros::Subscriber nmpc_weights_switch_sub;
ros::Subscriber nmpc_weights_Wx_sub;
ros::Subscriber nmpc_weights_Wu_sub;

// Publishers
ros::Publisher att_throttle_pub;
ros::Publisher attitude_pub;
ros::Publisher nmpc_cmd_rpy_pub;
ros::Publisher nmpc_cmd_Fz_pub;
ros::Publisher nmpc_cmd_exeTime_pub;
ros::Publisher nmpc_cmd_kkt_pub;
ros::Publisher nmpc_cmd_obj_pub;

Eigen::Vector3d ref_trajectory, ref_velocity;
int ref_traj_type;
double t, t_pc_loop;

std::vector<double> current_pos_att;
std::vector<double> current_vel_rate;

weights_struct_ nmpc_weights_struct;

