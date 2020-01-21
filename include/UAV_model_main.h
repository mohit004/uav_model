#pragma once

#include <NMPC_PC_main.h>
#include <UAV_model.h>
#include <sensor_msgs/Imu.h>


extern double sampleTime;

struct uav_struct_ uav_struct;
struct UAV_MODEL::publish_struct_ publish_struct;

Eigen::VectorXd Uref(NMPC_NU);
Eigen::VectorXd W(NMPC_NY);

std::vector<double> predicted_state;
std::vector<double> predicted_state_last;
std::vector<double> outer_nmpc_cmd;

int req_inner_loop_itr;

double t_loop;

// Publishers
ros::Publisher exeTime_pub;
ros::Publisher states_position_pub;
ros::Publisher states_velocity_pub;
ros::Publisher states_acceleration_pub;
ros::Publisher states_attitude_pub;
ros::Publisher states_rate_pub;
ros::Publisher local_vel_rates_pub;
ros::Publisher local_acc_pub;
ros::Publisher FxFyFz_disturb_pub;
