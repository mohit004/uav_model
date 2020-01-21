/**
 * @file   UAV_model_dnn_main_velRef.cpp
 * @author Mohit Mehndiratta
 * @date   December 2018
 *
 * @copyright
 * Copyright (C) 2018.
 */

#include <UAV_model_main.h>

using namespace Eigen;
using namespace ros;

double sampleTime = 0.01;

std_msgs::Bool trajectory_start_flag;
void trajectory_start_cb(const std_msgs::Bool::ConstPtr& msg)
{
  trajectory_start_flag = *msg;
}
float nmpc_start_flag;
void nmpc_start_cb(const std_msgs::Bool::ConstPtr& msg)
{
  nmpc_start_flag = msg->data;
}

void ref_trajectory_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
  ref_trajectory << msg->x, msg->y, msg->z;
}
void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}

double radtodeg = 180/M_PI;

void NMPC_PC::publish_rpyFz(struct command_struct &commandstruct)
{
  std::vector<double> rpy_vec = {commandstruct.roll_ang, commandstruct.pitch_ang, commandstruct.yaw_ang};
  std_msgs::Float64MultiArray rpy_cmd;
  rpy_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
  rpy_cmd.layout.dim[0].size = rpy_vec.size();
  rpy_cmd.layout.dim[0].stride = 1;
  rpy_cmd.layout.dim[0].label = "Roll, Pitch, Yaw (rad)";
  rpy_cmd.data.clear();
  rpy_cmd.data.insert(rpy_cmd.data.end(), rpy_vec.begin(), rpy_vec.end());
  nmpc_cmd_rpy_pub.publish(rpy_cmd);

  std::vector<double> Fz_vec = {commandstruct.Fz, commandstruct.Fz_scaled};
  std_msgs::Float64MultiArray Fz_cmd;
  Fz_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
  Fz_cmd.layout.dim[0].size = Fz_vec.size();
  Fz_cmd.layout.dim[0].stride = 1;
  Fz_cmd.layout.dim[0].label = "Fz (N), Fz_scaled";
  Fz_cmd.data.clear();
  Fz_cmd.data.insert(Fz_cmd.data.end(), Fz_vec.begin(), Fz_vec.end());
  nmpc_cmd_Fz_pub.publish(Fz_cmd);
}

void UAV_MODEL::publish_states()
{
  std_msgs::Float64MultiArray states_position_cmd;
  states_position_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
  states_position_cmd.layout.dim[0].size = publish_struct.states_position_vec.size();
  states_position_cmd.layout.dim[0].stride = 1;
  states_position_cmd.layout.dim[0].label = "x, y, z (m)";
  states_position_cmd.data.clear();
  states_position_cmd.data.insert(states_position_cmd.data.end(), publish_struct.states_position_vec.begin(),
                                  publish_struct.states_position_vec.end());
  states_position_pub.publish(states_position_cmd);

  std_msgs::Float64MultiArray states_velocity_cmd;
  states_velocity_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
  states_velocity_cmd.layout.dim[0].size = publish_struct.states_velocity_vec.size();
  states_velocity_cmd.layout.dim[0].stride = 1;
  states_velocity_cmd.layout.dim[0].label = "u, v, w (m/s)";
  states_velocity_cmd.data.clear();
  states_velocity_cmd.data.insert(states_velocity_cmd.data.end(), publish_struct.states_velocity_vec.begin(),
                                  publish_struct.states_velocity_vec.end());
  states_velocity_pub.publish(states_velocity_cmd);

  std::vector<double> states_attitude_vec_deg = {radtodeg*publish_struct.states_attitude_vec[0],
                                                 radtodeg*publish_struct.states_attitude_vec[1],
                                                 radtodeg*publish_struct.states_attitude_vec[2]};
  std_msgs::Float64MultiArray states_attitude_cmd;
  states_attitude_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
  states_attitude_cmd.layout.dim[0].size = publish_struct.states_attitude_vec.size();
  states_attitude_cmd.layout.dim[0].stride = 1;
  states_attitude_cmd.layout.dim[0].label = "phi, theta, psi (deg)";
  states_attitude_cmd.data.clear();
  states_attitude_cmd.data.insert(states_attitude_cmd.data.end(), states_attitude_vec_deg.begin(),
                                  states_attitude_vec_deg.end());
  states_attitude_pub.publish(states_attitude_cmd);

  std_msgs::Float64MultiArray states_rate_cmd;
  states_rate_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
  states_rate_cmd.layout.dim[0].size = publish_struct.states_rate_vec.size();
  states_rate_cmd.layout.dim[0].stride = 1;
  states_rate_cmd.layout.dim[0].label = "p, q, r (rad/s)";
  states_rate_cmd.data.clear();
  states_rate_cmd.data.insert(states_rate_cmd.data.end(), publish_struct.states_rate_vec.begin(),
                              publish_struct.states_rate_vec.end());
  states_rate_pub.publish(states_rate_cmd);

  geometry_msgs::TwistStamped local_vel_rates_msg;
  local_vel_rates_msg.header.stamp = ros::Time::now();
  local_vel_rates_msg.twist.linear.x = publish_struct.states_velocity_vec[0];
  local_vel_rates_msg.twist.linear.y = publish_struct.states_velocity_vec[1];
  local_vel_rates_msg.twist.linear.z = publish_struct.states_velocity_vec[2];
  local_vel_rates_msg.twist.angular.x = publish_struct.states_rate_vec[0];
  local_vel_rates_msg.twist.angular.y = publish_struct.states_rate_vec[1];
  local_vel_rates_msg.twist.angular.z = publish_struct.states_rate_vec[2];
  local_vel_rates_pub.publish(local_vel_rates_msg);

  sensor_msgs::Imu local_acc_msg;
  local_acc_msg.header.stamp = ros::Time::now();
  local_acc_msg.linear_acceleration.x = publish_struct.states_acceleration_vec[0];
  local_acc_msg.linear_acceleration.y = publish_struct.states_acceleration_vec[1];
  local_acc_msg.linear_acceleration.z = publish_struct.states_acceleration_vec[2];
  local_acc_pub.publish(local_acc_msg);

  std_msgs::Float64MultiArray FxFyFz_disturb_msg;
  FxFyFz_disturb_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  FxFyFz_disturb_msg.layout.dim[0].size = publish_struct.FxFyFz_disturb_vec.size();
  FxFyFz_disturb_msg.layout.dim[0].stride = 1;
  FxFyFz_disturb_msg.layout.dim[0].label = "dist_x, dist_y, dist_z (N)";
  FxFyFz_disturb_msg.data.clear();
  FxFyFz_disturb_msg.data.insert(FxFyFz_disturb_msg.data.end(), publish_struct.FxFyFz_disturb_vec.begin(),
                                publish_struct.FxFyFz_disturb_vec.end());
  FxFyFz_disturb_pub.publish(FxFyFz_disturb_msg);
}

int main(int argc, char **argv)
{
  req_inner_loop_itr = 1;

  ros::init(argc, argv, "UAV_model_main");
  ros::NodeHandle nh;

  trajectory_start_sub = nh.subscribe<std_msgs::Bool>("trajectory_on", 1, trajectory_start_cb);
  nmpc_start_sub = nh.subscribe<std_msgs::Bool>("nmpc_start", 1, nmpc_start_cb);
  ref_trajectory_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/pose", 1, ref_trajectory_cb);
  ref_velocity_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);

// ----------
// Publishers
// ----------
  nmpc_cmd_rpy_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/rpy", 1, true);
  nmpc_cmd_Fz_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/Fz_FzScaled", 1, true);
  states_position_pub = nh.advertise<std_msgs::Float64MultiArray>("UAV_model/position", 1, true);
  states_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("UAV_model/velocity", 1, true);
  states_attitude_pub = nh.advertise<std_msgs::Float64MultiArray>("UAV_model/attitude", 1, true);
  states_rate_pub = nh.advertise<std_msgs::Float64MultiArray>("UAV_model/rate", 1, true);
  local_vel_rates_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/mocap/velocity", 1, true);
  local_acc_pub = nh.advertise<sensor_msgs::Imu>("mavros/imu/data", 1, true);
  FxFyFz_disturb_pub = nh.advertise<std_msgs::Float64MultiArray>("UAV_model/FxFyFz_disturb", 1, true);

  int select_platform = 0;                 // Quad F330
//  int select_platform = 1;                 // Talon tricopter
//  int select_platform = 2;                 // New White tricopter
//  int select_platform = 3;                 // Microscale tricopter

  switch (select_platform) {
  case 0:
    uav_struct.uav_type = 0;
    uav_struct.m = 1.3;
    uav_struct.g = 9.81;
    uav_struct.K = {7.11e-06, 4.27e-7};
    uav_struct.len = {0.11864, 0.1190, 0, 0};
    uav_struct.J = {0.0149, 0.021915, 0.034433, 0.0, 0.000274004};

    Uref << 0, 0.0, 0, uav_struct.m*uav_struct.g;
    W << 20, 21, 30, 1.3, 1.3, 1.5,
         20, 20, 80, 0.043;
    break;
  case 1:
    uav_struct.uav_type = 1;
    uav_struct.m = 1.412;
    uav_struct.g = 9.81;
    uav_struct.K = {6.85e-6, 3.43e-7};
    uav_struct.len = {0.3373, 0.2685, 0.1654, 0.2700};
    uav_struct.J = {0.0209, 0.021915, 0.034433, 0.00901952, 0.000274004};

    Uref << -0.0414*1.5, 0.0, 0, uav_struct.m*uav_struct.g;
    W << 18, 20, 30, 0.8, 0.8, 0.1,
         20, 23, 80, 0.02;
    break;
  case 2:
    uav_struct.uav_type = 1;
    uav_struct.m = 1.443;
    uav_struct.g = 9.81;
    uav_struct.K = {7.11e-06, 4.23e-7};
    uav_struct.len = {0.28479, 0.21248, 0.085, 0.21248};
    uav_struct.J = {0.014965, 0.021915, 0.034433, 0.00026952, 0.000274004};

    Uref << -0.0414*1.5, 0.0, 0, uav_struct.m*uav_struct.g;
    W << 18, 20, 30, 1.3, 1.3, 1.0,
         20, 23, 30, 0.03;
    break;
  case 3:
    uav_struct.uav_type = 1;
    uav_struct.m = 0.570;
    uav_struct.g = 9.81;
    uav_struct.K = {1.26561e-6, 0.015*1.26561e-6};
    uav_struct.len = {0.1062, 0.0898, 0.0519, 0.0898};             // Have to check these numbers
    uav_struct.J = {0.0017, 0.00200, 0.00305, 0.0, 2.9e-06};

    Uref << -0.0414*1.5, 0.0, 0, uav_struct.m*uav_struct.g;
    W << 30, 35, 30, 0.7, 0.6, 2.5,
         23, 24, 80, 2.2e-2;
    break;
  default:
    break;
  }

  NMPC_PC *nmpc_pc = new NMPC_PC(uav_struct.m, uav_struct.g,
                                 Uref, W);

  UAV_MODEL *uav_model = new UAV_MODEL(sampleTime/req_inner_loop_itr,
                                       uav_struct);

  ros::Rate rate(1/sampleTime);

  current_pos_att.resize(uav_model->publish_struct.states_position_vec.size() +
                         uav_model->publish_struct.states_attitude_vec.size());
  current_vel_rate.resize(uav_model->publish_struct.states_velocity_vec.size() +
                          uav_model->publish_struct.states_rate_vec.size());

//  for (int i=0; i<current_pos_att.size(); i++)
//  {
//    std::cout<<"uav_model->publish_struct.states_attitude_vec["<<i<<"] = "<<uav_model->publish_struct.states_attitude_vec[i]<<"\n";
//    std::cout<<"uav_model->publish_struct.states_rate_vec["<<i<<"] = "<<uav_model->publish_struct.states_rate_vec[i]<<"\n";
//  }


  ref_traj_type = 0;
  ref_trajectory << 0, 0, 0;
  ref_velocity << 0, 0, 0;

  bool control_stop = false;
  bool model_stop = false;

  bool use_learning = false;
  nmpc_weights_struct.onSwitch = false;
  nmpc_weights_struct.Wx.resize(NMPC_NX);
  nmpc_weights_struct.Wu.resize(NMPC_NU);
  for (int i=0; i<NMPC_NX; ++i)
    nmpc_weights_struct.Wx(i) = W(i);
  for (int i=0; i<NMPC_NU; ++i)
    nmpc_weights_struct.Wu(i) = W(i+NMPC_NX);

  for (int i=0; i<100; ++i)
  {
//    std::cout<<"outer_nmpc_cmd_Fz = "<<outer_nmpc_cmd_Fz(0)<<"\n";
    ros::spinOnce();
    rate.sleep();
  }

  while(ros::ok() && !control_stop && !model_stop)
  {
    if (control_stop)
    {
      delete nmpc_pc;
      NMPC_PC *nmpc_pc = new NMPC_PC(uav_struct.m, uav_struct.g,
                                     Uref, W);
      ROS_WARN_STREAM("New instance of NMPC_PC is created!");
      UAV_MODEL *uav_model = new UAV_MODEL(sampleTime/req_inner_loop_itr,
                                           uav_struct);
      ROS_WARN_STREAM("New instance of UAV_MODEL is created!");
    }

    t = ros::Time::now().toSec();

    if (!use_learning)
      nmpc_start_flag = 1;

    while(ros::ok() && !control_stop && !model_stop && nmpc_start_flag && trajectory_start_flag.data)
    {

      for (int i=0; i<current_pos_att.size(); ++i)
      {
        if (i<3)
        {
          current_pos_att[i] = uav_model->publish_struct.states_position_vec[i];
          current_vel_rate[i] = uav_model->publish_struct.states_velocity_vec[i];
        }
        else
        {
          current_pos_att[i] = uav_model->publish_struct.states_attitude_vec[i-3];
          current_vel_rate[i] = uav_model->publish_struct.states_rate_vec[i-3];
        }
//        std::cout<<"current_pos_att["<<i<<"] = "<<current_pos_att[i]<<"\n";
//        std::cout<<"current_vel_rate["<<i<<"] = "<<current_vel_rate[i]<<"\n";
      }

      if(!nmpc_pc->return_control_init_value())
          nmpc_pc->nmpc_init(current_pos_att, nmpc_pc->nmpc_struct);

      t_loop = ros::Time::now().toSec() - t;
      std::cout<<"loop time for safety check node: " << t_loop << " (sec)"<<"\n";

      nmpc_pc->nmpc_core(nmpc_pc->nmpc_struct, nmpc_pc->nmpc_cmd_struct,
                         ref_trajectory, ref_velocity, current_pos_att, current_vel_rate, nmpc_weights_struct);

      if(nmpc_pc->acado_feedbackStep_fb != 0)
          control_stop = true;

      if(std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
         std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
      {
        ROS_ERROR_STREAM("Controller ERROR at time = " << ros::Time::now().toSec() - t <<" (sec)" );
        ROS_ERROR_STREAM("nmpc_struct.u = "<<nmpc_pc->nmpc_struct.u[0]<<", "<<nmpc_pc->nmpc_struct.u[1]<<", "
                                           <<nmpc_pc->nmpc_struct.u[2]<<", "<<nmpc_pc->nmpc_struct.u[3]<<"\n");
        control_stop = true;
//                exit;
      }

      outer_nmpc_cmd = {nmpc_pc->nmpc_cmd_struct.roll_ang, nmpc_pc->nmpc_cmd_struct.pitch_ang,
                        nmpc_pc->nmpc_cmd_struct.yaw_ang, nmpc_pc->nmpc_cmd_struct.Fz};

//      std::cout<<"outer_nmpc_cmd = "<<outer_nmpc_cmd[0]<<", "
//                                    <<outer_nmpc_cmd[1]<<", "
//                                    <<outer_nmpc_cmd[3]<<", "
//                                    <<outer_nmpc_cmd[4]<<"\n";

      // Giving time to low level controller to
      // follow position controller reference
      for (int i=0; i<req_inner_loop_itr; ++i)
        uav_model->uav_model_core(outer_nmpc_cmd);

      for (int i=0; i<3; ++i)
      {
        if(std::isnan(uav_model->publish_struct.states_position_vec[i]) == true ||
           std::isnan(uav_model->publish_struct.states_velocity_vec[i]) == true ||
           std::isnan(uav_model->publish_struct.states_attitude_vec[i]) == true ||
           std::isnan(uav_model->publish_struct.states_rate_vec[i]) == true)
        {
            ROS_ERROR_STREAM("UAV Model ERROR at time = " << ros::Time::now().toSec() - t <<" (sec)" );
            model_stop = true;
//            exit;
        }
      }

      nmpc_pc->publish_rpyFz(nmpc_pc->nmpc_cmd_struct);
      uav_model->publish_states();

      ros::spinOnce();
      rate.sleep();
    }

    nmpc_pc->publish_rpyFz(nmpc_pc->nmpc_cmd_struct);
    uav_model->publish_states();

    ros::spinOnce();
    rate.sleep();

  }

  return 0;
}
