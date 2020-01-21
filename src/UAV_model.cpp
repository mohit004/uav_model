/**
 * @file   UAV_model.cpp
 * @author Mohit Mehndiratta
 * @date   December 2018
 *
 * @copyright
 * Copyright (C) 2018.
 */

#include <UAV_model.h>

using namespace Eigen;

UAV_MODEL::UAV_MODEL(double samTime, struct uav_struct_ uavstruct)
{
  sampleTime = samTime;
  model_run_time = 0;
  rand_seed.seed(std::time(0));


  uav_struct = uavstruct;

  ATTITUDE_TC_DEFAULT = 0.2;

  thrust_moment_vec << uav_struct.m*uav_struct.g, 0.0, 0.0, 0.0;

  states_vec = {0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0};
  states_dot_vec = {0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0};

  rpyFz_cmd_struct.des_roll = 0.0;
  rpyFz_cmd_struct.des_pitch = 0.0;
  rpyFz_cmd_struct.des_yaw = 0.0;
  rpyFz_cmd_struct.des_Fz = uav_struct.m*uav_struct.g;

  switch (uav_struct.uav_type) {
  case 0:
    // Roll controller setup
    roll_struct.MC_TC = 0.2;
    roll_struct.MC_angle_P = 5.6;
    roll_struct.MC_angle_FF = 0.1;
    roll_struct.MC_angle_max = 35;                              // in degrees
    roll_struct.MC_rate_P = 0.15;
    roll_struct.MC_rate_I = 0.01;
    roll_struct.MC_rate_D = 0.001;
    roll_struct.MC_rate_FF = 0.5;
    roll_struct.MC_rate_max = 220;                              // in degrees/sec
    roll_struct.pre_error_rate = 0.0;
    roll_struct.intergral_rate = 0.0;

    // Pitch controller setup
    pitch_struct.MC_TC = 0.2;
    pitch_struct.MC_angle_P = 5.6;
    pitch_struct.MC_angle_FF = 0.1;
    pitch_struct.MC_angle_max = 35;                             // in degrees
    pitch_struct.MC_rate_P = 0.15;
    pitch_struct.MC_rate_I = 0.01;
    pitch_struct.MC_rate_D = 0.001;
    pitch_struct.MC_rate_FF = 0.5;
    pitch_struct.MC_rate_max = 220;                             // in degrees/sec
    pitch_struct.pre_error_rate = 0.0;
    pitch_struct.intergral_rate = 0.0;

    // Yaw controller setup
    yaw_struct.MC_TC = 0.2;
    yaw_struct.MC_angle_P = 2.8;
    yaw_struct.MC_angle_FF = 0.5;
    yaw_struct.MC_angle_max = 100;                              // in degrees
    yaw_struct.MC_rate_P = 0.2;
    yaw_struct.MC_rate_I = 0.1;
    yaw_struct.MC_rate_D = 0.0;
    yaw_struct.MC_rate_FF = 0;
    yaw_struct.MC_rate_P = 0.2;
    yaw_struct.MC_rate_I = 0.1;
    yaw_struct.MC_rate_D = 0.0;
    yaw_struct.MC_rate_FF = 0.3;
    yaw_struct.MC_rate_max = 200;                               // in degrees/sec
    yaw_struct.pre_error_rate = 0.0;
    yaw_struct.intergral_rate = 0.0;
    break;
  case 1:
    // Roll controller setup
    roll_struct.MC_TC = 0.2;
    roll_struct.MC_angle_P = 6.5;
    roll_struct.MC_angle_FF = 0.1;
    roll_struct.MC_angle_max = 35;                              // in degrees
    roll_struct.MC_rate_P = 0.25;
    roll_struct.MC_rate_I = 0.03;
    roll_struct.MC_rate_D = 0.001;
    roll_struct.MC_rate_FF = 0.5;
    roll_struct.MC_rate_max = 220;                              // in degrees/sec
    roll_struct.pre_error_rate = 0.0;
    roll_struct.intergral_rate = 0.0;

    // Pitch controller setup
    pitch_struct.MC_TC = 0.2;
    pitch_struct.MC_angle_P = 6.5;
    pitch_struct.MC_angle_FF = 0.1;
    pitch_struct.MC_angle_max = 35;                             // in degrees
    pitch_struct.MC_rate_P = 0.25;
    pitch_struct.MC_rate_I = 0.03;
    pitch_struct.MC_rate_D = 0.001;
    pitch_struct.MC_rate_FF = 0.5;
    pitch_struct.MC_rate_max = 220;                             // in degrees/sec
    pitch_struct.pre_error_rate = 0.0;
    pitch_struct.intergral_rate = 0.0;

    // Yaw controller setup
    yaw_struct.MC_TC = 0.2;
    yaw_struct.MC_angle_P = 1;
    yaw_struct.MC_angle_FF = 0.5;
    yaw_struct.MC_angle_max = 100;                              // in degrees
    yaw_struct.MC_rate_P = 0.2;
    yaw_struct.MC_rate_I = 0.1;
    yaw_struct.MC_rate_D = 0.0;
    yaw_struct.MC_rate_FF = 0;
    yaw_struct.MC_rate_P = 0.2;
    yaw_struct.MC_rate_I = 0.1;
    yaw_struct.MC_rate_D = 0.0;
    yaw_struct.MC_rate_FF = 0.3;
    yaw_struct.MC_rate_max = 200;                               // in degrees/sec
    yaw_struct.pre_error_rate = 0.0;
    yaw_struct.intergral_rate = 0.0;
    break;
  }

  control_alloc_struct.actuator_cmd  = {0.0, 0.0, 0.0, 0.0};
  control_alloc_struct.control_vec << 0.0, 0.0, 0.0, 0.0;
  control_alloc_struct.control_alloc_matrix_inverse.setZero();
  switch (uav_struct.uav_type) {
  case 0:
    control_alloc_struct.control_alloc_matrix_inverse <<
                         uav_struct.K[0],                    uav_struct.K[0],                    uav_struct.K[0],                    uav_struct.K[0],
      -uav_struct.K[0]*uav_struct.len[1],  uav_struct.K[0]*uav_struct.len[1],  uav_struct.K[0]*uav_struct.len[1], -uav_struct.K[0]*uav_struct.len[1],
      -uav_struct.K[0]*uav_struct.len[0],  uav_struct.K[0]*uav_struct.len[0], -uav_struct.K[0]*uav_struct.len[0],  uav_struct.K[0]*uav_struct.len[0],
                        -uav_struct.K[1],                   -uav_struct.K[1],                    uav_struct.K[1],                    uav_struct.K[1];
    break;
  case 1:
    control_alloc_struct.control_alloc_matrix_inverse <<
                         uav_struct.K[0],                    uav_struct.K[0],                   uav_struct.K[0],                               0.0,
      -uav_struct.K[0]*uav_struct.len[1],  uav_struct.K[0]*uav_struct.len[3],                               0.0,                               0.0,
      -uav_struct.K[0]*uav_struct.len[2], -uav_struct.K[0]*uav_struct.len[2], uav_struct.K[0]*uav_struct.len[0],                   uav_struct.K[1],
                         uav_struct.K[1],                   -uav_struct.K[1],                  -uav_struct.K[1], uav_struct.K[0]*uav_struct.len[0];
    break;
  }
  control_alloc_struct.control_alloc_matrix_inverse = control_alloc_struct.control_alloc_matrix_inverse.inverse();

  publish_struct.states_position_vec = {0.0, 0.0, 0.0};
  publish_struct.states_velocity_vec = {0.0, 0.0, 0.0};
  publish_struct.states_acceleration_vec = {0.0, 0.0, 0.0};
  publish_struct.states_attitude_vec = {0.0, 0.0, 0.0};
  publish_struct.states_rate_vec = {0.0, 0.0, 0.0};
  publish_struct.FxFyFz_disturb_vec = {0.0, 0.0, 0.0};

  std::cout<<"Constructor of the class UAV_MODEL is created\n";
}

UAV_MODEL::~UAV_MODEL()
{
    std::cout<<"Destructor of the class UAV_MODEL\n";
}

void UAV_MODEL::uav_model_core(std::vector<double> outernmpccmd)
{

  // set the reference path
  set_reftrajectory(outernmpccmd);

  // PID: calculate and apply control for each direction
  // ---------------------------------------------------

  // Low level control
  thrust_moment_vec(0) = rpyFz_cmd_struct.des_Fz;

  thrust_moment_vec(1) = low_level_calculate(rpyFz_cmd_struct.des_roll,
                                             states_vec[6],
                                             states_vec[9],
                                             roll_struct);
  thrust_moment_vec(2) = low_level_calculate(rpyFz_cmd_struct.des_pitch,
                                             states_vec[7],
                                             states_vec[10],
                                             pitch_struct);
  thrust_moment_vec(3) = low_level_calculate(rpyFz_cmd_struct.des_yaw,
                                             states_vec[8],
                                             states_vec[11],
                                             yaw_struct);

  // Control mixing
  control_alloc_struct.actuator_cmd = moment_to_actuator_cmd();
//  std::cout<<"actuator_cmd = "<<control_alloc_struct.actuator_cmd[0]<<", "
//                          <<control_alloc_struct.actuator_cmd[1]<<", "
//                          <<control_alloc_struct.actuator_cmd[2]<<", "
//                          <<control_alloc_struct.actuator_cmd[3]<<"\n";

  // Nonlinear model
  integrate_rk4(control_alloc_struct.actuator_cmd, states_vec);
//  integrate_euler(control_alloc_struct.actuator_cmd, states_vec);

  for (int i=0; i<3; ++i)
  {
    publish_struct.states_position_vec[i] = states_vec[i];
    publish_struct.states_velocity_vec[i] = states_vec[i+3];
    publish_struct.states_acceleration_vec[i] = states_dot_vec[i+3];
    publish_struct.states_attitude_vec[i] = states_vec[i+6];
    publish_struct.states_rate_vec[i] = states_vec[i+9];
  }

//  ROS_INFO_STREAM("Stoptime UAV model: " << ros::Time::now().toSec() - stopwatch.toSec() << " (sec)");
}

std::vector<double> UAV_MODEL::moment_to_actuator_cmd()
{
  control_alloc_struct.control_vec = control_alloc_struct.control_alloc_matrix_inverse * thrust_moment_vec;

  switch (uav_struct.uav_type) {
  case 0:
    control_alloc_struct.actuator_cmd = {sqrt(abs(control_alloc_struct.control_vec(0))),
                                         sqrt(abs(control_alloc_struct.control_vec(1))),
                                         sqrt(abs(control_alloc_struct.control_vec(2))),
                                         sqrt(abs(control_alloc_struct.control_vec(3)))};
    break;
  case 1:
    control_alloc_struct.actuator_cmd = {sqrt(abs(control_alloc_struct.control_vec(0))),
                                         sqrt(abs(control_alloc_struct.control_vec(1))),
                                         sqrt( sqrt( control_alloc_struct.control_vec(2)*control_alloc_struct.control_vec(2) +
                                            control_alloc_struct.control_vec(3)*control_alloc_struct.control_vec(3) ) ),
                                         atan2(control_alloc_struct.control_vec(3),control_alloc_struct.control_vec(2))};
    break;
  }
  return control_alloc_struct.actuator_cmd;
}

double UAV_MODEL::low_level_calculate(double des_angle, double curr_angle, double curr_rate, struct low_level_struct_ &lowlevelstruct)
{

    double dt = sampleTime;

    // -------------------------
    // P controller for attitude
    // -------------------------

    // Check for maximum desired angle
    if (des_angle > lowlevelstruct.MC_angle_max * M_PI/180)
        des_angle = lowlevelstruct.MC_angle_max * M_PI/180;

    // Calculate error
    double error_angle = des_angle - curr_angle;

    // Proportional term
    double p_angle_out = lowlevelstruct.MC_angle_P * (ATTITUDE_TC_DEFAULT/lowlevelstruct.MC_TC) * error_angle;

    // FeedForward term
    double FF_angle_out = lowlevelstruct.MC_angle_FF * des_angle;

    // Calculate total angle output
    double des_rate = p_angle_out + FF_angle_out;


    // -------------------------
    // PID controller for rates
    // -------------------------

    // Check for maximum desired rate
    if (des_rate > lowlevelstruct.MC_rate_max * M_PI/180)
        des_angle = lowlevelstruct.MC_rate_max * M_PI/180;

    // Calculate error
    double error_rate = des_rate - curr_rate;

    // Proportional term
    double p_rate_out = lowlevelstruct.MC_rate_P * (ATTITUDE_TC_DEFAULT/lowlevelstruct.MC_TC) * error_rate;

    // Integral term
    lowlevelstruct.intergral_rate += error_rate * dt;
    double i_rate_out = lowlevelstruct.MC_rate_I * lowlevelstruct.intergral_rate;

    // Derivative term
    double derivative = (error_rate - lowlevelstruct.pre_error_rate) / dt;
    double d_rate_out = lowlevelstruct.MC_rate_D * (ATTITUDE_TC_DEFAULT/lowlevelstruct.MC_TC) * derivative;

    // FeedForward term
    double FF_rate_out = lowlevelstruct.MC_rate_FF * des_rate;

    // Calculate total rate output
    double rate_output = p_rate_out + i_rate_out + d_rate_out + FF_rate_out;

    // Save error to previous error
    lowlevelstruct.pre_error_rate = error_rate;

    return rate_output;
}

void UAV_MODEL::set_reftrajectory(std::vector<double> outernmpccmd)
{
  rpyFz_cmd_struct.des_roll = outernmpccmd[0];
  rpyFz_cmd_struct.des_pitch = outernmpccmd[1];
  rpyFz_cmd_struct.des_yaw = outernmpccmd[2];
  rpyFz_cmd_struct.des_Fz = outernmpccmd[3];
}

void UAV_MODEL::integrate_euler(std::vector<double> actuatorcmd, std::vector<double> statesvec)
{
  std::vector<double> statesdotvec;
  statesdotvec = state_equations(actuatorcmd, statesvec);
  for (int i; i<statesvec.size(); ++i)
    states_vec[i] = statesvec[i] + sampleTime*statesdotvec[i];
}

void UAV_MODEL::integrate_rk4(std::vector<double> actuatorcmd, std::vector<double> statesvec)
{
  std::vector<double> K1, K2, K3, K4;
  std::vector<double> statesvec_inp;
  statesvec_inp = statesvec;
  K1 = state_equations(actuatorcmd, statesvec_inp);
  for (int i; i<statesvec.size(); ++i)
    statesvec_inp[i] = statesvec[i] + (sampleTime/2) * K1[i];
  K2 = state_equations(actuatorcmd, statesvec_inp);
  for (int i; i<statesvec.size(); ++i)
    statesvec_inp[i] = statesvec[i] + (sampleTime/2) * K2[i];
  K3 = state_equations(actuatorcmd, statesvec_inp);
  for (int i; i<statesvec.size(); ++i)
    statesvec_inp[i] = statesvec[i] + (sampleTime) * K3[i];
  K4 = state_equations(actuatorcmd, statesvec_inp);

  for (int i; i<statesvec.size(); ++i)
    states_vec[i] = statesvec[i] + sampleTime/6 * (K1[i] + 2*K2[i] + 2*K3[i] + K4[i]);

}

std::vector<double> UAV_MODEL::state_equations(std::vector<double> actuatorcmd,
                                               std::vector<double> statesvec)
{
  double omega_1_sqr, omega_2_sqr, omega_3_sqr, omega_4_sqr, mu;
  double x,y,z;
  double u,v,w;
  double phi,theta,psi;
  double p_rate,q_rate,r_rate;
  Vector3d velocity, rate;
  Vector3d position_dot, velocity_dot, attitude_dot, rate_dot;
  double Fx, Fy, Fz;
  double tau_prop_x, tau_prop_y, tau_prop_z;
  double tau_gyro_x, tau_gyro_y, tau_gyro_z;
  double tau_x, tau_y, tau_z;
  Matrix3d R_IB, T_IB;

  model_run_time += sampleTime;

  x = statesvec[0];
  y = statesvec[1];
  z = statesvec[2];

  u = statesvec[3];
  v = statesvec[4];
  w = statesvec[5];
  velocity << u, v, w;

  phi = statesvec[6];
  theta = statesvec[7];
  psi = statesvec[8];

  p_rate = statesvec[9];
  q_rate = statesvec[10];
  r_rate = statesvec[11];
  rate << p_rate, q_rate, r_rate;

  switch (uav_struct.uav_type) {
  case 0:
    omega_1_sqr = actuatorcmd[0]*actuatorcmd[0];
    omega_2_sqr = actuatorcmd[1]*actuatorcmd[1];
    omega_3_sqr = actuatorcmd[2]*actuatorcmd[2];
    omega_4_sqr = actuatorcmd[3]*actuatorcmd[3];

    Fx = 0;
    Fy = 0;
    Fz = uav_struct.K[0]*(omega_1_sqr + omega_2_sqr +
                          omega_3_sqr + omega_3_sqr);

    tau_x = uav_struct.K[0]*((omega_2_sqr + omega_3_sqr) -
                             (omega_1_sqr + omega_4_sqr))*uav_struct.len[1];
    tau_y = uav_struct.K[0]*((omega_2_sqr + omega_4_sqr) -
                             (omega_1_sqr + omega_3_sqr))*uav_struct.len[0];
    tau_z = uav_struct.K[1]*(omega_3_sqr + omega_4_sqr -
                             omega_1_sqr - omega_2_sqr);

    break;
  case 1:
    omega_1_sqr = actuatorcmd[0]*actuatorcmd[0];
    omega_2_sqr = actuatorcmd[1]*actuatorcmd[1];
    omega_3_sqr = actuatorcmd[2]*actuatorcmd[2];
    mu = actuatorcmd[3];

    Fx = 0;
    Fy = -uav_struct.K[0]*omega_3_sqr*sin(mu);
    Fz = uav_struct.K[0]*(omega_1_sqr + omega_2_sqr +
                          omega_3_sqr*cos(mu));

    tau_x = uav_struct.K[0]*(omega_2_sqr*uav_struct.len[3] -
                             omega_1_sqr*uav_struct.len[1]);
    tau_y = uav_struct.K[0]*(omega_3_sqr*cos(mu)*uav_struct.len[0] -
                             (omega_1_sqr + omega_2_sqr)*uav_struct.len[2]) +
            uav_struct.K[1]*omega_3_sqr*sin(mu);
    tau_z = uav_struct.K[1]*(omega_1_sqr - omega_2_sqr -
                             omega_3_sqr*cos(mu)) +
            uav_struct.K[0]*omega_3_sqr*sin(mu)*uav_struct.len[0];

//            tau_prop_x = uav_struct.K[0]*omega_2_sqr*uav_struct.len[3] -
//                         uav_struct.K[0]*omega_1_sqr*uav_struct.len[1];
//            tau_prop_y = uav_struct.K[0]*omega_3_sqr*cos(mu)*uav_struct.len[0] -
//                         uav_struct.K[0]*(omega_1_sqr + omega_2_sqr)*uav_struct.len[2] +
//                         uav_struct.K[1]*omega_3_sqr*sin(mu);
//            tau_prop_z = uav_struct.K[1]*omega_1_sqr -
//                         uav_struct.K[1]*omega_2_sqr -
//                         uav_struct.K[1]*omega_3_sqr*cos(mu) +
//                         uav_struct.K[0]*omega_3_sqr*sin(mu)*uav_struct.len[0];

//            tau_gyro_x = uav_struct.J[4]*(q_rate*(sqrt(omega_1_sqr)-sqrt(omega_2_sqr)) -
//                         sqrt(omega_3_sqr)*(cos(mu)*q_rate + sin(mu)*r_rate));
//            tau_gyro_y = uav_struct.J[4]*(p_rate*(sqrt(omega_2_sqr) - sqrt(omega_1_sqr)) + cos(mu)*sqrt(omega_3_sqr));
//            tau_gyro_z = uav_struct.J[4]*(-p_rate*sin(mu)*sqrt(omega_3_sqr));

//            tau_x = tau_prop_x + tau_gyro_x;
//            tau_y = tau_prop_y + tau_gyro_y;
//            tau_z = tau_prop_z + tau_gyro_z;
    break;
  }

  R_IB.setZero();
  R_IB << cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi),
          cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(psi)*cos(phi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi),
                  -sin(theta),                              sin(phi)*cos(theta),                              cos(phi)*cos(theta);

  T_IB.setZero();
  T_IB << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
          0,            cos(phi),           -sin(phi),
          0, sin(phi)/cos(theta), cos(phi)/cos(theta);

  position_dot = R_IB * velocity;
  attitude_dot = T_IB * rate;

  velocity_dot(0) = r_rate*v - q_rate*w + uav_struct.g*sin(theta) + (1/uav_struct.m)*Fx;
  velocity_dot(1) = p_rate*w - r_rate*u - uav_struct.g*sin(phi)*cos(theta) + (1/uav_struct.m)*Fy;
  velocity_dot(2) = q_rate*u - p_rate*v - uav_struct.g*cos(phi)*cos(theta) + (1/uav_struct.m)*Fz;

  rate_dot(0) = (1/(uav_struct.J[0]*uav_struct.J[2] - uav_struct.J[3]*uav_struct.J[3]))*
                                ((-p_rate*q_rate*uav_struct.J[3] + q_rate*r_rate*(uav_struct.J[1] - uav_struct.J[2]))*uav_struct.J[2] -
                                 (q_rate*r_rate*uav_struct.J[3] + p_rate*q_rate*(uav_struct.J[0] - uav_struct.J[1]))*uav_struct.J[3] +
                                 tau_x*uav_struct.J[2] - tau_z*uav_struct.J[3]);
  rate_dot(1) = ((uav_struct.J[2] - uav_struct.J[0])/uav_struct.J[1]) *p_rate*r_rate -
                                 (r_rate*r_rate - p_rate*p_rate)*(uav_struct.J[3]/uav_struct.J[1]) + tau_y*(1/uav_struct.J[1]);
  rate_dot(2) = (1/(uav_struct.J[0]*uav_struct.J[2] - uav_struct.J[3]*uav_struct.J[3]))*
                                ((q_rate*r_rate*uav_struct.J[3] + p_rate*q_rate*(uav_struct.J[0] - uav_struct.J[1]))*uav_struct.J[0] -
                                 (-p_rate*q_rate*uav_struct.J[3] + q_rate*r_rate*(uav_struct.J[1] - uav_struct.J[2]))*uav_struct.J[3] +
                                 tau_z*uav_struct.J[0] - tau_x*uav_struct.J[3]);

  std::normal_distribution<double> disturb(0, 0.05);

  publish_struct.FxFyFz_disturb_vec = {1+0.2*(sin(model_run_time/4)*sin(model_run_time/4) + sin(model_run_time/4)) + 1*disturb(rand_seed),
                                       0.0*sin(model_run_time),
                                       0.0*sin(model_run_time)};

  std::cout<<"FxFyFz_disturb_vec = "<<publish_struct.FxFyFz_disturb_vec[0]<<", "
                                    <<publish_struct.FxFyFz_disturb_vec[1]<<", "
                                    <<publish_struct.FxFyFz_disturb_vec[2]<<"\n";

  for (int i=0; i<3; ++i)
  {
    states_dot_vec[i] = position_dot(i);
    states_dot_vec[i+3] = velocity_dot(i) + publish_struct.FxFyFz_disturb_vec[i];
    states_dot_vec[i+6] = attitude_dot(i);
    states_dot_vec[i+9] = rate_dot(i);

  }

  return states_dot_vec;
}


