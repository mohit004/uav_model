/**
 * @file   NMPC_PC_velRef.cpp
 * @author Mohit Mehndiratta
 * @date   December 2018
 *
 * @copyright
 * Copyright (C) 2018.
 */

#include <NMPC_PC.h>

using namespace Eigen;
//using namespace ros;
// USING_NAMESPACE_ACADO;

NMPCworkspace nmpcWorkspace;
NMPCvariables nmpcVariables;

NMPC_PC::NMPC_PC(double m_in, double g_in,
                 Eigen::VectorXd Uref_in, Eigen::VectorXd W_in)
{
  is_control_init = false;

  m = m_in;
  g = g_in;

  min_Fz_scale = -0.2* m*g;
  max_Fz_scale = 2.4* m*g;

  W_Wn_factor = 1.5;

  U_ref = Uref_in;
  W = W_in;
  WN.resize(NMPC_NYN);
  WN << W_Wn_factor*W_in[0], W_Wn_factor*W_in[1], W_Wn_factor*W_in[2], W_Wn_factor*W_in[3], W_Wn_factor*W_in[4], W_Wn_factor*W_in[5];

  // --------------------
  // ACADO NMPC CONTROLLER
  // --------------------

  nmpc_struct.initializeSolver = boost::bind(nmpc_initializeSolver);
  nmpc_struct.preparationStep = boost::bind(nmpc_preparationStep);
  nmpc_struct.feedbackStep = boost::bind(nmpc_feedbackStep);
  nmpc_struct.getKKT = boost::bind(nmpc_getKKT);
  nmpc_struct.getObjective = boost::bind(nmpc_getObjective);
  nmpc_struct.printDifferentialVariables = boost::bind(nmpc_printDifferentialVariables);
  nmpc_struct.printControlVariables = boost::bind(nmpc_printControlVariables);

  nmpc_struct.acado_N = NMPC_N;
  nmpc_struct.acado_NX = NMPC_NX;
  nmpc_struct.acado_NY = NMPC_NY;
  nmpc_struct.acado_NYN = NMPC_NYN;
  nmpc_struct.acado_NU = NMPC_NU;
  nmpc_struct.acado_NOD = NMPC_NOD;

  nmpc_struct.x0 = &nmpcVariables.x0[0];
  nmpc_struct.x = &nmpcVariables.x[0];
  nmpc_struct.od = &nmpcVariables.od[0];
  nmpc_struct.y = &nmpcVariables.y[0];
  nmpc_struct.yN = &nmpcVariables.yN[0];
  nmpc_struct.u = &nmpcVariables.u[0];
  nmpc_struct.W = &nmpcVariables.W[0];
  nmpc_struct.WN = &nmpcVariables.WN[0];

  nmpc_cmd_struct.roll_ang = 0.0;
  nmpc_cmd_struct.pitch_ang = 0.0;
  nmpc_cmd_struct.yaw_ang = 0.0;
  nmpc_cmd_struct.Fz = m*g;
  nmpc_cmd_struct.Fz_scaled = ( (1 - 0)/(max_Fz_scale - min_Fz_scale) ) * (nmpc_cmd_struct.Fz - min_Fz_scale);
  nmpc_cmd_struct.kkt_tol = 0.0;
  nmpc_cmd_struct.obj_val = 0.0;

  std::cout<<"Constructor of the class NMPC_PC is created\n";

}

NMPC_PC::~NMPC_PC()
{
    std::cout<<"Destructor of the class NMPC_PC\n";
}

bool NMPC_PC::return_control_init_value()
{
    return NMPC_PC::is_control_init;
}

void NMPC_PC::nmpc_init(std::vector<double> posref, struct acado_struct& acadostruct)
{

  std::cout<<"outer_nmpc_initController - start\n";

  // Initialize the solver
  // ---------------------
  acadostruct.initializeSolver();

  // NMPC: initialize/set the states
  // ---------------------
  for (int i = 0; i < acadostruct.acado_NX * (acadostruct.acado_N + 1); ++i)
  {
      acadostruct.x[i] = 0.0;
  }

  // NMPC: initialize/set the controls
  // ---------------------
  for (int i = 0; i < acadostruct.acado_N; ++i)
  {
      for (int j = 0; j < acadostruct.acado_NU; ++j)
          acadostruct.u[(i * acadostruct.acado_NU) + j] = U_ref[j];
  }

  // NMPC: initialize/set the online data
  // ---------------------
  for (int i = 0; i < acadostruct.acado_NOD * (acadostruct.acado_N + 1); ++i)
  {
      acadostruct.od[i] = 0.0;
  }

  // NMPC: initialize the measurements/reference
  // ---------------------
  for (int i = 0; i < acadostruct.acado_NY * acadostruct.acado_N; ++i)
  {
      acadostruct.y[i] = 0.0;
  }
  for (int i = 0; i < acadostruct.acado_NYN; ++i)
  {
      acadostruct.yN[i] = 0.0;
  }

  // NMPC: initialize the current state feedback
  // ---------------------
#if ACADO_INITIAL_STATE_FIXED
  for (int i = 0; i < acadostruct.acado_NX; ++i)
  {
      if (i < 3)
      {
          acadostruct.x0[i] = posref[i];
      }
      else
          acadostruct.x0[i] = 0;
  }
#endif

  // NMPC: initialize the weight matrices
  // ---------------------
  for(int i = 0; i < acadostruct.acado_NY; ++i )
  {
      for(int j = 0; j < acadostruct.acado_NY; ++j )
      {
          if(i==j)
              acadostruct.W[(i * acadostruct.acado_NY) + j] = W[i];
          else
              acadostruct.W[(i * acadostruct.acado_NY) + j] = 0.0;
      }
  }
//  std::cout<<"W = "<<W<<"\n";
  for(int i = 0; i < acadostruct.acado_NYN; ++i )
  {
      for(int j = 0; j < acadostruct.acado_NYN; ++j )
      {
          if(i==j)
              acadostruct.WN[(i * acadostruct.acado_NYN) + j] = WN[i];
          else
              acadostruct.WN[(i * acadostruct.acado_NYN) + j] = 0.0;
      }
  }
//  std::cout<<"WN = "<<WN<<"\n";

  // Prepare first step
  // ------------------
  acadostruct.preparationStep();

  std::cout<<"Outer NMPC: initialized correctly\n";
  is_control_init = true;

}

void NMPC_PC::nmpc_core(struct acado_struct &acadostruct, struct command_struct &commandstruct,
                        Vector3d reftrajectory, Vector3d refvelocity, std::vector<double> currentposatt,
                        std::vector<double> currentvelrate, struct weights_struct_ nmpcweightsstruct)
{

  // set the current state feedback
  set_measurements(acadostruct, currentposatt, currentvelrate, nmpcweightsstruct);

  // set the reference path
  set_reftrajectory(acadostruct, reftrajectory, refvelocity);

  // NMPC: calc and apply control and prepare optimization for the next step
  // ----------------------------------------------------------------------

  // Execute Calculation (Optimization)
  acado_feedbackStep_fb = acadostruct.feedbackStep();

  if (acado_feedbackStep_fb != 0)
  {
    std::cout<<"ACADO ERROR: " << acado_feedbackStep_fb<<"\n";
    std::cout<<
        "acado outer nmpc controller states: x, y, z, u, v, w = " << acadostruct.x0[0] << ", "
        << acadostruct.x0[1] << ", " << acadostruct.x0[2] << ", " << acadostruct.x0[3] << ", "
        << acadostruct.x0[4] << ", " << acadostruct.x0[5] << "\n";
  }

  // Apply the new control immediately to the process, first NU components.
  commandstruct.roll_ang = acadostruct.u[0];
  commandstruct.pitch_ang = acadostruct.u[1];
  commandstruct.yaw_ang = acadostruct.u[2];
  commandstruct.Fz = acadostruct.u[3];
  commandstruct.Fz_scaled = ( (1 - 0)/(max_Fz_scale - min_Fz_scale) ) * (commandstruct.Fz - min_Fz_scale);
  commandstruct.kkt_tol = acadostruct.getKKT();
  commandstruct.obj_val = acadostruct.getObjective();

  // Settings for the next iteration
  acadostruct.preparationStep();

//    ROS_INFO_STREAM("Stoptime outer NMPC: " << ros::Time::now().toSec() - stopwatch.toSec() << " (sec)");

/* ------ NMPC_DEBUG ------*/
//  acadostruct.printDifferentialVariables();
//  acadostruct.printControlVariables();
}

void NMPC_PC::set_measurements(struct acado_struct &acadostruct, std::vector<double> currentposatt,
                               std::vector<double> currentvelrate, struct weights_struct_ nmpcweightsstruct)
{
  for (int i=0; i<3; ++i)
    acadostruct.x0[i] = currentposatt[i];

  tf::Quaternion q;
  if (currentposatt.size() == 6)
    q.setRPY(currentposatt[3], currentposatt[4], currentposatt[5]);
  else
  {
    q.setX(currentposatt[3]); q.setY(currentposatt[4]);
    q.setZ(currentposatt[5]); q.setW(currentposatt[6]);
//    tf::Quaternion q(currentposatt[3], currentposatt[4],
//                     currentposatt[5], currentposatt[6]);
  }
  tf::Matrix3x3 R_BI(q);
  R_BI = R_BI.transpose();
  for (int i=0; i<3; ++i)
  {
    acadostruct.x0[i + 3] = R_BI[i][0]*currentvelrate[0] +
                            R_BI[i][1]*currentvelrate[1] +
                            R_BI[i][2]*currentvelrate[2];
  }
  for (int i = 0; i < acadostruct.acado_N + 1; ++i)
  {
    acadostruct.od[(i * acadostruct.acado_NOD)]     = currentvelrate[3];
    acadostruct.od[(i * acadostruct.acado_NOD) + 1] = currentvelrate[4];
    acadostruct.od[(i * acadostruct.acado_NOD) + 2] = currentvelrate[5];
  }
  if (nmpcweightsstruct.onSwitch)
  {
    W << nmpcweightsstruct.Wx, nmpcweightsstruct.Wu;
    for(int i = 0; i < acadostruct.acado_NY; ++i )
    {
      for(int j = 0; j < acadostruct.acado_NY; ++j )
      {
        if(i==j)
          acadostruct.W[(i * acadostruct.acado_NY) + j] = W[i];
        else
          acadostruct.W[(i * acadostruct.acado_NY) + j] = 0.0;
      }
    }
    for(int i = 0; i < acadostruct.acado_NYN; ++i )
    {
      for(int j = 0; j < acadostruct.acado_NYN; ++j )
      {
        if(i==j)
          acadostruct.WN[(i * acadostruct.acado_NYN) + j] = W_Wn_factor*nmpcweightsstruct.Wx[i];
        else
          acadostruct.WN[(i * acadostruct.acado_NYN) + j] = 0.0;
      }
    }
  }
//  std::cout<<"W = "<<W<<"\n";
}

void NMPC_PC::set_reftrajectory(struct acado_struct &acadostruct, Vector3d reftrajectory,
                                                                  Vector3d refvelocity)
{
  acadostruct.yN[0] = reftrajectory(0);
  acadostruct.yN[1] = reftrajectory(1);
  acadostruct.yN[2] = reftrajectory(2);
  acadostruct.yN[3] = refvelocity(0);
  acadostruct.yN[4] = refvelocity(1);
  acadostruct.yN[5] = refvelocity(2);


  for(int i = 0; i < acadostruct.acado_N; ++i )
  {
    for(int j = 0; j < acadostruct.acado_NY; ++j )
    {
      if (j < acadostruct.acado_NX)
        acadostruct.y[(i * acadostruct.acado_NY) + j] = acadostruct.yN[j];
      else
        acadostruct.y[(i * acadostruct.acado_NY) + j] = U_ref[j - acadostruct.acado_NX];
    }
  }
}
