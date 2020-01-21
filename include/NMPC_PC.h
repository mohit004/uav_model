#pragma once

#include <math.h>
#include <Eigen/Dense>
#include <tf/tf.h>

#include "nmpc_common.h"
#include "nmpc_auxiliary_functions.h"

extern NMPCworkspace nmpcWorkspace;
extern NMPCvariables nmpcVariables;

struct weights_struct_
{
  bool onSwitch;
  Eigen::VectorXd Wx;
  Eigen::VectorXd Wu;
};

class NMPC_PC
{
	private:
		double m;
		double g;
		double min_Fz_scale;
		double max_Fz_scale;
		
		bool is_control_init;

		Eigen::VectorXd U_ref;
		Eigen::VectorXd W;
		double W_Wn_factor;
		Eigen::VectorXd WN;

		double x_ref_last;
		double y_ref_last;
		double z_ref_last;

	public:	
		int acado_feedbackStep_fb;

		struct acado_struct
		{
			boost::function<int(void)> initializeSolver;
			boost::function<int(void)> preparationStep;
			boost::function<int(void)> feedbackStep;
			boost::function<real_t(void)> getKKT;
			boost::function<real_t(void)> getObjective;
			boost::function<void(void)> printDifferentialVariables;
			boost::function<void(void)> printControlVariables;

			int acado_N;
			int acado_NX;
			int acado_NY;
			int acado_NYN;
			int acado_NU;
			int acado_NOD;

			real_t * x0;
			real_t * u;
			real_t * x;
			real_t * od;
			real_t * y;
			real_t * yN;
			real_t * W;
			real_t * WN;
		} nmpc_struct;

		struct command_struct
		{
			double roll_ang;
			double pitch_ang;
			double yaw_ang;
			double Fz;
			double Fz_scaled;	// between 0 and 1
			double kkt_tol;
			double obj_val;

		} nmpc_cmd_struct;

		NMPC_PC(double m_in, double g_in, Eigen::VectorXd Uref_in, Eigen::VectorXd W_in);
		~NMPC_PC();

		bool return_control_init_value();

		void nmpc_init(std::vector<double> posref, struct acado_struct& acadostruct);

		void nmpc_core(struct acado_struct &acadostruct, struct command_struct &commandstruct, Eigen::Vector3d reftrajectory, std::vector<double> currentposatt, std::vector<double> currentvelrate, struct weights_struct_ nmpcweightsstruct);

		void nmpc_core(struct acado_struct &acadostruct, struct command_struct &commandstruct, Eigen::Vector3d reftrajectory, Eigen::Vector3d refvelocity, std::vector<double> currentposatt, std::vector<double> currentvelrate, struct weights_struct_ nmpcweightsstruct);

        void publish_rpyFz(struct command_struct &commandstruct);

	protected:

		void set_measurements(struct acado_struct &acadostruct,  std::vector<double> currentposatt, std::vector<double> currentvelrate, struct weights_struct_ nmpcweightsstruct);

		void set_reftrajectory(struct acado_struct &acadostruct, Eigen::Vector3d reftrajectory);
		void set_reftrajectory(struct acado_struct &acadostruct, Eigen::Vector3d reftrajectory, Eigen::Vector3d refvelocity);

};
