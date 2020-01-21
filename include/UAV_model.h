#pragma once

#include <math.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <random>

using namespace Eigen;

struct uav_struct_
{
	int uav_type;
	double m;
	double g;
	
	std::vector<double> K;
	std::vector<double> len;
	std::vector<double> J;
};

class UAV_MODEL
{
	private:
		double sampleTime;
		double model_run_time;
		std::default_random_engine rand_seed;

		uav_struct_ uav_struct;
		
//		bool is_model_init;

		double ATTITUDE_TC_DEFAULT;
		
		Vector4d thrust_moment_vec;

		std::vector<double> states_vec;
		std::vector<double> states_dot_vec;


	public:

		struct outercontrol_cmd_struct
		{
			double des_roll;
			double des_pitch;
			double des_yaw;
			double des_Fz;
		} rpyFz_cmd_struct;

		struct low_level_struct_
		{
			double MC_TC;
			double MC_angle_P;
			double MC_angle_FF;
			double MC_angle_max;
			double MC_rate_P;
			double MC_rate_I;
			double MC_rate_D;
			double MC_rate_FF;
			double MC_rate_max;
			
			double pre_error_rate;
			double intergral_rate;
		} roll_struct, pitch_struct, yaw_struct;
	
		struct control_alloc_struct_
		{
			std::vector<double> actuator_cmd;
			Vector4d control_vec;
    			Matrix4d control_alloc_matrix_inverse;
		} control_alloc_struct;

		struct publish_struct_
		{
			std::vector<double> states_position_vec;
			std::vector<double> states_velocity_vec;
			std::vector<double> states_acceleration_vec;
			std::vector<double> states_attitude_vec;
			std::vector<double> states_rate_vec;
			
			std::vector<double> FxFyFz_disturb_vec; 
		} publish_struct;
		
		UAV_MODEL(double samTime, struct uav_struct_ uavstruct);
		~UAV_MODEL();

		void uav_model_core(std::vector<double> outernmpccmd);

		void publish_states();

	protected:
		
		void set_reftrajectory(std::vector<double> outernmpccmd);
		double low_level_calculate(double des_angle, double curr_angle, double curr_rate, struct low_level_struct_ &lowlevelstruct);

		std::vector<double> moment_to_actuator_cmd();

		void integrate_rk4(std::vector<double> actuatorcmd, std::vector<double> statesvec);
		void integrate_euler(std::vector<double> actuatorcmd, std::vector<double> statesvec);

    		std::vector<double> state_equations(std::vector<double> actuatorcmd, std::vector<double> statesvec);
};
