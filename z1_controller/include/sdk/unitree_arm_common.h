#pragma once

#include <array>
#include "UnitreeArmModule/math/mathTypes.h"

namespace UNITREE_ARM_SDK
{
	enum class ArmMode {
		Invalid,
		Passive,
		LowCmd,
		JointSpeedCtrl,
		JointPositionCtrl,
		Teach,
		TeachRepeat,
		Calibration,
		ClearError
	};

	constexpr int ErrorNum = 16;
	enum class Error{
		/**
		 * @brief True if the robot exceeeded joint velocity limits.
		 */
		joint_position_limits_violation,

		/**
		 * @brief True if the robot exceeded joint velocity limits.
		 */
		joint_velocity_violation,

		/**
		 * @brief True if a collision was detected.
		 */
		colision_detected,

		/**
		 * @brief True if the trajectory file is not avaliable when TeachRepeat
		 */
		repeat_file_invalid,
	};

	enum class MotorError {
		/**
		 * @brief True if the motor has lost connection.
		 */
		disconnection,

		/**
		 * @brief True if the motor phase current is too large.
		 */
		phase_current_large,

		/**
		 * @brief True if the motor has phase leakage.
		 */
		phase_leakage,

		/**
		 * @brief True if the motor temperature is larger than 80 degrees centigrade.
		 */
		over_temperature,

		/**
		 * @brief True if the motor winds overheat.
		 */
		wind_overheat,

		/**
		 * @brief True if the motor parameters jumped.
		 */
		parameters_jumped,
	};

  typedef struct
  {
		/**
		 * @brief Target gripper angle. [-1, 0], the negative direction indicates opening.
		 */
    double angle;                       

		/**
		 * @brief Closing speed in rad/s. Must be positive value.
		 */
    double speed;                       

		/**
		 * Grasping max tau in Nm. Must be positive value.
		 */
    double maxTau;       

		/**
		 * @brief Maximum tolerated deviation when the actual grasped angle
		 *  is smaller than the commanded grasp angle.
		 */
    double epsilon_inner;

		/**
		 * @brief Maximum tolerated deviation when the actual grasped angle
		 *  is larger than the commanded grasp angle.
		 */
    double epsilon_outer;
  }GripperCmd;

  typedef struct
  {
		/**
		 * @brief Measured gripper angle. Unit: rad
		 */
    double angle;

		/**
		 * @brief Measured gripper speed. Unit: rad/s
		 */
		double speed;

		/**
		 * @brief Measured gripper output torque. Unit: Nm
		 */
		double tau;

		/**
		 * @brief True if the grippper reaches target angle.
		 */
		bool reached_goal;

		/**
		 * @brief True if the gripper don't move.
		 */
		bool stalled;

		/**
		 * @brief True if the z1 robot has UnitreeGripper.
		 */
		bool exist;
  }GripperState;

  struct ArmCmd
  {
		/**
		 * @brief The z1 controller version.
		 */
		std::array<uint8_t, 3> version{};
		
		/**
		 * @brief Desired running state.
		 * @see ArmMode
		 */
		uint8_t mode{};

		/**
		 * @brief Configured mass of the end effector.
		 */
		double mass_ee{};

		/**
		 * @brief Configured center of mass of the end effector.
		 */
		std::array<double, 3> com_ee{};

		/**
		 * @brief Configured rotational inertia matrix of the end effector load with respect to center of mass.
		 */
		std::array<double, 9> inertia_ee{};

		/**
		 * @brief Configured the end effector in flange frame.
		 * Pose is represented as a 4x4 matrix in column-major format.
		 */
		std::array<double, 16> F_T_EE{};

		/**
		 * @brief Configured mass of the external load.
		 */
		double mass_load{};

		/**
		 * @brief Configured center of mass of the external load with respect to flange frame.
		 */
		std::array<double, 3> com_load{};

		/**
		 * @brief Configured inertia matrix of the external load with respect to center of mass.
		 */
		std::array<double, 9> inertia_load{};

		/**
		 * @brief Desired gripper command.
		 */
    GripperCmd gripperCmd{};

		/**
		 * @brief Joint position gains.
		 */
		std::array<double, 6> Kp{};

		/**
		 * @brief Joint velocity gains.
		 */
		std::array<double, 6> Kd{};		

		/**
		 * @brief Desired joint position. Unit: rad
		 */
		std::array<double, 6> q_d{};
		std::array<double, 6> q_d_1{};
		std::array<double, 6> q_d_2{};

		/**
		 * @brief Desired joint velocity. Unit: rad/s
		 */
		std::array<double, 6> dq_d{};
		std::array<double, 6> dq_d_1{};
		std::array<double, 6> dq_d_2{};

		/**
		 * @brief Desired joint feedforward torque. Unit: Nm
		 */
		std::array<double, 6> tau_d{};
		std::array<double, 6> tau_d_1{};
		std::array<double, 6> tau_d_2{};

		/**
		 * @brief Teach & TeachRepeat file name.
		 */
		std::array<char, 10> label{};
		uint32_t crc{};

		Vec6 getQ()  			{ return Eigen::Map<Vec6>(q_d.data()); }
		Vec6 getDq()  		{ return Eigen::Map<Vec6>(dq_d.data()); }
		Vec6 getTau() 		{ return Eigen::Map<Vec6>(tau_d.data()); }

		Mat4 getF_T_EE()  				{ return Eigen::Map<Mat4>(F_T_EE.data()); }
		Vec3 get_com_ee()  				{ return Eigen::Map<Vec3>(com_ee.data()); }
		Mat3 get_inertia_ee()  		{ return Eigen::Map<Mat3>(inertia_ee.data()); }
		Vec3 get_com_load()  			{ return Eigen::Map<Vec3>(com_load.data()); }
		Mat3 get_inertia_load()  	{ return Eigen::Map<Mat3>(inertia_load.data()); }

		void setQ(const Vec6& q, size_t index = 0) { 
			switch (index)
			{
			case 0: std::copy(q.data(), q.data()+q.size(), q_d.data()); break;
			case 1: std::copy(q.data(), q.data()+q.size(), q_d_1.data()); break;
			case 2: std::copy(q.data(), q.data()+q.size(), q_d_2.data()); break;
			}
		}
		void setDq(const Vec6& dq, size_t index = 0) {
			switch (index)
			{
				case 0: std::copy(dq.data(), dq.data()+dq.size(), dq_d.data()); break;
				case 1: std::copy(dq.data(), dq.data()+dq.size(), dq_d_1.data()); break;
				case 2: std::copy(dq.data(), dq.data()+dq.size(), dq_d_2.data()); break;
			}
		}
		void setTau(const Vec6& tau, size_t index = 0) {
			switch (index)
			{
				case 0: std::copy(tau.data(), tau.data()+tau.size(), tau_d.data()); break;
				case 1: std::copy(tau.data(), tau.data()+tau.size(), tau_d_1.data()); break;
				case 2: std::copy(tau.data(), tau.data()+tau.size(), tau_d_2.data()); break;
			}
		}

		/**
		 * @brief If the controller don't receive the next command, 
		 * then use the future command.
		 * Useful in case of occasional packet loss.
		 */
		void shift_command() {
			q_d     = q_d_1;
			q_d_1   = q_d_2;
			dq_d    = dq_d_1;
			dq_d_1  = dq_d_2;
			tau_d   = tau_d_1;
			tau_d_1 = tau_d_2;
		}

		void setF_T_EE(const Mat4& T) 			{ std::copy(T.data(), T.data()+16, F_T_EE.data()); }
		void set_com_ee(const Vec3& com)    { std::copy(com.data(), com.data()+com.size(), com_ee.data());}
		void set_inertia_ee(const Mat3& I) 	{ std::copy(I.data(), I.data()+9, inertia_ee.data()); }
		void set_inertia_load(const Mat3& I) 	{ std::copy(I.data(), I.data()+9, inertia_load.data()); }
	};

  struct ArmState
  {
		/**
		 * @brief The z1 controller version.
		 */
		std::array<uint8_t, 3> version{};

		/**
		 * @brief Desired running state.
		 * @see ArmMode
		 */
		uint8_t mode{};

		/**
		 * @brief Mesured gripper state.
		 */
    GripperState gripperState{};

		/**
		 * @brief Last desired gripper command.
		 */
		GripperCmd gripperCmd{};

		/**
		 * @brief Measured joint position. Unit: rad
		 */
		std::array<double, 6> q{};

		/**
		 * @brief Last desired joint position. Unit: rad
		 */
		std::array<double, 6> q_d{};

		/**
		 * @brief Mesaured joint speed. Unit: rad/s
		 */
		std::array<double, 6> dq{};

		/**
		 * @brief Last desired joint speed. Unit: rad/s
		 */
		std::array<double, 6> dq_d{};

		/**
		 * @brief Measured joint output torque. Unit: Nm
		 */
		std::array<double, 6> tau{};

		/**
		 * @brief Last desired joint feedforward torque. Unit: Nm
		 */
		std::array<double, 6> tau_d{};

		/**
		 * @brief Measured motor temperature.
		 */
		std::array<int8_t, 8> motor_temperature{};

		/**
		 * @brief Configured the end effector in flange frame
		 */
		std::array<double, 16> F_T_EE{};

		/**
		 * @brief Current robot error state.
		 */
		std::array<bool, ErrorNum> errors{};

		/**
		 * @brief Current motor error state.
		 */
		std::array<std::array<bool, 8>, 8> motor_errors{};

		uint32_t crc{};

		Vec6 getQ()  				{ return Eigen::Map<Vec6>(q.data()); }
		Vec6 getQ_d()  			{ return Eigen::Map<Vec6>(q_d.data()); }
		Vec6 getDq()  			{ return Eigen::Map<Vec6>(dq.data()); }
		Vec6 getDq_d()  		{ return Eigen::Map<Vec6>(dq_d.data()); }
		Vec6 getTau()  			{ return Eigen::Map<Vec6>(tau.data()); }
		Vec6 getTau_d()  		{ return Eigen::Map<Vec6>(tau_d.data()); }
		Mat4 getF_T_EE() 		{ return Eigen::Map<Mat4>(F_T_EE.data()); }
		bool hasError()  		{ return std::any_of(errors.cbegin(), errors.cend(), [](bool x){ return x; }); }
		bool hasMotorError() {
			return std::any_of(motor_errors.cbegin(), motor_errors.cend(), [](const std::array<bool, 8>& row){
				return std::any_of(row.cbegin(), row.cend(), [](bool x){ return x; });
			});
		}
  };

	constexpr size_t ARM_CMD_LENGTH = sizeof(ArmCmd);
	constexpr size_t ARM_STATE_LENGTH = sizeof(ArmState);
} // namespace UNITREE_ARM_SDK
