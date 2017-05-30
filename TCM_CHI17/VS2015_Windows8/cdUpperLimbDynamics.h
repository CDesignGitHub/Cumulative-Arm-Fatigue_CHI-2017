//----------------------------------------------------------------------------------------------------
// Implementation of TCM Cumulative Fatigue Model and Biomechanical Arm Analysis
// Author: Sujin Jang, Purdue University, May 2017.
// Please cite the following publication for public use:
// DOI: https://doi.org/10.1145/3025453.3025523
//
// Project Webpage: https://engineering.purdue.edu/cdesign/wp/modeling-cumulative-arm-fatigue/
//----------------------------------------------------------------------------------------------------

#ifndef CDULDYNAMICS_EXPORTS  
#define CDULDYNAMICS_API __declspec(dllexport)   
#else  
#define CDULDYNAMICS_API __declspec(dllimport)   
#endif  

#ifndef CDFORWARDKINEMATICS_H
#define CDFORWARDKINEMATICS_H

#include <stdio.h>
#include "cdMatrix.h"

using namespace std;

class cdUpperLimbDynamics
{
private:
	int m_nJoints; // # of joints
	float *m_joints; // coordinates of "m_nJoints" number of joints
	float *m_prev_joints; // coordinates of previous joint
	float *m_q; // current joint angles
	float *m_pre_q; // previous joint angles
	float *m_pre_pre_q; // previous joint angles
	float *m_q_dot; // current joint angular speed
	float *m_pre_q_dot; // previous joint angular speed
	float *m_pre_pre_q_dot; // previous joint angular speed
	float *m_q_dot_dot; // current joint angular ||acceleration||

	//--- Average filter variables
	int m_filterCountIdx;
	float **m_buffer_q;
	float **m_buffer_q_dot;
	float **m_buffer_q_dot_dot;
	float *m_sum_q;
	float *m_sum_q_dot;
	float *m_sum_q_dot_dot;
	float *m_sum_torque;
	float *m_filtered_q;
	float *m_filtered_q_dot;
	float *m_filtered_q_dot_dot;


	//=====================================================================================
	// Upper limb 4-DOF model parameters
	// In default every vector is expressed with intertial coord. system (F) unless indicated.
	// e.g., m_r73 (w.r.t F-coord) --> m_r73_0 (w.r.t 0-coord).
	// 
	// 'm_R' indicates a rotation matrix
	// e.g., m_R_23: the rotation matrix between 2th and 3th link.
	//=====================================================================================

	float m_phi;
	cdMatrix m_R_F0, m_R_0F, m_R_F0_1, m_R_F0_2;
	cdMatrix m_R_01 , m_R_10 , m_R_12 , m_R_21,  m_R_23 , m_R_32 , m_R_34 , m_R_43 ;
	cdMatrix m_R_45, m_R_54;
	cdMatrix m_R_F1 , m_R_1F ;
	cdMatrix m_R_F2 , m_R_2F ;
	cdMatrix m_R_F3 , m_R_3F ;
	cdMatrix m_R_F4 , m_R_4F ;
	cdMatrix m_r73 , m_r73_0 , m_r37 ;
	cdMatrix m_r31 , m_r31_2 ;
	cdMatrix m_z0;
	cdMatrix m_x, m_y, m_z;
	cdMatrix m_z3_F, m_x3_F, m_y2_F;
	cdMatrix m_z_th3;
	cdMatrix m_z0_1;
	cdMatrix m_z1_2;
	cdMatrix m_z2_3;
	cdMatrix m_z3_4;
	cdMatrix m_r62 , m_r62_0 , m_r26 ;
	cdMatrix m_r20 , m_r20_2 ;

	//=====================================================================================
	//-- Inverse dynamics parameters
	//=====================================================================================

	cdMatrix m_w0, m_w1, m_w2, m_w3, m_w4; // angular velocity (body fixed frame)
	cdMatrix m_w0_dot, m_w1_dot, m_w2_dot, m_w3_dot, m_w4_dot; // angular acc (body fixed frame)
	cdMatrix m_ve0_dot, m_ve1_dot, m_ve2_dot, m_ve3_dot, m_ve4_dot; // linear acc. (body fixed frame)
	cdMatrix m_f1, m_f2, m_f3, m_f4, m_f5; // force from previous joint (body fixed frame)
	cdMatrix m_n1, m_n2, m_n3, m_n4, m_n5; // torque from previous joint (body fixed frame)
	cdMatrix m_a1, m_a2, m_a3, m_a4; // linear acc of COM
	cdMatrix m_s1, m_s2, m_s3, m_s4; // vector linking origin of i-th and com of i-th frame
	cdMatrix m_p1, m_p2, m_p3, m_p4; // vector linking origin of i-th and (i-1)th frame
	cdMatrix m_th_dot; // angular velocity in body fixed frame
	cdMatrix m_th_ddot; // angular acceleration in body fixed frame
	float *m_torques; // joint torques [T1, T2, T3, T4]
	float m_mass1, m_mass2, m_mass3, m_mass4; // link masses [m1, m2, m3, m4]

	//=====================================================================================
	//-- Time variables
	//=====================================================================================

	float m_currentTime; // Second
	float m_prevTime;
	float m_dt; // time step
	float m_pre_dt; // time step
	float m_elapsedTime; //elapsed time of interaction
	bool m_isAccelerationComputed; // flag indicating passing the first three frame
	int m_windowSize; // moving average windows size (default == 11)
	int m_countFrame; // count frames passed

	//=====================================================================================
	//-- Body segment parameters (BSP)
	//=====================================================================================

	float m_L_UA, m_L_LA, m_L_Hand;   // Length of arm segment (m)
	float m_comRatio_UA, m_comRatio_LA, m_comRatio_Hand; // center of mass in % of limb length from proximal joint
	float com_UA, com_LA, com_Hand, com_LA_Hand; // center of mass position
	cdMatrix m_pre_com_Arm; // center of mass w.r.t intertial reference frame
	cdMatrix m_com_UA; // center of mass w.r.t intertial reference frame
	cdMatrix m_com_LA; // center of mass w.r.t intertial reference frame
	cdMatrix m_ve_com; // velocity of center of mass
	cdMatrix m_pre_ve_com; // velocity of center of mass
	cdMatrix m_acc_com; // acceleration of center of mass
	cdMatrix m_alpha_com; // angular acceleration of center of mass
	cdMatrix m_gravity; // gravity = [9.8, 0, 0] in 0-coord.

	
	cdMatrix m_I_UA, m_I_LA, m_I_0, m_I_Arm; // Inertia tensor matrix components Tr[3x3]
	float m_Ixx_UA, m_Iyy_UA, m_Izz_UA; // kg*(m^2)
	float m_Ixx_LA, m_Iyy_LA, m_Izz_LA; // kg*(m^2)
	float m_Ixx_Hand, m_Iyy_Hand, m_Izz_Hand; // kg*(m^2)
	float m_mass_UA, m_mass_LA, m_mass_hand, m_mass_Arm; // arm segment masses

	//=====================================================================================
	// Dynamic Fatigue Model Parameters
	//=====================================================================================

	float m_maxShoulderTorque; // maximum shoulder torque
	float m_MF0; // initial muscle in fatigue (0%)
	float m_MR0; // initial muscle in rest (100%)
	float m_MA0; // initial muscle in active (0%)
	float m_F; // fatigue-active ratio
	float m_R; // fatigue-rest ratio
	float m_LD; // force development rate
	float m_LR; // relaxation factor

	float m_MF; // muscle unit in fatigue
	float m_MR; // muscle unit in rest
	float m_MA; // muscle unit in active

public:
	CDULDYNAMICS_API cdUpperLimbDynamics(int gender, float totalWeight,
		float HandLength, float LowerArmLength, float UpperArmLength, float ExtraWeight, float MaxShoulderTorque);
	CDULDYNAMICS_API ~cdUpperLimbDynamics();

	//=====================================================================================
	// Load & Give Joint Coordinates Info.
	//=====================================================================================

	CDULDYNAMICS_API void LoadJointCoordinates(float *joints/*joint tracking*/);
	CDULDYNAMICS_API void SetCurrentTime(float t);
	CDULDYNAMICS_API void ResetFKModel(); //reset forward kinematics model for a new data
	CDULDYNAMICS_API void SetNJoints(int nJoints);
	CDULDYNAMICS_API float GetTime();
	CDULDYNAMICS_API void GetJointCoordinates(int jntIdx, float* des);
	CDULDYNAMICS_API void GetUnitDirFromJointIdx(int jntIdxStart, int jntIdxEnd, float *des);
	CDULDYNAMICS_API void GetDirFromJointIdx(float *jntStart, float *jntEnd, float* des);
	CDULDYNAMICS_API float GetJointAngle(int idx);
	CDULDYNAMICS_API float GetAngularVel(int idx);
	CDULDYNAMICS_API float GetAngularAcc(int idx);
	CDULDYNAMICS_API float GetJointTorque(int idx);
	CDULDYNAMICS_API float GetFilteredJointAngle(int idx);
	CDULDYNAMICS_API float GetFilteredAngularVel(int idx);
	CDULDYNAMICS_API float GetFilteredAngularAcc(int idx);
	CDULDYNAMICS_API float GetShoulderTorque();

	//=====================================================================================
	// Compute joint angle (q), angular velocity (q_dot), angular acceleration (q_dot_dot)
	//=====================================================================================
	
	CDULDYNAMICS_API void ComputeJointAngleVariables();
	CDULDYNAMICS_API void ComputeJointAngleVariables_MovingAvg();

	//=====================================================================================
	// Forward velocity analysis
	//=====================================================================================

	CDULDYNAMICS_API void ForwardVelocityAnalysis();
	CDULDYNAMICS_API void ForwardVelocityAnalysis(/*INPUT*/cdMatrix pre_w, cdMatrix pre_w_dot, cdMatrix pre_v_dot,
									cdMatrix p, cdMatrix com, cdMatrix R, float q_dot, float q_ddot,
									/*OUTPUT*/cdMatrix &w, cdMatrix &w_dot, cdMatrix &v_dot, cdMatrix &a);

	//=====================================================================================
	// Neuton-Euler Dynamics
	//=====================================================================================

	CDULDYNAMICS_API void NEInverseDynamics();
	CDULDYNAMICS_API void NEInverseDynamics(cdMatrix w, cdMatrix w_dot, cdMatrix a, cdMatrix p, cdMatrix com,
							cdMatrix R_i_ii, cdMatrix R_ii_i, cdMatrix I,
							cdMatrix pre_f, cdMatrix pre_n, float mass, /*INPUT*/
							cdMatrix &f, cdMatrix &n /*OUTPU*/);

	//=====================================================================================
	// Dynamic Fatigue Model
	// Implementation of Three Compartment Muscle (TCM) Fatigue Model [Xia and Frey-Law 2008]
	//=====================================================================================

	CDULDYNAMICS_API void DynamicFatigueModel();
	CDULDYNAMICS_API void DynamicFatigueModel(float TL/*target torque*/, float MF0/*fatigue MU at 0*/,
								float MR0/*rest MU at 0*/, float MA0/*rest MU at 0*/,
								float F/*fatigue-active ratio*/, float R/*fatigue-rest ratio*/,
								float LD/*control 1*/, float LR/*control 2*/, float dt,
								float *des);
	CDULDYNAMICS_API float ActivationDrive(float MA, float MR, float TL, float LD, float LR);
	CDULDYNAMICS_API float MF_Dot(float F, float R, float MF, float MA);
	CDULDYNAMICS_API float MA_Dot(float F, float MA, float C);
	CDULDYNAMICS_API float MR_Dot(float R, float MF, float C);
	CDULDYNAMICS_API void SetFatigueRatio(float F);
	CDULDYNAMICS_API void SetRestRatio(float R);
	CDULDYNAMICS_API void SetForceDevelopment(float LD);
	CDULDYNAMICS_API void SetRelaxationRate(float LR);
	CDULDYNAMICS_API float GetMuscleFatigueState(){return m_MF;}
	CDULDYNAMICS_API float GetMuscleRestState(){return m_MR;}
	CDULDYNAMICS_API float GetMuscleActiveState(){return m_MA;}

	//=====================================================================================
	// Set Coordinate Systems (compute orthogonal vectors)
	// Model specific configurations
	//=====================================================================================

	CDULDYNAMICS_API void ComputeBiomechanicalArmModelVariables(float hand[3], float elbow[3], float shoulder[3], float shoulderCenter[3], float time);

};
#endif // CDFORWARDKINEMATICS_H
