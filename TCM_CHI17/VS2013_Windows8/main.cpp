//----------------------------------------------------------------------------------------------------
// Cumulative Fatigue Model and Biomechanical Arm Analysis.
// Author: Sujin Jang, Purdue University, May 2017.
// Please cite the following publication for public use:
// DOI: https://doi.org/10.1145/3025453.3025523
//
// Project Webpage: https://engineering.purdue.edu/cdesign/wp/modeling-cumulative-arm-fatigue/
//----------------------------------------------------------------------------------------------------

#include <iostream>  
#include "cdMatrix.h"
#include "cdUserData.h"
#include "cdUpperLimbDynamics.h"

void main()
{
	//----------------------------------------------------------------------------------------------------
	// Load joint tracking data.
	// ** Users do not necessarily need to use the function ("OpenDataFile") to load skeleton data.
	// This is only for showing an example case.
	// Users may directly use joint tracking outputs from motion capture sensors (e.g., MS Kinect).
	//----------------------------------------------------------------------------------------------------

	cdUserData *m_dataBase = new cdUserData;
	char fileName[100] = "KinectSkeletonData.dat";
	m_dataBase->OpenDataFile(fileName);		

	//----------------------------------------------------------------------------------------------------
	// Initialize a class variable ,"cdUpperLimbDynamics", for biomechanical arm analysis 
	//----------------------------------------------------------------------------------------------------

	// Example participant information
	int gender = 0; // 0: male, 1: female
	float totalWeight = 71.1; // kg,
	float handLength = 0.21; // meter 
	float lowerArmLength = 0.26; // meter
	float upperArmLength = 0.275; // meter
	float extraWeight = 0; // kg (bare hand: 0 kg, extra weight can be added in use of input device)
	float maxShoulderTorque = 57.2; // Newton*meter

	cdUpperLimbDynamics *m_UpperlimbModel = new cdUpperLimbDynamics(gender, totalWeight, handLength, lowerArmLength,
																	upperArmLength, extraWeight, maxShoulderTorque);
	m_UpperlimbModel->SetFatigueRatio(0.0146); // Set free parameter 'F' (fatigue rate)
	m_UpperlimbModel->SetRestRatio(0.0022); // Set free parameter 'R' (rest rate)
	m_UpperlimbModel->SetForceDevelopment(10); // Set force development rate LD
	m_UpperlimbModel->SetRelaxationRate(10); // Set relaxation rate LR

	int userId = 0; // default user id = 0
	int nFrames = m_dataBase->GetNpoints(userId); // number of frames in joint tracking
	float hand[3], elbow[3], shoulder[3], SpineShoulder[3]; // arm joint position variables (Right side)

	//----------------------------------------------------------------------------------------------------
	// Save Biomechanical analysis results to a file
	//----------------------------------------------------------------------------------------------------

	FILE *file;
	file = fopen("result.dat", "wb"); // Matlab script is provided (find "Matlab code") for plotting results.


	for (int frame = 0; frame < nFrames; frame++)
	{
		float* joint = m_dataBase->GetCoordinate(userId, frame);
		float time = m_dataBase->GetTimeStamp(userId, frame);

		//----------------------------------------------------------------------------------------------------
		// Kinect Joint Labeling Information in 'joint' array variable.
		// Please refer to Figure 1. in (https://msdn.microsoft.com/en-us/library/microsoft.kinect.jointtype.aspx)
		// for the location of body joints.
		// HandLeft 0:(0 ~ 2),
		// HandRight 1:(3 ~ 5),
		// ElbowLeft 2:(6 ~ 8),
		// ElbowRight 3:(9 ~ 11),
		// HipLeft 4:(12 ~ 14),
		// HipRight 5:(15 ~ 17),
		// ShoulderLeft 6:(18 ~ 20),
		// ShoulderRight 7:(21 ~ 23),
		// SpineMid 8:(24 ~ 26),
		// SpineShoulder 9:(27 ~ 29),
		// Head 10:(30 ~ 32),
		// Neck 11:(33 ~ 45).
		//----------------------------------------------------------------------------------------------------

		hand[0] = joint[1 * 3];
		hand[1] = joint[1 * 3 + 1];
		hand[2] = joint[1 * 3 + 2];

		elbow[0] = joint[3 * 3];
		elbow[1] = joint[3 * 3 + 1];
		elbow[2] = joint[3 * 3 + 2];

		shoulder[0] = joint[7 * 3];
		shoulder[1] = joint[7 * 3 + 1];
		shoulder[2] = joint[7 * 3 + 2];

		SpineShoulder[0] = joint[9 * 3];
		SpineShoulder[1] = joint[9 * 3 + 1];
		SpineShoulder[2] = joint[9 * 3 + 2];


		//----------------------------------------------------------------------------------------------------
		// Biomechanical analysis requires four joint positions of body including "hand", "elbow", "shoulder",
		// "SpineShoulder".
		//----------------------------------------------------------------------------------------------------

		m_UpperlimbModel->ComputeBiomechanicalArmModelVariables(hand, elbow, shoulder, SpineShoulder, time);

		if (frame > 3) // minimum three frames are needed to compute accelerations
		{
			//----------------------------------------------------------------------------------------------------
			// Get and save  biomechanical analysis results
			//----------------------------------------------------------------------------------------------------
			
			fwrite(&time, sizeof(float), 1, file); // time

			float torque = m_UpperlimbModel->GetShoulderTorque(); // shoulder torque
			fwrite(&torque, sizeof(float), 1, file);

			float F = m_UpperlimbModel->GetMuscleFatigueState(); // M_F
			fwrite(&F, sizeof(float), 1, file);

			float R = m_UpperlimbModel->GetMuscleRestState(); // M_R
			fwrite(&R, sizeof(float), 1, file);

			float A = m_UpperlimbModel->GetMuscleActiveState(); // M_A
			fwrite(&A, sizeof(float), 1, file);
		}
	}
	
	delete m_dataBase;
	delete m_UpperlimbModel;
}