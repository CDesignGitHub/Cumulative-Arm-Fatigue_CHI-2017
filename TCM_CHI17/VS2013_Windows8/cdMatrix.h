//----------------------------------------------------------------------------------------------------
// Implementation of TCM Cumulative Fatigue Model and Biomechanical Arm Analysis
// Author: Sujin Jang, Purdue University, May 2017.
// Please cite the following publication for public use:
// DOI: https://doi.org/10.1145/3025453.3025523
//
// Project Webpage: https://engineering.purdue.edu/cdesign/wp/modeling-cumulative-arm-fatigue/
//----------------------------------------------------------------------------------------------------

#pragma once  

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <math.h>

#ifndef CDMATRIX_EXPORTS  
#define CDMATRIX_API __declspec(dllexport)   
#else  
#define CDMATRIX_API __declspec(dllimport)   
#endif  

#ifndef CDMATRIX_H
#define CDMATRIX_H

using namespace std;
#define STD_USED

#ifndef STD_USED
using std::cin;
using std::cout;
using std::endl;
using std::vector;
#endif

/* Define angle input type */
//#define DEGREE
#define RADIAN

#ifdef DEGREE
// c = cosine(DEGREE)
#define c(x) cos(((float )x)*M_PI/(180.0))

// s = sine(DEGREE)
#define s(x) sin(((float )x)*M_PI/(180.0))

// a_c = arccosine(RADIAN)
#define a_c(x) ((acos((float )x))*(180.0/M_PI))

// a_s = arcsine(RADIAN)
#define a_s(x) ((asin((float )x))*(180.0/M_PI))
#endif

#ifdef RADIAN
// c = cosine(RADIAN)
#define c(x) cos((float )x)

// s = sine(RADIAN)
#define s(x) sin((float )x)

// a_c = arccosine(RADIAN)
#define a_c(x) ((acos((float )x))

// a_s = arcsine(RADIAN)
#define a_s(x) ((asin((float )x))
#endif

class cdMatrix
{
protected:
	float **Data; //Original T-matrix data
	int rows, columns; //Number of rows and columns in a T-matrix
	float angle; //Rotation angle
	float Unit_vector[3]; //Direction unit vector

public:
	//Matrix();
	CDMATRIX_API cdMatrix();
	CDMATRIX_API cdMatrix(int _rows, int _columns);
	CDMATRIX_API cdMatrix(const cdMatrix &_a);
	CDMATRIX_API ~cdMatrix();

	//Matrix equalize operator
	CDMATRIX_API cdMatrix operator =(const cdMatrix &_a);

	//Matrix plus operator
	CDMATRIX_API cdMatrix operator +(const cdMatrix &_a)const;

	//Matrix minus operator
	CDMATRIX_API cdMatrix operator -();
	CDMATRIX_API cdMatrix operator -(const cdMatrix &_a)const;

	//Matrix multiply operator
	CDMATRIX_API cdMatrix operator *(const cdMatrix &_a)const;
	CDMATRIX_API cdMatrix operator *(float mult)const;
	//friend Matrix operator *(float mult, const Matrix &_a)const;

	// (Matrix/numerical value)
	CDMATRIX_API cdMatrix operator /(float divide)const;

	//Cross product
	CDMATRIX_API cdMatrix operator |(const cdMatrix &_a)const;

	//Dot product
	CDMATRIX_API float operator ^(const cdMatrix &_a)const;

	//Input matrix data, only 4 columns matrix
	CDMATRIX_API void Input_data(float _Data[4][4]);

	//Input matrix data, only 3 columns matrix
	CDMATRIX_API void Input_data(float _Data[3][3]);

	//Input matrix data, only 1 columns matrix
	CDMATRIX_API void Input_data(float _Data[][1]);
	CDMATRIX_API void Input_data(float *_Data);
	CDMATRIX_API void Input_data(float x, float y, float z);
	CDMATRIX_API void Input_data(float x, float y, float z, float t); // homogeneous coordinates

	// Output matrix raw data
	CDMATRIX_API void Output_data(float **Output_data);
	CDMATRIX_API void Output_data(float Output_data[][1]);
	CDMATRIX_API void Output_data(float *Output_data);
	CDMATRIX_API float GetElement(int i, int j);

	// Set matrix size (could be vector also)
	CDMATRIX_API void SetMatDim(int _rows, int _cols);

	// Return number of rows
	//CDMATRIX_API int Get_rows_number();

	// Return number of columns
	//CDMATRIX_API int Get_columns_number();

	//Inverse matrix, and save data to 'Data' buffer
	CDMATRIX_API void Transpose_matrix();

	//-- Show matrix data
	CDMATRIX_API void Show_matrix();

	//-- Compute transformation matrices
	CDMATRIX_API void ComputeTransfromMat4x4(float th, float al, float d, float a); // return [4x4]
	CDMATRIX_API void ComputeRotMat3x3(float th, float al, float d, float a); // return  [3x3]
	CDMATRIX_API void ComputeRotMat3x3(float th, char dir); // return  [3x3]
	CDMATRIX_API void ComputeRotMat4x4(float th, char dir);
	CDMATRIX_API void ComputeTransMat4x4(float dx, float dy, float dz);
	CDMATRIX_API void ComputeTrace3x3(float x, float y, float z);
	CDMATRIX_API void ComputeIdentity3x3();
	CDMATRIX_API void Norm();
	CDMATRIX_API float Length();
};

#endif // CDMATRIX_H
