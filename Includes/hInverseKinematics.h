/*
 * Robot Teaching Library
 * Copyright (C) 2014-2016 ETRI, South Korea
 * Author: Hooman Lee, Taewoo Kim
 * email:  lhm85@snu.ac.kr
 * website: https://sites.google.com/site/hoomanleerobot/
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __HINVERSEKINEMATICS_H__
#define __HINVERSEKINEMATICS_H__

#include "hMatrix.h"
//#include <math.h>
//#include <Geometry>

//////////////////////////////////// 역기구학 ///////////////////////////////

using namespace std;

hMatrix A_hMatrix(double theta, double alpha, double a, double d);	//A_hMatrix 
hMatrix T_hMatrix(double *theta, double *alpha, double *a, double *d, int joint);		//T_hMatrix 
hMatrix Jacobian_hMatrix(double *theta, double *alpha, double *a, double *d);		//자코비안 행렬
hMatrix cross(double *row, double *col);		//행렬의 외적		
hMatrix Inverse(hMatrix Mat);		//역행렬
hMatrix Pseudo_Inverse(hMatrix Mat);		//의사 역행렬
hMatrix Transpose(hMatrix Mat);		//전치 행렬
double dot(hMatrix A,hMatrix B);		//행렬의 내적
hMatrix T_Rotation(hMatrix Mat);		//T 행렬의 회전 행렬 부분 
hMatrix Inverse_Kinematics(hMatrix Initial_T, hMatrix Goal_T, double *Initial_t, double *DH_alpha, double *DH_a, double *DH_d, int joint);
int Inv_solution();
hMatrix Tool_Position(double roll, double pitch, double yaw);
	
//////////////////////////////////////////////////////////////////////////////////////////////

#endif