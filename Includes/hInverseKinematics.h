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

//////////////////////////////////// ���ⱸ�� ///////////////////////////////

using namespace std;

hMatrix A_hMatrix(double theta, double alpha, double a, double d);	//A_hMatrix 
hMatrix T_hMatrix(double *theta, double *alpha, double *a, double *d, int joint);		//T_hMatrix 
hMatrix Jacobian_hMatrix(double *theta, double *alpha, double *a, double *d);		//���ں�� ���
hMatrix cross(double *row, double *col);		//����� ����		
hMatrix Inverse(hMatrix Mat);		//�����
hMatrix Pseudo_Inverse(hMatrix Mat);		//�ǻ� �����
hMatrix Transpose(hMatrix Mat);		//��ġ ���
double dot(hMatrix A,hMatrix B);		//����� ����
hMatrix T_Rotation(hMatrix Mat);		//T ����� ȸ�� ��� �κ� 
hMatrix Inverse_Kinematics(hMatrix Initial_T, hMatrix Goal_T, double *Initial_t, double *DH_alpha, double *DH_a, double *DH_d, int joint);
int Inv_solution();
hMatrix Tool_Position(double roll, double pitch, double yaw);
	
//////////////////////////////////////////////////////////////////////////////////////////////

#endif