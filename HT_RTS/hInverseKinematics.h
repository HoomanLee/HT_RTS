#ifndef __HINVERSEKINEMATICS_H__
#define __HINVERSEKINEMATICS_H__

#include "hMatrix.h"
#include <math.h>
#include <Geometry>

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