/*
 * Robot Teaching Library
 * Copyright (C) 2014-2016 ETRI, South Korea
 * Author: Hooman Lee, Taewoo Kim
 * email:  lhm85@snu.ac.kr; twkim0812@gmail.com
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

#ifndef __ROBOT_H__
#define __ROBOT_H__

#pragma once
//#include <Eigen/Dense>
//#include <iostream>
#include "Precompiled.h" 

#define DtoR	M_PI/180	// degree to radian
#define RtoD	180/M_PI	// radian to degree

#define AMIRO_L	0x01
#define AMIRO_R	0x02

#define STAUBLI	0x04


#define PAUL	TRUE
#define CRAIG	FALSE


using namespace Eigen;
using namespace std;

// Orientation Notation
enum typeRot {Exyz, Ezyz, Estaubli};

class T_Robot
{
public:
	T_Robot(void);
	~T_Robot(void);

private:	// ��� ���� ���� 
	Matrix4d *A;	// A01, A12, A23, ...
	Matrix4d *T;	// T01, T02, T03, ...

	unsigned int dof;
	unsigned int end;	// End-effector index
	VectorXd	 q;		
	MatrixXd	 Jacobian;
	BOOL		 kinType;		// Kinematics type PAUL:TRUE,  CRAIG:FALSE
	typeRot		 oriType;		// Orientation type, ex) Ezyz, Exyz...

	VectorXd a, alpha, d, theta;			// DH parameters
	double x_offset, y_offset, z_offset;	// offset parameters for drawing


	Matrix4d	preH;	// Base coordinate�� �����ؾ� �� �ʿ䰡 ���� �� ���� ����. �ʱ� ���� Identity
	Matrix4d	postH;	// End-effector or Tool��ǥ�� �߰��� ���� �� �ʿ䰡 ���� �� ���� ����. �ʱ� ���� Identity

	double		t_a, t_alpha, t_d, t_theta;	// Additional Tool Transform
	Matrix4d	T0E;			// Tool Matrix
	Matrix4d	T_Tooloffset;	// Tool of End-effector offset
	BOOL		bTool;			// Tool�� ��� ����
				


public:	// ��� �Լ� ����
	
	void		Initialization();

	// Calc: ���, ������Ʈ
	/* calculate FK with current q */
	void		CalcFK();
	/* calculate jacobian with current q */
	void		CalcJacobian();
	
	/* Damped Least Square Method */
	void		CalcIK(Matrix4d &goal_T, VectorXd &_dq, double _damp_param);
	void		CalcIK_custom(MatrixXd _Jacobian, VectorXd _dx, VectorXd &_dq, double _damp_param);
	void		CalcIK_JLA(Matrix4d &goal_T, VectorXd &_dq, double _damp_param, VectorXd _Midjoint, MatrixXd _weight);
	void		MakeRobotTransform(Vector3d Pos, Vector3d Ori, Matrix4d &rTransform, char* str);
	void		MakeRobotTransform(Vector3d Pos, Vector3d Ori, Matrix4d &rTransform);

	
	// Get: ���� ����(Calc�� ���� ���־�� ��)
	int			GetDOF();
	Matrix4d	GetA(int iq);
	Matrix4d	GetT(int iq);
	Matrix4d	*GetpT();
	MatrixXd	GetJacobian();
	MatrixXd	GetDH();
	double		GetQ(int index);	// for element
	VectorXd	GetQ();				// Overloading, for all
	double		GetOffset(int index);
	bool		isTool();
	Matrix4d	GetTool();
	Vector3d	GetPosition();
	Vector3d	GetOrientation();	// ���� ������ oriType���� ���� ���� ����
	VectorXd	GetPosNOri();
	Vector3d	GetOrientation(char* eType);
	Vector3d	GetOrientation_eXYZ(Matrix3d R);
	Vector3d	GetOrientation_eZYZ(Matrix3d R);
	Vector3d	GetEulerAngleRad(Matrix3d rotMat);
	Matrix4d	GetHTransform(double a, double alpha, double d, double theta);
	MatrixXd	GetPointsforDraw();

	//// ******************* �κ� �ʱ�ȭ �Լ� **************** ////
	// �ʼ�
	void		SetDH(VectorXd _a, VectorXd _alpha, VectorXd _d, VectorXd _theta, unsigned int _dof, BOOL kinType);	

	// �ɼ�. ���� �����ָ� �ش� �������� �⺻������ ����
	void		SetToolDH(double _a, double _alpha, double _d, double _theta, BOOL isTool);		// Tool ����
	void		SetPreH(double _a, double _alpha, double _d, double _theta);					// Base Frame ��ȯ ��ų�� ���
	void		SetPostH(double _a, double _alpha, double _d, double _theta);					// Tool Frame or End-effector ��ȯ �� ��� 
	void		SetPostH(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double a13,	// Overloading
						 double a20, double a21, double a22, double a23, double a30, double a31, double a32, double a33);
	void		SetOriType(char* str);	// ���� ǥ�� ��� ����. �ʱⰪ�� Exyz, ("Exyz", "Ezyz", "Estaubli")
	void		SetOffset(double _x_offset, double _y_offset, double _z_offset);	// Base Frame���� ���� �κ� ��ü ��ġ�� offset ��ų �� ���
	//// ******************* �κ� �ʱ�ȭ �Լ� **************** ////

	
	void		SetQ(int index, double rad);	// for element
	void		SetQ(VectorXd q_rad);			// Overloading, for all
	
	double		griptime;
	double		releasetime;

};

#endif

// �ٲ� �κ�!
// SetToolDH: �Ķ���� ���� �޶���
// CalcFK: ���� ����
// GetPosition : �Լ� �߰�
// GetOrientation : �Լ� �߰�
// SetPreH : preH matrix�� ��������
// oriType : ���� �κ��� orientation ǥ�� ����� ��Ÿ���� ���� ���� �߰�
// SetOriType : ó���� orientation type�� �������ִ� �Լ�
// GetOrientation() : parameter���� �ƴ� ������ �κ� Ŭ������ ������ oriType�� ���� ���� ����
// MakeRobotTransform(Vector3d Pos, Vector3d Ori, Matrix4d &rTransform) : ���� Ÿ���� �Է� ���� �ʰ� ������ ������ Ÿ�Կ� �°� ��ȯ
// SetPostH(double _a, double _alpha, double _d, double _theta) : End-effector ~ Tool������ ��ǥ�� ��ȯ�� ��
// SetPostH(Matrix4d _postH) : ���� ���� �����Ͽ� ������ ��..