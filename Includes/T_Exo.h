/*
 * Robot Teaching Library
 * Copyright (C) 2014 ETRI, South Korea
 * Author: Hooman Lee
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

#ifndef __T_EXO_H__
#define __T_EXO_H__

#pragma once

//#include <ctime>
//#include <vector>
//#include <process.h>
//
//#include <Dense>
//#include <Geometry>
//#include <Core>

#include <CommThread.h>

#define SWITCH_L_X			0
#define SWITCH_L_SQR		1
#define SWITCH_L_TRI		2
#define SWITCH_L_FRONT		3
#define SWITCH_R_X			4
#define SWITCH_R_SQR		5
#define SWITCH_R_TRI		6
#define SWITCH_R_FRONT		7

#define MCUDATALEN			200

using namespace Eigen;

class T_Exo 
{
public:
	bool				connect_stats;
	int					daytatype;
	int					connection_type; //1.0 = POSEONLY, 2.0 = ORIONLY, 3.0 = POS_ORI 
	int					connection_mode; //100 = studio, 200 = robot
	double				temp_joint[6];  
	double				temp_point[4];  // ���� ������� �ϳ� �÷���.. button ������.. Network_HM.cpp�� ��������!
	double				temp_ori[4];    // ���� ������� �ϳ� �÷���.. button ������.. Network_HM.cpp�� ��������!
	double				temp_pos_ori[7]; // ���� ������� �ϳ� �÷���.. button ������.. Network_HM.cpp�� ��������!
	int					button;

	//vector <double>		exohome;
	//vector <double>		exoinit_ori;
	//vector <double>		exoinit_rxryrz;
	Matrix4d			calib_robotHome;		//Ķ���극�̼��� ���� ���� (�� ���� ���� �����Ƿ�..)
	Matrix4d			calib_robotHome_L;		//Ķ���극�̼��� ���� ����
	Matrix4d			calib_robotHome_R;		//Ķ���극�̼��� ���� ����
	Matrix4d			calib_robotHome_elboL;	//Ķ���극�̼��� ���� ����
	Matrix4d			calib_robotHome_elboR;	//Ķ���극�̼��� ���� ����
	Matrix4d			calib_exoHome_L;		//Ķ���극�̼��� ���� ����
	Matrix4d			calib_exoHome_R;		//Ķ���극�̼��� ���� ����
	Matrix4d			calib_exoHome_elboL;	//Ķ���극�̼��� ���� ����
	Matrix4d			calib_exoHome_elboR;	//Ķ���극�̼��� ���� ����

	Matrix4d			exoHT_ee_R;		//���� ������ EE HTransform
	Matrix4d			exoHT_ee_L;		//���� ������ EE HTransform
	Vector3d			exo_pos_L;		//���� ������ ������
	Vector3d			exo_pos_R;		//���� ������ ������
	Vector3d			exo_pos_elboL;	//���� ELBO ������ ������
	Vector3d			exo_pos_elboR;	//���� ELBO ������ ������
	Matrix3d			exo_ori_L;		//���� ���������̼� ������
	Matrix3d			exo_ori_R;		//���� ���������̼� ������

	double				scale;			//�Է� ������ �����ϸ� �Ķ����

	CCommThread			_serial;	//�ø�����Ʈ Ŭ����
	int					commIndex;	//Ŀ�´����̼� �ε���
	BYTE				aByte;		//�����͸� ������ ����
	vector <float>		v;			//�����͸� ������ ����
	clock_t				buttonclk;	//��ư �ߺ��Է� ��������..
	clock_t				currclk;    //��ư �ߺ��Է� ��������..
	
	bool bQuit;
	int targc;
	char **targv;
	CRITICAL_SECTION cs;

	//���뺯��
	vector <double> shared_v;
	BYTE mcu_data[200]; // data ������ array
	bool data_analyzed;

public:
	T_Exo();
	~T_Exo();
	
	bool	Init_Exo(wchar_t* portname);
	bool	Discnt_Exo();
	void	DAQ_Exo(vector<float> &Exodata, Matrix4d &HTransform_EE_R, Matrix4d &HTransform_EE_L, Vector3d &pos_elbo_R, Vector3d &pos_elbo_L, int &_button);

	void Calib_exo(Vector3d robotHome, Vector3d exoHome);
	void Calib_exo(Vector3d robotHome, Vector3d exoHome, Vector3d robotHome2, Vector3d exoHome2);
	void RHmapper_exo(Matrix4d& rHTranform_target, Matrix3d rotation_mapper, Vector3d robotHome, Vector3d exoHome, Vector3d exoPos, Matrix3d exoOri, double scale);

	void Staubli_Exo_mapper(vector <double> &rPos_target, vector <double> &rOri_target, vector <double> robotHome, vector <double> exoHome, vector <float> exoData);
	void MakestaubliOri(vector <double> rExoOri, vector <double> rExoOriHome, vector <double> & rStaubliOri);
	//void GetExoOri(vector <double> rExoOri, vector <double>& rxryrz);
	

private:

};

#endif