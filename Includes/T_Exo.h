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
	double				temp_point[4];  // 원래 사이즈보다 하나 늘렸음.. button 때문에.. Network_HM.cpp도 수정했음!
	double				temp_ori[4];    // 원래 사이즈보다 하나 늘렸음.. button 때문에.. Network_HM.cpp도 수정했음!
	double				temp_pos_ori[7]; // 원래 사이즈보다 하나 늘렸음.. button 때문에.. Network_HM.cpp도 수정했음!
	int					button;

	//vector <double>		exohome;
	//vector <double>		exoinit_ori;
	//vector <double>		exoinit_rxryrz;
	Matrix4d			calib_robotHome;		//캘리브레이션을 위한 변수 (한 팔일 수도 있으므로..)
	Matrix4d			calib_robotHome_L;		//캘리브레이션을 위한 변수
	Matrix4d			calib_robotHome_R;		//캘리브레이션을 위한 변수
	Matrix4d			calib_robotHome_elboL;	//캘리브레이션을 위한 변수
	Matrix4d			calib_robotHome_elboR;	//캘리브레이션을 위한 변수
	Matrix4d			calib_exoHome_L;		//캘리브레이션을 위한 변수
	Matrix4d			calib_exoHome_R;		//캘리브레이션을 위한 변수
	Matrix4d			calib_exoHome_elboL;	//캘리브레이션을 위한 변수
	Matrix4d			calib_exoHome_elboR;	//캘리브레이션을 위한 변수

	Matrix4d			exoHT_ee_R;		//엑소 오른팔 EE HTransform
	Matrix4d			exoHT_ee_L;		//엑소 오른팔 EE HTransform
	Vector3d			exo_pos_L;		//엑소 포지션 데이터
	Vector3d			exo_pos_R;		//엑소 포지션 데이터
	Vector3d			exo_pos_elboL;	//엑소 ELBO 포지션 데이터
	Vector3d			exo_pos_elboR;	//엑소 ELBO 포지션 데이터
	Matrix3d			exo_ori_L;		//엑소 오리엔테이션 데이터
	Matrix3d			exo_ori_R;		//엑소 오리엔테이션 데이터

	double				scale;			//입력 데이터 스케일링 파라미터

	CCommThread			_serial;	//시리얼포트 클래스
	int					commIndex;	//커뮤니케이션 인덱스
	BYTE				aByte;		//데이터를 저장할 변수
	vector <float>		v;			//데이터를 저장할 변수
	clock_t				buttonclk;	//버튼 중복입력 방지위해..
	clock_t				currclk;    //버튼 중복입력 방지위해..
	
	bool bQuit;
	int targc;
	char **targv;
	CRITICAL_SECTION cs;

	//공용변수
	vector <double> shared_v;
	BYTE mcu_data[200]; // data 수집할 array
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