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

#ifndef __T_OMNI_H__
#define __T_OMNI_H__

#pragma once

//#include <ctime>
//#include <vector>
//#include <process.h>
//
//#include <Dense>
//#include <Geometry>
//#include <Core>

using namespace Eigen;


static const hduVector3Dd maxGimbalTorque(188.0,188.0,48.0); //mNm
static const hduVector3Dd nominalBaseTorque(200.0,350.0,200.0); //mNm
static bool TorqueMode = true;

#define BTN_1_SHORT		1
#define BTN_2_SHORT		2
#define BTN_1_LONG		3
#define BTN_2_LONG		4

/* Synchronization structure. */
typedef struct
{
    HDint buttonValues[1];
	HDdouble forceValues[3];
    HDdouble jointTorqueValues[3];   
    HDdouble gimbalTorqueValues[3];   
} DeviceStateStruct;


/* Button command¸¦ À§ÇÑ structure. */
typedef struct
{
    HLboolean Button1;
	HLboolean Button2;
	HDint ButtonState;
	float button1_prsd_time;
	float button2_prsd_time;
	float button3_prsd_time;
	clock_t prsdclk;
	clock_t endclk;
	int button1_cnt;
	int button2_cnt;
	int button3_cnt;

	clock_t prsdclk3;
	clock_t endclk3;
} ButtonStruct;

class T_Omni
{
public:
	bool			connect_stats;
	
	//VectorXd		_pos_haptics(3);
	//MatrixXd		_ori_haptics(4, 4);
	//VectorXd		_vel_haptics(3);
	//VectorXd		_force_haptics(3);
	//VectorXd		_filtered_FT(3);
	Vector3d		_pos_haptics;
	Matrix4d		_ori_haptics;
	Vector3d		_vel_haptics;
	Vector3d		_force_haptics;
	Vector3d		_filtered_FT;
	Vector3d		_enc_haptics;

	Transform<double, 3, 2> HT_target;

	HHD _hHD;
	int button;
	clock_t btn_preclk;
	clock_t btn_curclk;
	int btn_prsdcnt;
	HDSchedulerHandle hGravityWell;

	ButtonStruct BtnCmd;
	//static clock_t prsdclk;
	//static clock_t endclk;
	//static int button1_cnt;
	//static int button2_cnt;
	//static float button1_prsd_time;
	//static float button2_prsd_time;

	bool _haptics;
	bool _FT_filter;
	bool _haptics_getpos;
	bool _haptics_getforce;
	bool _haptics_getenc;

	HANDLE hMutex_pos;
	HANDLE hMutex_force;
	HANDLE hMutex_enc;
	HANDLE event_sequence;

	bool bQuit;
	int targc;
	char **targv;
	CRITICAL_SECTION cs;

public:
	T_Omni();
	~T_Omni();
	
	static unsigned int WINAPI OmniThreadFunction(void* arg);
	static unsigned int WINAPI NetThreadFunction(void* arg);
	
	int Haptic_initialize();
	void Haptic_finish();
	void setHapticPos(hduVector3Dd rPos_haptics, HDdouble rOri_haptics[], hduVector3Dd rVel_haptics);
	void getHapticPos(Vector3d& rPos, Matrix3d& rOri, Vector3d& rVel);
	void setForce_Haptic(hduVector3Dd& rForce_haptics);
	void getForce_Haptic(Vector3d rForce);
	void RHmapper(Transform<double, 3, 2>& rHTranform_target, Vector3d& rVel_target, Vector3d rHome, 
		Vector3d rPos, Matrix3d rOri, Vector3d rVel);
	void RHForcemapper(Vector3d& rForce_target, Vector3d rForce);
	void TeachingForcemapper(Vector3d& rForce_target, Vector3d rForce);
	void Low_Pass_Filter(MatrixXd& rDataset, int rDatacount, int rDatasize, VectorXd rData_pre, 
		VectorXd& rData);
	void setEnc_Haptic(HDlong rEnc_haptics[]);
	void getEnc_Haptic(Vector3d& rEnc);

	Matrix3d	Rotate_with_X(double rAngle);
	Matrix3d	Rotate_with_Y(double rAngle);
	Matrix3d	Rotate_with_Z(double rAngle);
	
	//void MakestaubliOri(Matrix3d& rHapticOri, Vector3d& rStaubliOri);
	
	/////////////////////////////////
	void PrintHelp();
	void PrintDeviceState(HDboolean bContinuous);

	void mainLoop();
	int initDemo(void);
	
	static HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData);
	static HDCallbackCode HDCALLBACK jointTorqueCallback(void *data);
	static HDCallbackCode HDCALLBACK hm_buttonCallback(void *userdata);

private:


public:
	void MakestaubliTransform(Vector3d rStaubliPos, Vector3d rStaubliOri, Matrix4d &rStaubliTransform);

};

#endif