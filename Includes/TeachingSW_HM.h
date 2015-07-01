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

#ifndef __TEACHINGSW_HM_H__
#define __TEACHINGSW_HM_H__

//#include <iostream>
//#include <conio.h>
//#include <math.h>
//#include <vector>
//#include <fstream>
//#include <sstream>

//#include <Dense>
//#include <Geometry>
//#include <Core>

//#include <ctime>

#include "Precompiled.h" // ������ �ð� ������ ����..

#include "hDef.h"
#include "fadiff.h"      // FADBAD++
//#include "fadbad.h

#include "Robot.h"
typedef T_Robot Robot;	// Robot �߻�ȭ�� ���Ͽ�

#include "ViaPoint.h"
typedef CViaPoint ViaPoint;	// ViaPoint �߻�ȭ�� ���Ͽ�

#include "T_Exo.h"
#include "T_Omni.h"


#include <CommThread.h>
#include <Network_HM.h>

using namespace std;
using namespace Eigen;


/*@ Object ID definetion */
enum CELLPHONEPART
{
	CRADLE, 
	PLUG,
	USBCON,
	EARPHONE,
};



/*@ Vision data�� ���� �ӽ� Ŭ����!! */
struct TargetObject {
	
	// object class ID
	int ID;
	
	// Clip Image ��ǥ
	int cix;
	int ciy;

	// ��ü Image ��ǥ
	int ix; 
	int iy; 

	// World ��ǥ
	double wx; 
	double wy; 
	double wz;
	
	// Robot ��ǥ
	double rox; 
	double roy; 
	double roz;
	
	// Orientation
	double Rx;		
	double Ry;		
	double Rz;	

	double density;		// KDE�� ã�� �� �е� ��
	int	cnt;			// Pixel ����

	TargetObject() {
		ID = -1;

		cix = 0;
		ciy = 0;

		ix = 0;
		iy = 0;
		
		wx = 0.0;
		wy = 0.0;
		wz = 0.0;

		rox = 0.0;
		roy = 0.0;
		roz = 0.0;
		
		Rx = 0.0;
		Ry = 0.0;
		Rz = 0.0;
		density = 0.0;
		cnt = 0;
	}
};

/*@ Networ�� �ְ���� ������ */
typedef struct
{
	double temp_joint[6];  
	double temp_point[4];  // ���� ������� �ϳ� �÷���.. button ������.. 
	double temp_ori[4];    // ���� ������� �ϳ� �÷���.. button ������.. 
	/* last element is for playback notification */
	double temp_pos_ori[7]; // ���� ������� �ϳ� �÷���.. button ������.. �ƴϴ�! button ���� ���� �ְ� ���� ���⿡ playback���� �ƴ��� ����. 
	double temp_vision[16]; // �ڽ� �� �� ������ �޾ƿ� ��. RGBY ����, ���� x, y, z, rz��.

	int temp_button;		// ��ư �Է��� ������ ��.
	
	double received_posori[7]; // ���� ������� �ϳ� �÷���.. bLastnode ������.. 
	double recv_container[20]; // 0~5 : cartesian, 6~11 : joint, 12~17 : F/T. 18 : bContact, 19 : bLastnode(�̰� �ٲ� �� ����)

	double recv_vision[16];  // �ڽ� �� �� ������ �޾ƿ� ��. RGBY ����, ���� x, y, z, rz��.
} T_NetData;


class TeachingSW_HM
{
public:
	TeachingSW_HM();
	~TeachingSW_HM();

	CCommThread		_serial;	//�ø�����Ʈ Ŭ����
	Net_Class		_tcpip;		//TCP/IP Ŭ����
	T_NetData		T_ND;		// ��Ʈ��ũ�� �ְ���� �������� ����ü
	
	VectorXd		temp_cartesian_data;	// ƼĪ �����ͷ� �����ϱ� ���� �ӽù���
	VectorXd		temp_joint_data;		// ƼĪ �����ͷ� �����ϱ� ���� �ӽù���
	MatrixXd		Teaching_data;			// ����Ǵ� ƼĪ ������
	MatrixXd		Teaching_data_j;		// ����Ǵ� ƼĪ ������ joint

	MatrixXd		Teaching_data_R;		// ����Ǵ� ƼĪ ������ (���ȷκ��� �� ������)
	MatrixXd		Teaching_data_L;		// ����Ǵ� ƼĪ ������ (���ȷκ��� �� ����)
	MatrixXd		Teaching_data_j_R;		// ����Ǵ� ƼĪ ������ joint (���ȷκ��� �� ������)
	MatrixXd		Teaching_data_j_L;		// ����Ǵ� ƼĪ ������ joint (���ȷκ��� �� ����)
	MatrixXd		Loading_data_R;			// �ε�Ǵ� ƼĪ ������ (���ȷκ��� �� ������)
	MatrixXd		Loading_data_L;			// �ε�Ǵ� ƼĪ ������ (���ȷκ��� �� ����)
	MatrixXd		Loading_data_j_R;		// �ε�Ǵ� ƼĪ ������ joint (���ȷκ��� �� ������)
	MatrixXd		Loading_data_j_L;		// �ε�Ǵ� ƼĪ ������ joint (���ȷκ��� �� ����)

	/*@  Vision boolean */
	bool bool_vision;

	/*@ Teleoperation boolean */
	bool bool_Teleoperation;
	
	/*@ Playback boolean */
	bool bool_Playback;
	/*@ Teaching data setting for playback boolean */
	bool bool_load;
	/* for playback related functions.. To track the teaching data */
	int pb_stepper;
	/* for playback_woVsn function.. To track the hash table */
	int	hash_track;				
	double event_time;

	VectorXd home_q;
	VectorXd home_x;
	VectorXd event_q;
	VectorXd event_x;
	VectorXd cur_q;
	VectorXd des_q;
	VectorXd cur_x;
	VectorXd des_x;	

	int pb_stepper_R;
	int pb_stepper_L;
	int	hash_track_R;	// �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	int	hash_track_L;				
	double event_time_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	double event_time_L;
	VectorXd home_q_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	VectorXd home_q_L;
	VectorXd home_x_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	VectorXd home_x_L;
	VectorXd event_q_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	VectorXd event_q_L;
	VectorXd event_x_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	VectorXd event_x_L;
	VectorXd cur_q_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	VectorXd cur_q_L;
	VectorXd des_q_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	VectorXd des_q_L;
	VectorXd cur_x_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)
	VectorXd cur_x_L;
	VectorXd des_x_R; // �÷��̹� �� �ʿ� ������ (���ȷκ��� �� ������)	
	VectorXd des_x_L;	
	
	// Time üũ
	double temp_time;
	double _delT;

	// HTransform from base to end-effector
	Matrix4d T0E;
	
public :
	/////////// Pointers and Instances
	/*# ViaPoint Instance */
	ViaPoint	*m_vp;
	
	/*# Robot Pointer */
	Robot		*m_robot;	// Robot parameter member ����
	
	/*# Nominal Robot Pointer */
	Robot		*nominal_robot;
	
	/*# Exoskeleton */
	T_Exo		m_exo;

	/*# Omni */
	T_Omni		m_omni;

	/*#	TargetObject from vision data	*/
	TargetObject	m_objV;
	
	/*# �κ� ������Ʈ */
	//void updateRobot(Robot *_Robot, VectorXd _q);

public :
	////////// TeachingSW �⺻ ���
	/*# save data as cartesian coordinate */ 
	bool save_taught_data_p(const char* file_name, VectorXd cartesian_data, MatrixXd& T_data);
	/*# save data as joint coordinate */ 
	bool save_taught_data_q(const char* file_name, VectorXd joint_data, MatrixXd& T_data);
	/*# save data as transformation matrix */ 
	bool save_taught_data_T(const char* file_name, MatrixXd transform_data, MatrixXd& T_data);
	/*# save continuous data (temporary. to be modified) */
	bool save_countinuous_path(bool mkfile, const char* file_name, VectorXd current_data, MatrixXd& T_data);
	/*# Add header for the taught data */
	void Addheader(const char* dirSrc, const char* dirDst);

	/*# Create hash table for the taught data */
	bool save_hash_table(const char* _path_hash, string _label);
	/*# Decode hash table for the taught data */
	bool decode_hash_table(const char* _path_hash, int &_numindx, int &_gripindx, int &_releaseindx, int &_gripindxV, int &_releaseindxV);

	/*# load taught data  */
	bool load_taught_data(const char* file_name, MatrixXd& teached_data);
	/*# Import matrix from a text file */
	void import_matrix_from_txt_file(const char* filename_X, vector <double>& v, int& rows, int& cols);
	
	
	/*# Playback with vision algorithm */
	void playback_wVsn(ViaPoint _vp, T_Robot *_robot, int &_hash_track, int _hash_numindx, int _hash_gripindx, int _hash_releaseindx,
						int _jdof, int& _pb_stepper, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback);
	/*# Playback without vision algorithm */
	void playback_woVsn(ViaPoint _vp, T_Robot *_robot, int &_hash_track, int _hash_numindx, int _hash_gripindx, int _hash_releaseindx,
						int _jdof, int& _pb_stepper, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback);

	/*# Playback without vision algorithm .. without pbstepper */
	void playback_woVsn2(ViaPoint _vp, T_Robot *_robot, int &_hash_track, int _hash_numindx, int _hash_gripindx, int _hash_releaseindx,
						int _jdof, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback);

	/*# Playback filnalist.. for total playback solution  */
	void playback_finalist(ViaPoint _vp, T_Robot *_robot, int &_hash_track,
						int _jdof, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback);

	/*# Playback filnalist.. for total playback solution... To do without decoding of hash table*/
	void playback_demo(ViaPoint _vp, T_Robot *_robot, int &_hash_track, int _subhash_GR,
						int _jdof, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback);


	
	/*# Playback algorithm.. �ϴ� ��а��� joint space������ playback �����ϵ��� ����.*/
	void playback_controller(int& _pb_stepper, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home, VectorXd& _x_home, 
			VectorXd& _event_q_0, VectorXd& _event_x_0, VectorXd& _current_q, VectorXd& _current_x, MatrixXd _Teaching_Data, MatrixXd _Teaching_Data_q, 
			VectorXd& _Desired_x, VectorXd& _Desired_q, float _resolution, bool& _bool_playback);
	/*# Playback algorithm.. �ϴ� ��а��� joint space������ playback �����ϵ��� ����.*/
	void playback_controller_q(int _jdof, int& _pb_stepper, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback);

	
	/*# Playback�� ���ؼ� grasp�� IK�� Ǫ�� �Լ� */
	void IKLoop_grasp(T_Robot *_robot, Matrix4d &_grasp, double pos_res, double ori_res, VectorXd& _approachG_q, VectorXd& _reachG_q, VectorXd& _leaveG_q);
	/*# Playback�� ���ؼ� release�� IK�� Ǫ�� �Լ� */
	void IKLoop_release(T_Robot *_robot, Matrix4d &_release, double pos_res, double ori_res, VectorXd& _approachG_q, VectorXd& _reachG_q, VectorXd& _leaveG_q);


	/*# Clear jot file */
	void clear_jobfile(const char* file_name);


public:
	/////////////// Robotics Algorithm
	/*# IK Loop. Put this in the iterative loop */
	VectorXd	IKLoop(Robot *_Amiro, Matrix4d &_goal_T, double pos_res, double ori_res, double pos_error, double ori_error, bool &_bfinished);
	/*# selective IK Loop. Select with diagonal term you want. Selection matrix has to be 6x6 sized. Put this in the iterative loop */
	VectorXd	IKLoop_selective(Robot *_Robot, Matrix4d &_goal_T, double pos_res, double ori_res, double pos_error, double ori_error, MatrixXd _selectionMatrix, bool &_bfinished);
	/*# IK Loop with Null space velocity. Put this in the iterative loop */
	VectorXd	IKLoop_with_Null(Robot *_Robot, Matrix4d &_goal_T, VectorXd Null_vel, double pos_res, double ori_res, double pos_error, double ori_error);
	/*# IK Loop of dual arm manipulator with torso */
	VectorXd	IKLoop_dualArm_with_torso(T_Robot *_AmiroR, T_Robot *_AmiroL, Matrix4d &_goal_TR, Matrix4d &_goal_TL, double pos_res, double ori_res, double pos_error, double ori_error, bool &_bfinished);
	
	/*# To make skew symmetric matrix */
	MatrixXd Skew(Vector3d _Skew);
	/*# Get Phi to control orientation*/
	void GetPhi(Matrix3d RotationMtx, Vector3d s1d, Vector3d s2d, Vector3d s3d, Vector3d& phi);

	/*# spline interpolation with cubic polynomial */
	double Cubic_path2(double rT, double rT_0, double rT_f, double rTheta_0, double rTheta_f,
												 double rTheta_dot_0, double rTheta_dot_f);
	/*# Check whether or not the robot is reached */
	bool Is_reached(VectorXd destination, VectorXd _current, float threshold);
	/*# Joint space controller */
	void JointSpaceController(MatrixXd inertia, VectorXd gravity_torque, float Kp, float Kv,
						  VectorXd _q, VectorXd desired_q, VectorXd _qdot, VectorXd desired_qdot, VectorXd& torque);
	/*# Task space controller */
	void TaskSpaceController(int jointDOF, int taskDOF, MatrixXd task_jacobian, MatrixXd inertia, VectorXd gravity_torque, 
		                  MatrixXd Kp, MatrixXd Kv, VectorXd _x, VectorXd _qdot, VectorXd _x_error, VectorXd desired_xdot, VectorXd& torque);
	
	/*# To avoid joint limit, put the center position between the joint limits */
	VectorXd AvoidJointlimitCtrl(Robot *_Robot, VectorXd _Midjoint, VectorXd _weight);
	
	
	// Zero/Home position controller �߰��ϱ�.
	
	/*# Get Manipulability */
	F<double> GetManipulability(Robot *_Robot);
	/*# Get Manipulability gradient (numerical method..) */
	void GetMnPlBltyGrad(Robot *_Robot, VectorXd &_MnPlBltyGrad);


	
public:
	////////// Orientation Representations
	/*# Get Euler Angle from fixedXYZ Transform.. dual arm */
	void GetOrientation_fXYZ(vector <float> rExoData, vector <float>& rxryrz_R, vector <float>& rxryrz_L);
	
	/*# Get Euler Angle from eulerXYZ Transform.. dual arm */
	void GetOrientation_eXYZ(Matrix3d OriR, Matrix3d OriL, Vector3d& rxryrz_R, Vector3d& rxryrz_L);
	
	/*# Get Euler Angle from eulerXYZ Transform.. from Staubli source */
	void GetOrientation_eXYZ(Matrix3d& rotMat, Vector3d& eAngle);

	/*# Get Euler Angle from eulerZYZ Transform.. dual arm */
	void GetOrientation_eZYZ(Matrix3d ExoOriR, Matrix3d ExoOriL, Vector3d& rxryrz_R, Vector3d& rxryrz_L);
	
	/*# Get Transform from eulerXYZ orientation */ 
	Matrix3d GetRfrom_eXYZ(double alpha, double beta, double gamma);

	/*# Get Transform from eulerZYZ orientation */ 
	Matrix3d GetRfrom_eZYZ(double alpha, double beta, double gamma); //GetRobotTransform�� ����� ��ü�� �� ����.

	/*# Get Transform from fixedXYZ orientation */ 
	Matrix3d GetRfrom_fXYZ(double alpha, double beta, double gamma);

	/*# Get Transform from fixedZYZ orientation */ 
	Matrix3d GetRfrom_fZYZ(double alpha, double beta, double gamma);

	/*# Extract euler parameters between two orientations */
	void EulerParameters(Matrix3d _rInitialorientation, Matrix3d _rPresentorientation, Vector3d& _rRotationalaxis, double& _rRotationalangle);



	////////// Transformation
	/*# Roatate with x axis with angles in radian */
	Matrix3d	Rotate_with_X(double rAngle);
	/*# Roatate with y axis with angles in radian */
	Matrix3d	Rotate_with_Y(double rAngle);
	/*# Roatate with z axis with angles in radian */
	Matrix3d	Rotate_with_Z(double rAngle);
	/*# Roatate with arbitrary axis with angles in radian */
	Matrix3d	Rotate_with_Axis(Vector3d rAxis, double rAngle);

	/*# Roatate with x axis with sinusoidal trajectory */
	Matrix3d	Rotate_with_X_traj(double rT, double rAngle, double rPeriod, double rTranslation);
	/*# Roatate with y axis with sinusoidal trajectory */
	Matrix3d	Rotate_with_Y_traj(double rT, double rAngle, double rPeriod, double rTranslation);
	/*# Roatate with z axis with sinusoidal trajectory */
	Matrix3d	Rotate_with_Z_traj(double rT, double rAngle, double rPeriod, double rTranslation);
	/*# Roatate with arbitrary axis with sinusoidal trajectory */
	Matrix3d	Rotate_with_Axis_traj(Vector3d rAxis, double rT, double rAngle, double rPeriod, double rTranslation);




public :
	/////// �� �� ��ɵ�
	/// ���ͷ�
	VectorXd LPF1(double alpha, VectorXd input, VectorXd output);
	VectorXd LPF2(double tau, double Ts, VectorXd input, VectorXd output);
	
	/// Vision ������ Ȱ�� �� ������ ���
	VectorXd snpshot;
	void Snapshot(VectorXd& snapshot);
	//// Vision ������ Ȱ�� �� Grip�� �߻����� ��, ID�� �ο��ϱ� ���� �迭�� ���� ���� �ֱ�
	void SetBoxID(Vector3d robotpos, VectorXd snpshot, VectorXd& _order, string& boxID);

	// ���ڿ� ��
	int StringComp(char msg1[], char msg2[]);

};

#endif