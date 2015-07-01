#pragma once
#include <Eigen/Dense>
#include <iostream>

#define DtoR	M_PI/180	// degree to radian
#define RtoD	180/M_PI	// radian to degree

#define STAUBLI	0
#define AMIRO	!STAUBLI

#define PAUL	0
#define CRAIG	!PAUL


using namespace Eigen;
using namespace std;

// 로봇의 각종 기구학 및 DH 파라미터를 정의하는 구조체
//// 일단 6자유도 로봇을 가정하고 추상화하자.
typedef struct RoParam
{
	Matrix4d *A;	// A01, A12, A23, ...
	Matrix4d *T;	// T01, T02, T03, ...

	int _jdof;
	VectorXd _q;
	MatrixXd Jacobian;

	VectorXd a, alpha, d, theta;		// DH parameters
	float x_offset, y_offset, z_offset;	// offset parameters for drawing

	float t_a, t_alpha, t_d, t_theta;	// Additional Tool DH
	Matrix4d	tmpM;
	BOOL		t_offset;
	Matrix4d	T0E;
	

	// 생성자
	RoParam(){
#if STAUBLI
		//// ********************* Staubli ************************* ////
		_jdof = 6;	// User defined parameter
		Jacobian.resize(6,_jdof);	// 6 x DOF

		// matrix gen
		A = new Matrix4d[_jdof];
		T = new Matrix4d[_jdof];
		
		// offset (for Staubli: x: 0, y:0, z: 375)
		x_offset = 0.0f;
		y_offset = 0.0f;
		z_offset = 375.0f;

		// Set DH(Paul)
		a.resize(_jdof);alpha.resize(_jdof);	d.resize(_jdof);	theta.resize(_jdof);

		a(0) = 0.0,		alpha(0) = -M_PI/2,		d(0) = 0.0,			theta(0) = 0.0;
		a(1) = 290.0,	alpha(1) = 0.0,			d(1) = 0.0,			theta(1) = -M_PI/2;
		a(2) = 0.0,		alpha(2) = M_PI/2,		d(2) = 20.0,		theta(2) = M_PI/2;
		a(3) = 0.0,		alpha(3) = -M_PI/2,		d(3) = 310.0,		theta(3) = 0.0;
		a(4) = 0.0,		alpha(4) = M_PI/2,		d(4) = 0.0,			theta(4) = 0.0;
		a(5) = 0.0,		alpha(5) = 0.0,			d(5) = 70.0,		theta(5) = 0.0;	//_d(5) = 70+150

		// Tool DH
		t_a = 0.0f;			t_alpha = 0.0f;		t_d = 0.0f;		t_theta = 0.0f;
		t_offset = FALSE;
#endif


#if	AMIRO
		//// ********************* Amiro ************************* ////
		_jdof = 9;	// User defined parameter
		Jacobian.resize(6,_jdof);	// 6 x DOF

		// matrix gen
		A = new Matrix4d[_jdof];
		T = new Matrix4d[_jdof];
		
		// offset (for Staubli: x: 0, y:0, z: 375)
		x_offset = 0.0f;
		y_offset = 0.0f;	
		z_offset = 0.0f;	// 31:320+a2:400 = 720

		// Set DH(Craig)
		a.resize(_jdof);alpha.resize(_jdof);	d.resize(_jdof);	theta.resize(_jdof);

		// Hooman ver
		//a(0) = 0.0,			alpha(0) = 0.0,			d(0) = 320.0,		theta(0) = M_PI;
		//a(1) = 0.0,			alpha(1) = M_PI/2,		d(1) = 0.0,			theta(1) = M_PI/2;
		//a(2) = 400.0,		alpha(2) = 0,			d(2) = -208.0,		theta(2) = M_PI;
		//a(3) = 30.0,		alpha(3) = M_PI/2,		d(3) = 0.0,			theta(3) = M_PI/2;
		//a(4) = -30.0,		alpha(4) = M_PI/2,		d(4) = 330.0,/*300*/theta(4) = M_PI;
		//a(5) = -32.0,		alpha(5) = M_PI/2,		d(5) = 0.0,			theta(5) = M_PI;
		//a(6) = -32.0,		alpha(6) = M_PI/2,		d(6) = 330.0,/*300*/theta(6) = M_PI;
		//a(7) = 0.0,			alpha(7) = M_PI/2,		d(7) = 0.0,			theta(7) = -M_PI/2;
		//a(8) = -95.0,		alpha(8) = M_PI/2,		d(8) = 0.0,			theta(8) = -M_PI/2;
		//t_a = 0.0f;			t_alpha = M_PI/2;		t_d = 52.0f;		t_theta = M_PI/2;



		//// 뉴로메카 ver
		// Pre-Multiplying Matrix
		// Torso
		a(0) = 0.0,			alpha(0) = 0.0,			d(0) = 570.0,		theta(0) = 0.0;
		a(1) = 0.0,			alpha(1) = M_PI/2,		d(1) = 0.0,			theta(1) = M_PI/2;

		// Right Arm
		a(2) = 400.0,		alpha(2) = M_PI,		d(2) = 208.0,		theta(2) = M_PI;
		a(3) = 30.0,		alpha(3) = -M_PI/2,		d(3) = 0.0,			theta(3) = -M_PI/2;
		a(4) = 30.0,		alpha(4) = -M_PI/2,		d(4) = 330.0,/*300*/theta(4) = M_PI;
		a(5) = 32.0,		alpha(5) = -M_PI/2,		d(5) = 0.0,			theta(5) = M_PI;
		a(6) = 32.0,		alpha(6) = -M_PI/2,		d(6) = 330.0,/*300*/theta(6) = 0.0;
		a(7) = 0.0,			alpha(7) = M_PI/2,		d(7) = 0.0,			theta(7) = M_PI/2;
		a(8) = 95.0,		alpha(8) = -M_PI/2,		d(8) = 0.0,			theta(8) = 0.0;

		//a(9) = 0.0,			alpha(9) = M_PI/2,		d(9) = 52.0,		theta(9) = M_PI/2;

		
		// Tool DH(hr: 145.5, hl:154.2)
		t_a = 145.5;		t_alpha = 0.0f;		t_d = 0.0f;		t_theta = 0.0f;
		t_offset = FALSE;
		T0E.setZero();

#endif
	};

} Rparam;	// Robot parameters name



class CStaubli
{

public:
	CStaubli(void);
	~CStaubli(void);

	Rparam param;	// Robot parameter member 변수

public:
	Matrix4d	GetHTransform(float a, float alpha, float d, float theta);
	void		GetForwardKin(Rparam *_Robot, VectorXd _des_q);
	void		GetHTransform(double a, double alpha, double d, double theta, Matrix4d& mat_Transform);
	void		MakestaubliTransform(Vector3d rStaubliPos, Vector3d rStaubliOri, Matrix4d &rStaubliTransform);
	void		StaubliJacobian(Rparam *m_Staubli);
	void		GetInverseKin(Rparam *m_Robot, Matrix4d &_des_T, VectorXd &_dq, double _damp_param);
	VectorXd	LPF1(double alpha, VectorXd input, VectorXd output);
	VectorXd	LPF2(double tau, double Ts, VectorXd input, VectorXd output);
	void		GetEulerAngleRad(Matrix3d& rotMat, Vector3d& eAngle);

	void	GetOrientation_eXYZ(Matrix3d OriR, Matrix3d OriL, Vector3d& rxryrz_R, Vector3d& rxryrz_L);
	void	GetOrientation_eZYZ(Matrix3d OriR, Matrix3d OriL, Vector3d& rxryrz_R, Vector3d& rxryrz_L);
};