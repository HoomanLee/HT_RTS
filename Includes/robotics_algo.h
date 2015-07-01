#ifndef __ROBOTICS_ALGO_H__
#define __ROBOTICS_ALGO_H__

//#include <iostream>
//#include <conio.h>
//#include <math.h>
//#include <vector>
//#include <fstream>
//#include <sstream>
//
//#include <Dense>
//#include <Geometry>
//#include <Core>

#include "Precompiled.h" // 컴파일 시간 단축을 위해..

using namespace std;
using namespace Eigen;

class robotics_algo
{
public:
	
	
	void JointSpaceController(MatrixXd inertia, VectorXd gravity_torque, float Kp, float Kv,
						  VectorXd _q, VectorXd desired_q, VectorXd _qdot, VectorXd desired_qdot, VectorXd& torque);
	void TaskSpaceController(int jointDOF, int taskDOF, MatrixXd task_jacobian, MatrixXd inertia, VectorXd gravity_torque, 
		                  MatrixXd Kp, MatrixXd Kv, VectorXd _x, VectorXd _qdot, VectorXd _x_error, VectorXd desired_xdot, VectorXd& torque);
	
	double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f);
	double Cubic_path(double rT, double rT_0, double rT_f, double rTheta_0, double rTheta_f);
	double Cubic_path2(double rT, double rT_0, double rT_f, double rTheta_0, double rTheta_f,
												 double rTheta_dot_0, double rTheta_dot_f);
	double Cubic_q(float rT, float rT_f, float rTheta_0, float rTheta_f);

	MatrixXd Skew(VectorXd _Skew);
	void GetPhi(MatrixXd RotationMtx, VectorXd s1d, VectorXd s2d, VectorXd s3d, VectorXd& phi);

	double Check_Manipulability(MatrixXd rJacobian);

	MatrixXd	Rotate_with_X(double rAngle);
	MatrixXd	Rotate_with_Y(double rAngle);
	MatrixXd	Rotate_with_Z(double rAngle);
	MatrixXd	Rotate_with_X_traj(double rT, double rAngle, double rPeriod, double rTranslation);
	MatrixXd	Rotate_with_Y_traj(double rT, double rAngle, double rPeriod, double rTranslation);
	MatrixXd	Rotate_with_Z_traj(double rT, double rAngle, double rPeriod, double rTranslation);

	MatrixXd	Rotate_with_Axis_traj(VectorXd rAxis, double rT, double rAngle, double rPeriod, double rTranslation);

	void RotationSetFromVectorAToVectorB(const VectorXd vA, const VectorXd vB, MatrixXd& rotation);

	bool robotics_algo::Is_reached_q(VectorXd destination, VectorXd _q, float threshold);

	VectorXd cross_product(VectorXd A, VectorXd B);

private:

};

#endif