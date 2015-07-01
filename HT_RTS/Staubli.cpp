#include "StdAfx.h"
#include "Staubli.h"


CStaubli::CStaubli(void)
{
#if AMIRO
	// 뉴로메카 아미로의 경우 처음에 180도 회전!
	this->param.tmpM = this->GetHTransform(0, 0, 0, M_PI);
#endif
}


CStaubli::~CStaubli(void)
{
	delete this->param.A;
	delete this->param.T;
}

// Homogeneous Transformation Matrix
Matrix4d CStaubli::GetHTransform(float a, float alpha, float d, float theta)
{
	Matrix4d T;
	T(0,0) = cos(theta);	T(0,1) = -cos(alpha)*sin(theta);	T(0,2) = sin(alpha)*sin(theta);		T(0,3) = a*cos(theta);
	T(1,0) = sin(theta);	T(1,1) = cos(alpha)*cos(theta);		T(1,2) = -sin(alpha)*cos(theta);	T(1,3) = a*sin(theta);
	T(2,0) = 0.0f;			T(2,1) = sin(alpha);				T(2,2) = cos(alpha);				T(2,3) = d;
	T(3,0) = 0.0f;			T(3,1) = 0.0f;						T(3,2) = 0.0f;						T(3,3) = 1.0f;

	return T;
}

void CStaubli::GetHTransform(double a, double alpha, double d, double theta, Matrix4d& mat_Transform)
{
	// craig text p.75.
	mat_Transform.setIdentity();

#if CRAIG
	// craig 방식 (변환 순서: d -> theta -> a -> alpha)
	mat_Transform(0, 0) = cos(theta);
	mat_Transform(0, 1) = -sin(theta);
	mat_Transform(0, 2) = 0.0;
	mat_Transform(0, 3) = a;

	mat_Transform(1, 0) = sin(theta) * cos(alpha);
	mat_Transform(1, 1) = cos(theta) * cos(alpha);
	mat_Transform(1, 2) = -sin(alpha);
	mat_Transform(1, 3) = -sin(alpha) * d;

	mat_Transform(2, 0) = sin(theta) * sin(alpha);
	mat_Transform(2, 1) = cos(theta) * sin(alpha);
	mat_Transform(2, 2) = cos(alpha);
	mat_Transform(2, 3) = cos(alpha) * d;
#endif

#if PAUL
	//// paul 방식 (변환 순서: alpha -> a -> theta -> d)
	mat_Transform(0, 0) = cos(theta);
	mat_Transform(0, 1) = -sin(theta) * cos(alpha);
	mat_Transform(0, 2) = sin(theta) * sin(alpha);
	mat_Transform(0, 3) = a * cos(theta);

	mat_Transform(1, 0) = sin(theta);
	mat_Transform(1, 1) = cos(theta) * cos(alpha);
	mat_Transform(1, 2) = -cos(theta) * sin(alpha);
	mat_Transform(1, 3) = a * sin(theta);

	mat_Transform(2, 0) = 0;
	mat_Transform(2, 1) = sin(alpha);
	mat_Transform(2, 2) = cos(alpha);
	mat_Transform(2, 3) = d;
#endif
}


void CStaubli::GetForwardKin(Rparam *_Robot, VectorXd _des_q)
{
	UINT dof = _Robot->_jdof;
	if(AMIRO) {	// 양팔 로봇인 경우
		for(int i=0; i<dof; i++){
			this->GetHTransform(_Robot->a(i), _Robot->alpha(i), _Robot->d(i), _Robot->theta(i)+_des_q(i), _Robot->A[i]);
			if(i==0) {
				// 뉴로 메카의 경우 base frame을 180도 회전 시킴
				_Robot->T[i] = this->param.tmpM * _Robot->A[i];
			} else{
				_Robot->T[i] = _Robot->T[i-1]*_Robot->A[i];

			}
		}

		// TODO
		// end effector frame 반영
		Matrix3d end_transform_R;
		Vector3d end_transform_P;

		end_transform_R.setZero();
		end_transform_P.setZero();

		end_transform_R(0, 2) = 1.0;
		end_transform_R(1, 1) = -1.0;
		end_transform_R(2, 0) = 1.0;

		end_transform_P(0) = 145.5; // Left arm : 0.1542 , Right Arm : 0.1455

		_Robot->T0E.col(3).segment(0, 3) = _Robot->T[dof-1].col(3).segment(0, 3) + _Robot->T[dof-1].topLeftCorner(3, 3) * end_transform_P; // posiion부분 변환
		_Robot->T0E.topLeftCorner(3, 3) = _Robot->T[dof-1].topLeftCorner(3, 3) * end_transform_R; // rotation부분 변환.. position 부분 먼저 한 이유는 rotation이 변하기 전에 하기 위해..

		Vector3d vec;
		GetOrientation_eZYZ(_Robot->T0E.topLeftCorner(3,3), _Robot->T0E.topLeftCorner(3,3), vec, vec);

		//cout << _Robot->T0E << endl;
		//cout << vec*RtoD << endl;

	} else {	// 한팔 로봇인 경우
		for(int i=0; i<dof; i++){
			this->GetHTransform(_Robot->a(i), _Robot->alpha(i), _Robot->d(i), _Robot->theta(i)+_des_q(i), _Robot->A[i]);
		
			if(i==0) _Robot->T[i] = _Robot->A[i];
			else _Robot->T[i] = _Robot->T[i-1]*_Robot->A[i];
		}	
	}
}


// Position과 Orientation정보를 4x4 Matrix형태로 만들어줌.
// Orientation은 radian으로 주어야 한다.
void CStaubli::MakestaubliTransform(Vector3d rStaubliPos, Vector3d rStaubliOri, Matrix4d &rStaubliTransform)
{
	rStaubliTransform.setIdentity();
	
	double alpha, beta, gamma;
	alpha = rStaubliOri(0), beta = rStaubliOri(1), gamma = rStaubliOri(2);

	rStaubliTransform(0, 3) = rStaubliPos(0);	// x
	rStaubliTransform(1, 3) = rStaubliPos(1);	// y
	rStaubliTransform(2, 3) = rStaubliPos(2);	// z

	// direction cos
	rStaubliTransform(0, 0) = cos(beta) * cos(gamma);
	rStaubliTransform(0, 1) = -cos(beta) * sin(gamma);
	rStaubliTransform(0, 2) = sin(beta);
	rStaubliTransform(1, 0) = sin(alpha) * sin(beta) * cos(gamma) + cos(alpha) * sin(gamma);
	rStaubliTransform(1, 1) = -sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
	rStaubliTransform(1, 2) = -sin(alpha) * cos(beta);
	rStaubliTransform(2, 0) = -cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
	rStaubliTransform(2, 1) = cos(alpha) * sin(beta) * sin(gamma) + sin(alpha) * cos(gamma);
	rStaubliTransform(2, 2) = cos(alpha) * cos(beta);
}


void CStaubli::StaubliJacobian(Rparam *m_Staubli)
{
	UINT dof = m_Staubli->_jdof;
	m_Staubli->Jacobian.resize(6, dof);
	VectorXd _q(dof);
	
	if(CRAIG) {
		_q = m_Staubli->_q; //// q 값 반영.
		Matrix4d *AT = new Matrix4d[dof];

		for(int i=0; i<dof; i++){
			AT[i] = m_Staubli->T[i];
		}
	
		Vector3d *P = new Vector3d[dof]; 
		float x_offset = m_Staubli->x_offset;
		float y_offset = m_Staubli->y_offset;
		float z_offset = m_Staubli->z_offset;
	
		// Position difference
		for(int i=0; i<dof; i++){
			P[i].setZero();	// init
			P[i](0) = AT[i](0,3)+x_offset;
			P[i](1) = AT[i](1,3)+y_offset;
			P[i](2) = AT[i](2,3)+z_offset;
		}

		Vector3d z, ep, tmp;
		for(int i=0; i<dof; i++){
			// z-axis
			z(0) = AT[i](0,2);
			z(1) = AT[i](1,2);
			z(2) = AT[i](2,2);

			// difference between end-effector pos and each joint pos
			ep(0) = P[dof](0)-P[i](0);
			ep(1) = P[dof](1)-P[i](1);
			ep(2) = P[dof](2)-P[i](2);

			// z0 X [Pn-P0], z1 X [Pn-P1],... 
			tmp = z.cross(ep);

			m_Staubli->Jacobian(0, i) = tmp(0);		// x
			m_Staubli->Jacobian(1, i) = tmp(1);		// y
			m_Staubli->Jacobian(2, i) = tmp(2);		// z
			m_Staubli->Jacobian(3, i) = z(0);		// alpha
			m_Staubli->Jacobian(4, i) = z(1);		// beta
			m_Staubli->Jacobian(5, i) = z(2);		// gamma
		}
		delete AT;
		delete P;
	} 


	else if(PAUL) {
		_q = m_Staubli->_q; //// q 값 반영.
		Matrix4d iMat; iMat.setIdentity(4,4);
		Matrix4d *AT = new Matrix4d[dof+1];
		UINT nMat = dof+1;	// Identity matrix 고려해서 +1 해줌

		for(int i=0; i<nMat; i++){
			if(i==0) {
				AT[i] = iMat;	// identity matrix
			} else {
				AT[i] = m_Staubli->T[i-1];
			}
		}
	
		Vector3d *P = new Vector3d[dof+1]; 
		float x_offset = m_Staubli->x_offset;
		float y_offset = m_Staubli->y_offset;
		float z_offset = m_Staubli->z_offset;
	
		// Position difference
		for(int i=1; i<nMat; i++){
			P[0].setZero();
			P[i](0) = AT[i](0,3)+x_offset;
			P[i](1) = AT[i](1,3)+y_offset;
			P[i](2) = AT[i](2,3)+z_offset;
		}

		Vector3d z, ep, tmp;
		for(int i=1; i<nMat; i++){
			// z-axis
			z(0) = AT[i-1](0,2);
			z(1) = AT[i-1](1,2);
			z(2) = AT[i-1](2,2);

			// difference between end-effector pos and each joint pos
			ep(0) = P[dof](0)-P[i-1](0);
			ep(1) = P[dof](1)-P[i-1](1);
			ep(2) = P[dof](2)-P[i-1](2);

			tmp = z.cross(ep);

			m_Staubli->Jacobian(0, i-1) = tmp(0);	// x
			m_Staubli->Jacobian(1, i-1) = tmp(1);	// y
			m_Staubli->Jacobian(2, i-1) = tmp(2);	// z
			m_Staubli->Jacobian(3, i-1) = z(0);		// alpha
			m_Staubli->Jacobian(4, i-1) = z(1);		// beta
			m_Staubli->Jacobian(5, i-1) = z(2);		// gamma
		}
		delete AT;
		delete P;
	}

}



void CStaubli::GetInverseKin(Rparam *m_Robot, Matrix4d &_des_T, VectorXd &_dq, double _damp_param)
{
	UINT endi = m_Robot->_jdof-1;
	UINT dof = m_Robot->_jdof;

	//// _x가 몇 자유도로 들어오는 지 잘 생각해 보아야 한다. 
	VectorXd err(3);
	err = _des_T.col(3).segment(0, 3) - m_Robot->T[endi].col(3).segment(0, 3);	// x, y, z값에 대한 것만

	VectorXd _dx(6);
	

	// orientation (direction cosine)
	Vector3d _s1, _s2, _s3, _s1d, _s2d, _s3d;	
	_s1 = m_Robot->T[endi].col(0).segment(0, 3), _s2 = m_Robot->T[endi].col(1).segment(0, 3), _s3 = m_Robot->T[endi].col(2).segment(0, 3);	// 현재 로봇의 direction cosine
	_s1d = _des_T.col(0).segment(0, 3),		 _s2d = _des_T.col(1).segment(0, 3),	  _s3d = _des_T.col(2).segment(0, 3);		// goal_T 의 direction cosine
	
		
	////// Generate dx.
	// goal_T와 현재 로봇 position 과의 차이에다가 현재 로봇의 direction cosine column을 dot product 해줌
	_dx(0) = _s1.dot(err);
	_dx(1) = _s2.dot(err);
	_dx(2) = _s3.dot(err);
	
	// Jong hwa method
	_dx(3) = 0.5*(_s3.dot(_s2d) - _s3d.dot(_s2));
	_dx(4) = 0.5*(_s1.dot(_s3d) - _s1d.dot(_s3));
	_dx(5) = 0.5*(_s2.dot(_s1d) - _s2d.dot(_s1));


	//// Change Coordinate.(orientation은 일단 0으로 설정?)
	MatrixXd Rot(6, 6);
	Rot.setZero();
	for(int i=0; i<3; i++){
		Rot.col(i).segment(0, 3) = m_Robot->T[endi].col(i).segment(0, 3);
		Rot.col(i+3).segment(3, 3) = m_Robot->T[endi].col(i).segment(0, 3);
	}

	_dx = Rot * _dx;
	
	MatrixXd _mat_Identity(6, 6);
	_mat_Identity.setIdentity();
	

	if(_damp_param < 2) _damp_param = 2;
	// dq = J'inv(J*J'+damp*I)*dx
	_dq.resize(dof);
	_dq = m_Robot->Jacobian.transpose() * (m_Robot->Jacobian * m_Robot->Jacobian.transpose() + _damp_param*_damp_param*_mat_Identity).inverse() * _dx;

}


void CStaubli::GetEulerAngleRad(Matrix3d& rotMat, Vector3d& eAngle)
{
	//방법 2. Handbook of Robotics에서 찾은 것.. psi = alpha, theta = beta, phi = gamma
	double alpha = 0.0;
	double beta = 0.0;
	double gamma = 0.0;
	int sign = 1;
	if( rotMat(0,2) < (-1.0 + 1e-5*1e-5/2.0) ){
		alpha = 0;
		beta = -M_PI/2;
		gamma = atan2(rotMat(1,0), rotMat(3,1));
	}else if( rotMat(0,2) > (1.0 + 1e-5*1e-5/2.0) ) {
		alpha = 0;
		beta = M_PI/2;
		gamma = atan2(rotMat(1,0), rotMat(3,1));
	}else {
		beta = asin(rotMat(0,2));
		if( rotMat(2,2)<0.0 && rotMat(0,0)<0.0 && ( abs(rotMat(0,2))>1e-10 || abs(rotMat(1,2))>1e-10  ) )
		{
			if(beta >= 0.0){
				beta = M_PI-beta;
			}else {
				beta = -M_PI-beta;
			}
			sign = -1;
		}
		alpha = atan2(-sign*rotMat(1,2), sign*rotMat(2,2));
		gamma = atan2(-sign*rotMat(0,1), sign*rotMat(0,0));
	}

	eAngle[0] = alpha;
	eAngle[1] = beta;
	eAngle[2] = gamma;
}



void CStaubli::GetOrientation_eXYZ(Matrix3d OriR, Matrix3d OriL, Vector3d& rxryrz_R, Vector3d& rxryrz_L)
{
        rxryrz_R.resize(3);
        rxryrz_L.resize(3);
        
        /*
        /// introduction to robotics page. 43
        rxryrz_R[1] = atan2(sqrt(ExoOriR(2, 0) * ExoOriR(2, 0) + ExoOriR(2, 1) * ExoOriR(2, 1)), ExoOriR(2, 2));
        rxryrz_R[2] = atan2(-ExoOriR(2, 1)/sin(rxryrz_R[1]), ExoOriR(2, 0)/sin(rxryrz_R[1]));
        rxryrz_R[0] = atan2(ExoOriR(1, 2)/sin(rxryrz_R[1]), ExoOriR(0, 2)/sin(rxryrz_R[1]));

        //// staubli uses degrees
        rxryrz_R[1] = 180 * rxryrz_R[1]/3.141592;
        rxryrz_R[2] = 180 * rxryrz_R[2]/3.141592;
        rxryrz_R[0] = 180 * rxryrz_R[0]/3.141592;

        /// introduction to robotics page. 43
        rxryrz_L[1] = atan2(sqrt(ExoOriL(2, 0) * ExoOriL(2, 0) + ExoOriL(2, 1) * ExoOriL(2, 1)), ExoOriL(2, 2));
        rxryrz_L[2] = atan2(-ExoOriL(2, 1)/sin(rxryrz_L[1]), ExoOriL(2, 0)/sin(rxryrz_L[1]));
        rxryrz_L[0] = atan2(ExoOriL(1, 2)/sin(rxryrz_L[1]), ExoOriL(0, 2)/sin(rxryrz_L[1]));

        //// staubli uses degrees
        rxryrz_L[1] = 180 * rxryrz_L[1]/3.141592;
        rxryrz_L[2] = 180 * rxryrz_L[2]/3.141592;
        rxryrz_L[0] = 180 * rxryrz_L[0]/3.141592;

        //if(rxryrz[0] >= 180.0 && rxryrz[0] <=360)
        //      rxryrz[0] = rxryrz[0]-360;
        //else if(rxryrz[0] <=0 && rxryrz[0] >= -180.0)
        //      rxryrz[0] = -rxryrz[0];
        //else if(rxryrz[0] <= -180.0 && rxryrz[0] >= -360)
        //      we;
        */ // 기존 방식의 문제를 해결한 것이 아래 방식이다. 기존 방식에서는 특히 경계조건에 도달했을 때 문제가 발생한다.
        
        if ( OriR(0, 2) < 1){
               if ( OriR(0, 2) > -1) // -1< ... <1
               {
                       rxryrz_R[1] = asin(OriR(0, 2));
                       rxryrz_R[0] = atan2(-OriR(1, 2), OriR(2, 2));
                       rxryrz_R[2] = atan2(-OriR(0, 1), -OriR(0, 0));
               }
               else // <= -1 .. numerical problem으로 -1보다 작게 나올 수도 있음
               {
                       // Not a unique solution : rxryrz_R[2] - rxryrz_R[0] = atan2 (r10 ,r11)
                       rxryrz_R[1] = -M_PI/2;
                       rxryrz_R[0] = -atan2(OriR(1, 0), OriR(1, 1));
                       rxryrz_R[2] = 0;
               }
        }
        else // 1 <= .. numerical problem으로 1보다 크게 나올 수도 있음
        {
               // Not a unique solution : rxryrz_R[2] + rxryrz_R[0] = atan2 (r10 ,r11)
               rxryrz_R[1] = M_PI/2;
               rxryrz_R[0] = atan2(OriR(1, 0), OriR(1, 1));
               rxryrz_R[2] = 0;
        }

        if ( OriL(0, 2) < 1){
               if ( OriL(0, 2) > -1) // -1< ... <1
               {
                       rxryrz_L[1] = asin(OriL(0, 2));
                       rxryrz_L[0] = atan2(-OriL(1, 2), OriL(2, 2));
                       rxryrz_L[2] = atan2(-OriL(0, 1), -OriL(0, 0));
               }
               else // <= -1 .. numerical problem으로 -1보다 작게 나올 수도 있음
               {
                       // Not a unique solution : rxryrz_L[2] - rxryrz_L[0] = atan2 (r10 ,r11)
                       rxryrz_L[1] = -M_PI/2;
                       rxryrz_L[0] = -atan2(OriL(1, 0), OriL(1, 1));
                       rxryrz_L[2] = 0;
               }
        }
        else // 1 <= .. numerical problem으로 1보다 크게 나올 수도 있음
        {
               // Not a unique solution : rxryrz_L[2] + rxryrz_L[0] = atan2 (r10 ,r11)
               rxryrz_L[1] = M_PI/2;
               rxryrz_L[0] = atan2(OriL(1, 0), OriL(1, 1));
               rxryrz_L[2] = 0;
        }

        rxryrz_R = rxryrz_R * RtoD;
        rxryrz_L = rxryrz_L * RtoD;
}


void CStaubli::GetOrientation_eZYZ(Matrix3d ExoOriR, Matrix3d ExoOriL, Vector3d& rxryrz_R, Vector3d& rxryrz_L)
{
	rxryrz_R.resize(3);
    rxryrz_L.resize(3);
        
    /*
    /// introduction to robotics page. 43
    rxryrz_R[1] = atan2(sqrt(ExoOriR(2, 0) * ExoOriR(2, 0) + ExoOriR(2, 1) * ExoOriR(2, 1)), ExoOriR(2, 2));
    rxryrz_R[2] = atan2(-ExoOriR(2, 1)/sin(rxryrz_R[1]), ExoOriR(2, 0)/sin(rxryrz_R[1]));
    rxryrz_R[0] = atan2(ExoOriR(1, 2)/sin(rxryrz_R[1]), ExoOriR(0, 2)/sin(rxryrz_R[1]));

    //// staubli uses degrees
    rxryrz_R[1] = 180 * rxryrz_R[1]/3.141592;
    rxryrz_R[2] = 180 * rxryrz_R[2]/3.141592;
    rxryrz_R[0] = 180 * rxryrz_R[0]/3.141592;

    /// introduction to robotics page. 43
    rxryrz_L[1] = atan2(sqrt(ExoOriL(2, 0) * ExoOriL(2, 0) + ExoOriL(2, 1) * ExoOriL(2, 1)), ExoOriL(2, 2));
    rxryrz_L[2] = atan2(-ExoOriL(2, 1)/sin(rxryrz_L[1]), ExoOriL(2, 0)/sin(rxryrz_L[1]));
    rxryrz_L[0] = atan2(ExoOriL(1, 2)/sin(rxryrz_L[1]), ExoOriL(0, 2)/sin(rxryrz_L[1]));

    //// staubli uses degrees
    rxryrz_L[1] = 180 * rxryrz_L[1]/3.141592;
    rxryrz_L[2] = 180 * rxryrz_L[2]/3.141592;
    rxryrz_L[0] = 180 * rxryrz_L[0]/3.141592;

    //if(rxryrz[0] >= 180.0 && rxryrz[0] <=360)
    //      rxryrz[0] = rxryrz[0]-360;
    //else if(rxryrz[0] <=0 && rxryrz[0] >= -180.0)
    //      rxryrz[0] = -rxryrz[0];
    //else if(rxryrz[0] <= -180.0 && rxryrz[0] >= -360)
    //      we;
    */ // 기존 방식의 문제를 해결한 것이 아래 방식이다. 기존 방식에서는 특히 경계조건에 도달했을 때 문제가 발생한다.

    if ( ExoOriR(2, 2) < 1){
            if ( ExoOriR(2, 2) > -1) // -1< ... <1
            {
                    rxryrz_R[1] = acos(ExoOriR(2, 2));
                    rxryrz_R[0] = atan2(ExoOriR(1, 2), ExoOriR(0, 2));
                    rxryrz_R[2] = atan2(ExoOriR(2, 1), -ExoOriR(2, 0));
            }
            else // <= -1 .. numerical problem으로 -1보다 작게 나올 수도 있음
            {
                    // Not a unique solution : rxryrz_R[2] - rxryrz_R[0] = atan2 (r10 ,r11)
                    rxryrz_R[1] = M_PI;
                    rxryrz_R[0] = -atan2(ExoOriR(1, 0), ExoOriR(1, 1));
                    rxryrz_R[2] = 0;
            }
    }
    else // 1 <= .. numerical problem으로 1보다 크게 나올 수도 있음
    {
            // Not a unique solution : rxryrz_R[2] + rxryrz_R[0] = atan2 (r10 ,r11)
            rxryrz_R[1] = 0;
            rxryrz_R[0] = atan2(ExoOriR(1, 0), ExoOriR(1, 1));
            rxryrz_R[2] = 0;
    }

    //if ( ExoOriL(2, 2) < 1){
    //        if ( ExoOriL(2, 2) > -1) // -1< ... <1
    //        {
    //                rxryrz_L[1] = acos(ExoOriL(2, 2));
    //                rxryrz_L[0] = atan2(ExoOriL(1, 2), ExoOriL(0, 2));
    //                rxryrz_L[2] = atan2(ExoOriL(2, 1), -ExoOriL(2, 0));
    //        }
    //        else // <= -1 .. numerical problem으로 -1보다 작게 나올 수도 있음
    //        {
    //                // Not a unique solution : rxryrz_L[2] - rxryrz_L[0] = atan2 (r10 ,r11)
    //                rxryrz_L[1] = M_PI;
    //                rxryrz_L[0] = -atan2(ExoOriL(1, 0), ExoOriL(1, 1));
    //                rxryrz_L[2] = 0;
    //        }
    //}
    //else // 1 <= .. numerical problem으로 1보다 크게 나올 수도 있음
    //{
    //        // Not a unique solution : rxryrz_L[2] + rxryrz_L[0] = atan2 (r10 ,r11)
    //        rxryrz_L[1] = 0;
    //        rxryrz_L[0] = atan2(ExoOriL(1, 0), ExoOriL(1, 1));
    //        rxryrz_L[2] = 0;
    //}

    //rxryrz_R = rxryrz_R * RtoD;
    //rxryrz_L = rxryrz_L * RtoD;

}





///////////////////////////////////////////////////////////////////////////////////////////////
//// 필터류
VectorXd CStaubli::LPF1(double alpha, VectorXd input, VectorXd output)
{
	for ( int i=0; i<input.size(); i++)
		output(i) = output(i) + alpha * (input(i) - output(i));
	return output;
}

VectorXd CStaubli::LPF2(double tau, double Ts, VectorXd input, VectorXd output)
{
	for ( int i=0; i<input.size(); i++)
		output(i) = (tau/(tau + Ts)) * output(i) + (Ts/(tau + Ts)) * input(i);
	return output;
}	
