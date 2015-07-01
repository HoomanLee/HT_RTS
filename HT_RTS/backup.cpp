

// ----------------------------------- 2015.01.07 ------------------------------------------//
// Staubli.cpp 
void CStaubli::GetInverseKin(Rparam *m_Robot, Matrix4d &_des_T, VectorXd &_dq, double _damp_param)
{
//// _x가 몇 자유도로 들어오는 지 잘 생각해 보아야 한다. 
	VectorXd _dist_x(3);
	
	// Xdiff = (Xdes - Xcurr)
	_dist_x = _des_T.col(3).segment(0, 3) - m_Robot->T06.col(3).segment(0, 3);	// x, y, z값에 대한 것만
	//cout<<"dist_x: "<<_dist_x(0)<<" "<<_dist_x(1)<<" "<<_dist_x(2)<<endl;

	VectorXd _dx(6);
	
	//while(_dist_x.norm() > 0.01){

		// orientation (direction cosine)
		Vector3d _s1, _s2, _s3, _s1d, _s2d, _s3d;	
		_s1 = m_Robot->T06.col(0).segment(0, 3), _s2 = m_Robot->T06.col(1).segment(0, 3), _s3 = m_Robot->T06.col(2).segment(0, 3);	// 현재 로봇의 direction cosine
		_s1d = _des_T.col(0).segment(0, 3),		 _s2d = _des_T.col(1).segment(0, 3),	  _s3d = _des_T.col(2).segment(0, 3);		// goal_T 의 direction cosine
		
		
		////// Generate dx.
		// goal_T와 현재 로봇 position 과의 차이에다가 현재 로봇의 direction cosine column을 dot product 해줌
		_dx(0) = _s1.dot(_dist_x);
		_dx(1) = _s2.dot(_dist_x);
		_dx(2) = _s3.dot(_dist_x);
		
		//// 일단 orientation은 잘 수렴이 안된다.

		//// Ossama method
		//VectorXd pi_d(3);
		//pi_d.setZero();
		//pi_d = -0.5*(_s1.cross(_s1d) + _s2.cross(_s2d) + _s3.cross(_s3d));
		//
		//_dx(3) = pi_d(0);
		//_dx(4) = pi_d(1);
		//_dx(5) = pi_d(2);
		
		// Jong hwa method
		_dx(3) = 0.5*(_s3.dot(_s2d) - _s3d.dot(_s2));
		_dx(4) = 0.5*(_s1.dot(_s3d) - _s1d.dot(_s3));
		_dx(5) = 0.5*(_s2.dot(_s1d) - _s2d.dot(_s1));

		//cout << "pi_d:  " << pi_d(0) << "  " << pi_d(1) << "  " << pi_d(2) << endl;

		//// Change Coordinate.(orientation은 일단 0으로 설정?)
		MatrixXd Rot(6, 6);
		Rot.setZero();
		for(int i=0; i<3; i++){
			Rot.col(i).segment(0, 3) = m_Robot->T06.col(i).segment(0, 3);
			Rot.col(i+3).segment(3, 3) = m_Robot->T06.col(i).segment(0, 3);
		}

		_dx = Rot * _dx;
		//cout << "dx-----------------\n" << _dx << endl;

		//JacobiSVD<MatrixXd> svd(m_Robot->Jacobian, ComputeFullU | ComputeFullV); //matrix가 square가 아니면 ComputeThinU
		////cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(_dx) << endl;


		//MatrixXd _Jacobian_pinv(6, 6);
		//MatrixXd singularvals(6, 6);
		//singularvals = svd.singularValues().asDiagonal();

		//double pinvtoler = max(m_Robot->Jacobian.rows(), m_Robot->Jacobian.cols()) * m_Robot->Jacobian.norm() * 2.22*exp(-16.0); ///tolerence 없으면 발산하는 부분이 발생한다. 
		//MatrixXd singularvals_inv(6, 6);
		//singularvals_inv.setZero();
		//for(int i=0; i<6; i++){
		//	if(singularvals(i, i) > pinvtoler)	// diagonal term of singular values
		//		singularvals_inv(i, i) = 1/singularvals(i, i);
		//}

		//cout<<"Its singular values are : "<<endl<<svd.singularValues()<<endl;
		//for(int i=0; i<6; i++){
		//	for(int j=0; j<6; j++)
		//		cout<<singularvals(i, j)<<"	";
		//	cout<<endl;
		//}
		//cout<<"Its singular values inverse are : "<<endl<<svd.singularValues()<<endl;
		//for(int i=0; i<6; i++){
		//	for(int j=0; j<6; j++)
		//		cout<<singularvals_inv(i, j)<<"	";
		//	cout<<endl;
		//}

		//// 아래 jacobian pinv는 matlab과 비교하여 옳다는 것을 검증했음.
		//_Jacobian_pinv = svd.matrixV() * singularvals_inv * svd.matrixU().transpose();

		//// 이거는 보통 사용하는 것들..
		//MatrixXd _jacobian_square(6, 6);
		//_jacobian_square = m_Robot->Jacobian * m_Robot->Jacobian.transpose();
		//_Jacobian_pinv = m_Robot->Jacobian.transpose() * _jacobian_square.inverse();

		_dq.resize(6);
		//_dq = _Jacobian_pinv * _dx;

		//// use damped least square method.
		//// It can be easily seen that the joint speeds are only zero if e has become zero. 
		//// A problem arises, however, when the end-effector has to go through a singularity to get to its goal. 
		//// Then, the solution to J^+ “explodes” and joint speeds go to infinity. 
		//// In order to work around this, we can introduce damping to the controller.
		//// 출처 : 에이치티티피:://correll.cs.colorado.edu/?p=1958
		MatrixXd _mat_Identity(6, 6);
		_mat_Identity.setIdentity();
		//// damping 튜닝하는 방법에 대한 스터디도 많다고 함.. 그래도 대부분 휴리스틱 하다고 함.
		//// 해보니깐 5.0은 약간 왔다리갔다리 하는 편이고 10.0은 지나치게 damped 되는 느낌이 있음. 느림.
		//// adaptive하게 튜닝해 주는 것이 좋겠다. 초반에는 damping을 크게 하다가 나중에는 적게 하는 방식으로!
		//// 글구 이건 논문 거리가 될 지는 모르겠는데, damping이 작으면 부드러운 곡선으로 다가가질 않고 이쪽저쪽 튀는 느낌이다.
		//// damping이 지나치게 작으면 오히려 값에 수렴하는 데 너무 오래 걸		
		//// damping에 0을 넣으면 jacobian의 inverse와 똑같아져 버리는데, 이건 존재하지 않을 경우가 많다. (pseudo inverse가 아니게 됨)
		//// 목표지점과의 거리차 * 0.01을 하면 적당한 속도로 도착하는 것 같고, 0.1을 하면 오히려 안좋다. 기준이 뭘까?
		//// 0.05일 때에도 도달 못하네.. 숫자의 차이에서 오는 차이는 이해를 못 하겠지만, 일단 짐작으로는 거의 도달했을 때 너무 damping이 
		//// 작아지면 이쪽저쪽 발산하느라 안되는 것으로 생각된다. 일정 값 이하일 때에는 미니멈 리밋 값을 줘야겠다.
		//// _dq = (m_Robot->Jacobian.transpose() * m_Robot->Jacobian + _damp_param*_damp_param*_mat_Identity).inverse() * _Jacobian_pinv * _dx;

		if(_damp_param < 2)
			_damp_param = 2;
		// 아래 식은 출처논문 : Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods
		_dq = m_Robot->Jacobian.transpose() * (m_Robot->Jacobian * m_Robot->Jacobian.transpose() + _damp_param*_damp_param*_mat_Identity).inverse() * _dx;

		//cout<<"_dx : "<<_dx<<endl;
		//cout<<"_dq : "<<_dq<<endl;

	//}

	//MatrixXd Jacobian_inv(6, 6);
	
	//if(m_Robot->_Jacobian.determinant() < 0.1){
	//	Jacobian_inv = m_Robot->_Jacobian.transpose();
	//	cout<<"transpose"<<endl;
	//}
	//else{
	//	Jacobian_inv = m_Robot->_Jacobian.inverse() * m_Robot->_Jacobian.transpose();
	//	cout<<"pseudo inverse"<<endl;
	//}

	//JacobiSVD<MatrixXd> svd(m_Robot->_Jacobian, ComputeFullU | ComputeFullV); //matrix가 square가 아니면 ComputeThinU
	//cout<<"Its singular values are : "<<endl<<svd.singularValues()<<endl;
	//cout<<"Its left singular vectors are the columns of the U matrix : " <<endl<<svd.matrixU()<<endl;
	//cout<<"Its right singular vectors are the columns of the V matrix : " <<endl<<svd.matrixV()<<endl;

	//cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(_dx) << endl;
}





void CStaubli::MakestaubliOri(Matrix3d& rHapticOri, Vector3d& rStaubliOri)
{
	//좌표계 보정부터 다시. z

	//방법 2. Handbook of Robotics에서 찾은 것.. psi = alpha, theta = beta, phi = gamma
	double psi = 0.0;
	double theta = 0.0;
	double phi = 0.0;
	theta = atan2(-rHapticOri.coeff(2, 0), sqrt(rHapticOri.coeff(0, 0)*rHapticOri.coeff(0, 0) + 
	rHapticOri.coeff(1, 0)*rHapticOri.coeff(1, 0)));
	psi = atan2(rHapticOri.coeff(1, 0)/cos(theta), rHapticOri.coeff(0, 0)/cos(theta));
	phi = atan2(rHapticOri.coeff(2, 1)/cos(theta), rHapticOri.coeff(2, 2)/cos(theta));

	//방.법 X-Y-Z Euler angle (refernce manual 보면 staubli 는 이거 사용 하는 것 처럼 보임)
	double alpha = 0.0;
	double beta = 0.0;
	double gamma = 0.0;


	// theta
	// phi
	// psi
	beta = atan2(rHapticOri.coeff(0, 2), sqrt(rHapticOri.coeff(1, 2)*rHapticOri.coeff(1, 2)+rHapticOri.coeff(2, 2)*rHapticOri.coeff(2, 2)));
    gamma = atan2(-rHapticOri.coeff(0, 1)/cos(beta), rHapticOri.coeff(0, 0)/cos(beta));
	alpha = atan2(-rHapticOri.coeff(1, 2)/cos(beta), rHapticOri.coeff(2, 2)/cos(beta));

	printf("alpha: %f, beta: %f, gamma: %f \n", alpha, beta, gamma);

	//if(alpha < -180)
	//	alpha = alpha+360;
	//else if(alpha>180)
	//	alpha = alpha-360;
	//if(beta < -180)
	//	beta = beta+360;
	//else if(beta>180)
	//	beta = beta-360;
	//if(gamma < -180)
	//	gamma = gamma+360;
	//else if(gamma>180)
	//	gamma = gamma-360;


	double pi = 3.141592;

	Vector3d omni_orihome; //이건 뭘 기준으로 가져왔더라? omni stylus를 직각으로 세워두었을 때의 값인 듯.
	omni_orihome[0] = 0;
	omni_orihome[1] = 0;
	omni_orihome[2] = -90;


	//VectorXd staubli_orihome(3);
	Vector3d staubli_orihome;
	staubli_orihome[0] = 0.0;
	staubli_orihome[1] = -180.0;
	staubli_orihome[2] = 0.0;
	
	Vector3d ori_omni;
	ori_omni[0] = alpha * 180/pi;
	ori_omni[1] = beta * 180/pi;
	ori_omni[2] = gamma * 180/pi;

	//rStaubliOri[0] = (ori_omni[0] - omni_orihome[0]) + staubli_orihome[0];
	//rStaubliOri[1] = (ori_omni[1] - omni_orihome[1]) + staubli_orihome[1];
	//rStaubliOri[2] = -(ori_omni[2] - omni_orihome[2]) + staubli_orihome[2]; // z축 방향만 반대..

	rStaubliOri[0] = ori_omni[0];
	rStaubliOri[1] = ori_omni[1];
	rStaubliOri[2] = ori_omni[2];

	//for(int i=0; i<3; i++){
	//	if(rStaubliOri[i] < -180)
	//		rStaubliOri[i] = rStaubliOri[i]+360;
	//	else if(rStaubliOri[i]>180)
	//		rStaubliOri[i] = rStaubliOri[i]-360;
	//}

	//static int printer_count = 0;
	//if (printer_count == 10){
	//	std::cout<<"2."<<psi * 180/pi<<" "<<theta * 180/pi<<" "<<phi * 180/pi<<std::endl; //1번 방법과 같은 지 확인 OK, 다만 tan가 각도가 다름
	//	std::cout<<"4."<<alpha * 180/pi<<" "<<beta * 180/pi<<" "<<gamma * 180/pi<<std::endl; // 이거는 omni의 값을 staubli 방식으로 해석한 것.
	//	std::cout<<rStaubliOri[0]<<" "<<rStaubliOri[1]<<" "<<rStaubliOri[2]<<std::endl;
	//	printer_count = 0;
	//}
	//printer_count++;
	
	//std::cout<<rHapticOri.coeff(0, 0)<<" h"<<rHapticOri(1, 1)<<" "<<rHapticOri(2, 2)<<std::endl;
}







void COpenGLControl::IK(CStaubli &m_Robot)
{
	////////////////IK 검증코드 ///////////////////////////////////
	//m_Robot.param._jdof = 6;						// DOF 설정
	//m_Robot.param._q.resize(m_Robot.param._jdof);	// DOF만큼 q vector 크기 변경
	//m_Robot.param._q.setZero();						// q를 0으로 초기화

	//// Joint 초기 위치 설정
	//VectorXd des_q(6);
	//des_q(0) = 0.0,	des_q(1) = 55.7*DtoR,		des_q(2) = 113.06*DtoR, 
	//des_q(3) = 0.0,	des_q(4) = 11.24*DtoR,		des_q(5) = 0.0;
	//
	//m_Robot.StaubliFK(&m_Robot.param, des_q);	// Get forward kinematics
	//
	//cout << "T01\n" << m_Robot.param.T01 << "\n" << endl;
	//cout << "T02\n" << m_Robot.param.T02 << "\n" << endl;
	//cout << "T03\n" << m_Robot.param.T03 << "\n" << endl;
	//cout << "T04\n" << m_Robot.param.T04 << "\n" << endl;
	//cout << "T05\n" << m_Robot.param.T05 << "\n" << endl;
	//cout << "T06\n" << m_Robot.param.T06 << "\n" << endl;
	

	//Matrix4d init_T; //, goal_T;

	// Cartesian 좌표 기준으로... 
	// Goal position, [299.99, 20.0, -360.63, -180.0*DtoR, 0.0, -180.0*DtoR]
	// 350.99, 20.0, -360.63...
	Vector3d rsPos_0, rsOri_0, rsPos_f, rsOri_f;
	rsPos_f(0) = 540.0,			rsPos_f(1) = 34.13,			rsPos_f(2) = -59.1;				// Position
	rsOri_f(0) = 0 * DtoR,			rsOri_f(1) = 0 * DtoR,		rsOri_f(2) = 0 * DtoR;		// Orientation -180, 0, -180

	// Desired position
	// 이 함수를 통해 goal_T에 값을 설정한다. 이 값을 변경함으로써 timer의 함수가 호출되는 꼴이다.
	m_Robot.MakestaubliTransform(rsPos_f, rsOri_f, goal_T);	 // Position및 Orientation을 Homogeneous Matrix형태로 변환
	

	//// Current position, [0.0, 20.0, 820.0, 0.0, 0.0, 0.0]
	//rsPos_0(0) = m_Robot.param.T06(0,3),		rsPos_0(1) = m_Robot.param.T06(1,3),		rsPos_0(2) = m_Robot.param.T06(2,3);
	//rsOri_0(0) = 0.0,							rsOri_0(1) = 0.0,							rsOri_0(2) = 0.0;

	
	//m_Robot.MakestaubliTransform(rsPos_0, rsOri_0, init_T);	// 현재 로봇의 위치와 방위 정보를 Homogeneous Matrix형태로 변환
	


	//VectorXd initq(6), currentq(6);
	//initq.setZero();
	//VectorXd dq(6);
	//dq.setZero();
	
	//m_Robot.param._q = m_Robot.param._q;				  // 현재 q 값을 저장한다. 
	//m_Robot.StaubliFK(&m_Robot.param, initq); // 일단 초기 q 값을 통해 초기 transformation을 만든다. 이놈을 실행하면 T06이 얻어짐
	//m_Robot.StaubliJacobian(&m_Robot.param);  // 초기 transformation으로부터 jacobian을 얻어온다.



	LARGE_INTEGER liCounter1, liCounter2, liFrequency; // 함수에 걸리는 시간 측정하기 위해서
	QueryPerformanceFrequency(&liFrequency);  // retrieves the frequency of the high-resolution performance counter
	//QueryPerformanceCounter(&liCounter1);         // Start
	//QueryPerformanceCounter(&liCounter2);         // End
	//printf("Time : %f\n", (double)(liCounter2.QuadPart - liCounter1.QuadPart) / (double)liFrequency.QuadPart);

	//QueryPerformanceCounter(&liCounter1);         // Start

	//while ( (goal_T - m_Robot.param.T06).col(3).segment(0, 3).norm() > 1 ){ // 1mm resolution의 IK임. ㅋ
	//	// IK 계산
	//	m_Robot.GetInverseKin(&m_Robot.param, goal_T, dq, 0.05*(goal_T - m_Robot.param.T06).col(3).segment(0, 3).norm());	// dq 계산해내기.
	//	dq = dq * DtoR;										// DtoR하지 않으면 너무 값이 커져버린다.

	//	
	//	// 로봇 update
	//	m_Robot.param._q = m_Robot.param._q + dq;					// q 값 갱신

	//	// TODO, 각 q값 출력
	//	cout << m_Robot.param._q(0)*RtoD << " " << m_Robot.param._q(1)*RtoD << " " << m_Robot.param._q(2)*RtoD << " " << m_Robot.param._q(3)*RtoD << " " 
	//		<< m_Robot.param._q(4)*RtoD << " " << m_Robot.param._q(5)*RtoD << endl;
	//	
	//	m_Robot.StaubliFK(&m_Robot.param, m_Robot.param._q);		// q 값에 의한 FK 값 갱신 => 현재의 transform을 구해오기 위해서.
	//	m_Robot.StaubliJacobian(&m_Robot.param);					// 자코비안 갱신
	//	m_Robot.param._q = m_Robot.LPF1(0.99, m_Robot.param._q + dq, m_Robot.param._q);

	//	// Orientation 
	//	/*Matrix3d testori;
	//	Vector3d tetetete;
	//	for(int i=0; i<3; i++)
	//		for(int j=0; j<3; j++)
	//			testori(i, j) = m_Robot.param.T06(i, j);
	//	m_Robot.MakestaubliOri(testori, tetetete);*/
	//	
	//}

	//QueryPerformanceCounter(&liCounter2);         // End
	printf("Time : %f\n", (double)(liCounter2.QuadPart - liCounter1.QuadPart) / (double)liFrequency.QuadPart);
	//////////////////////////////////////////////////////////////////////
}