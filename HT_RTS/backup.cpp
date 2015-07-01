

// ----------------------------------- 2015.01.07 ------------------------------------------//
// Staubli.cpp 
void CStaubli::GetInverseKin(Rparam *m_Robot, Matrix4d &_des_T, VectorXd &_dq, double _damp_param)
{
//// _x�� �� �������� ������ �� �� ������ ���ƾ� �Ѵ�. 
	VectorXd _dist_x(3);
	
	// Xdiff = (Xdes - Xcurr)
	_dist_x = _des_T.col(3).segment(0, 3) - m_Robot->T06.col(3).segment(0, 3);	// x, y, z���� ���� �͸�
	//cout<<"dist_x: "<<_dist_x(0)<<" "<<_dist_x(1)<<" "<<_dist_x(2)<<endl;

	VectorXd _dx(6);
	
	//while(_dist_x.norm() > 0.01){

		// orientation (direction cosine)
		Vector3d _s1, _s2, _s3, _s1d, _s2d, _s3d;	
		_s1 = m_Robot->T06.col(0).segment(0, 3), _s2 = m_Robot->T06.col(1).segment(0, 3), _s3 = m_Robot->T06.col(2).segment(0, 3);	// ���� �κ��� direction cosine
		_s1d = _des_T.col(0).segment(0, 3),		 _s2d = _des_T.col(1).segment(0, 3),	  _s3d = _des_T.col(2).segment(0, 3);		// goal_T �� direction cosine
		
		
		////// Generate dx.
		// goal_T�� ���� �κ� position ���� ���̿��ٰ� ���� �κ��� direction cosine column�� dot product ����
		_dx(0) = _s1.dot(_dist_x);
		_dx(1) = _s2.dot(_dist_x);
		_dx(2) = _s3.dot(_dist_x);
		
		//// �ϴ� orientation�� �� ������ �ȵȴ�.

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

		//// Change Coordinate.(orientation�� �ϴ� 0���� ����?)
		MatrixXd Rot(6, 6);
		Rot.setZero();
		for(int i=0; i<3; i++){
			Rot.col(i).segment(0, 3) = m_Robot->T06.col(i).segment(0, 3);
			Rot.col(i+3).segment(3, 3) = m_Robot->T06.col(i).segment(0, 3);
		}

		_dx = Rot * _dx;
		//cout << "dx-----------------\n" << _dx << endl;

		//JacobiSVD<MatrixXd> svd(m_Robot->Jacobian, ComputeFullU | ComputeFullV); //matrix�� square�� �ƴϸ� ComputeThinU
		////cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(_dx) << endl;


		//MatrixXd _Jacobian_pinv(6, 6);
		//MatrixXd singularvals(6, 6);
		//singularvals = svd.singularValues().asDiagonal();

		//double pinvtoler = max(m_Robot->Jacobian.rows(), m_Robot->Jacobian.cols()) * m_Robot->Jacobian.norm() * 2.22*exp(-16.0); ///tolerence ������ �߻��ϴ� �κ��� �߻��Ѵ�. 
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

		//// �Ʒ� jacobian pinv�� matlab�� ���Ͽ� �Ǵٴ� ���� ��������.
		//_Jacobian_pinv = svd.matrixV() * singularvals_inv * svd.matrixU().transpose();

		//// �̰Ŵ� ���� ����ϴ� �͵�..
		//MatrixXd _jacobian_square(6, 6);
		//_jacobian_square = m_Robot->Jacobian * m_Robot->Jacobian.transpose();
		//_Jacobian_pinv = m_Robot->Jacobian.transpose() * _jacobian_square.inverse();

		_dq.resize(6);
		//_dq = _Jacobian_pinv * _dx;

		//// use damped least square method.
		//// It can be easily seen that the joint speeds are only zero if e has become zero. 
		//// A problem arises, however, when the end-effector has to go through a singularity to get to its goal. 
		//// Then, the solution to J^+ ��explodes�� and joint speeds go to infinity. 
		//// In order to work around this, we can introduce damping to the controller.
		//// ��ó : ����ġƼƼ��:://correll.cs.colorado.edu/?p=1958
		MatrixXd _mat_Identity(6, 6);
		_mat_Identity.setIdentity();
		//// damping Ʃ���ϴ� ����� ���� ���͵� ���ٰ� ��.. �׷��� ��κ� �޸���ƽ �ϴٰ� ��.
		//// �غ��ϱ� 5.0�� �ణ �Դٸ����ٸ� �ϴ� ���̰� 10.0�� ����ġ�� damped �Ǵ� ������ ����. ����.
		//// adaptive�ϰ� Ʃ���� �ִ� ���� ���ڴ�. �ʹݿ��� damping�� ũ�� �ϴٰ� ���߿��� ���� �ϴ� �������!
		//// �۱� �̰� �� �Ÿ��� �� ���� �𸣰ڴµ�, damping�� ������ �ε巯�� ����� �ٰ����� �ʰ� �������� Ƣ�� �����̴�.
		//// damping�� ����ġ�� ������ ������ ���� �����ϴ� �� �ʹ� ���� ��		
		//// damping�� 0�� ������ jacobian�� inverse�� �Ȱ����� �����µ�, �̰� �������� ���� ��찡 ����. (pseudo inverse�� �ƴϰ� ��)
		//// ��ǥ�������� �Ÿ��� * 0.01�� �ϸ� ������ �ӵ��� �����ϴ� �� ����, 0.1�� �ϸ� ������ ������. ������ ����?
		//// 0.05�� ������ ���� ���ϳ�.. ������ ���̿��� ���� ���̴� ���ظ� �� �ϰ�����, �ϴ� �������δ� ���� �������� �� �ʹ� damping�� 
		//// �۾����� �������� �߻��ϴ��� �ȵǴ� ������ �����ȴ�. ���� �� ������ ������ �̴ϸ� ���� ���� ��߰ڴ�.
		//// _dq = (m_Robot->Jacobian.transpose() * m_Robot->Jacobian + _damp_param*_damp_param*_mat_Identity).inverse() * _Jacobian_pinv * _dx;

		if(_damp_param < 2)
			_damp_param = 2;
		// �Ʒ� ���� ��ó�� : Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods
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

	//JacobiSVD<MatrixXd> svd(m_Robot->_Jacobian, ComputeFullU | ComputeFullV); //matrix�� square�� �ƴϸ� ComputeThinU
	//cout<<"Its singular values are : "<<endl<<svd.singularValues()<<endl;
	//cout<<"Its left singular vectors are the columns of the U matrix : " <<endl<<svd.matrixU()<<endl;
	//cout<<"Its right singular vectors are the columns of the V matrix : " <<endl<<svd.matrixV()<<endl;

	//cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(_dx) << endl;
}





void CStaubli::MakestaubliOri(Matrix3d& rHapticOri, Vector3d& rStaubliOri)
{
	//��ǥ�� �������� �ٽ�. z

	//��� 2. Handbook of Robotics���� ã�� ��.. psi = alpha, theta = beta, phi = gamma
	double psi = 0.0;
	double theta = 0.0;
	double phi = 0.0;
	theta = atan2(-rHapticOri.coeff(2, 0), sqrt(rHapticOri.coeff(0, 0)*rHapticOri.coeff(0, 0) + 
	rHapticOri.coeff(1, 0)*rHapticOri.coeff(1, 0)));
	psi = atan2(rHapticOri.coeff(1, 0)/cos(theta), rHapticOri.coeff(0, 0)/cos(theta));
	phi = atan2(rHapticOri.coeff(2, 1)/cos(theta), rHapticOri.coeff(2, 2)/cos(theta));

	//��.�� X-Y-Z Euler angle (refernce manual ���� staubli �� �̰� ��� �ϴ� �� ó�� ����)
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

	Vector3d omni_orihome; //�̰� �� �������� �����Դ���? omni stylus�� �������� �����ξ��� ���� ���� ��.
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
	//rStaubliOri[2] = -(ori_omni[2] - omni_orihome[2]) + staubli_orihome[2]; // z�� ���⸸ �ݴ�..

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
	//	std::cout<<"2."<<psi * 180/pi<<" "<<theta * 180/pi<<" "<<phi * 180/pi<<std::endl; //1�� ����� ���� �� Ȯ�� OK, �ٸ� tan�� ������ �ٸ�
	//	std::cout<<"4."<<alpha * 180/pi<<" "<<beta * 180/pi<<" "<<gamma * 180/pi<<std::endl; // �̰Ŵ� omni�� ���� staubli ������� �ؼ��� ��.
	//	std::cout<<rStaubliOri[0]<<" "<<rStaubliOri[1]<<" "<<rStaubliOri[2]<<std::endl;
	//	printer_count = 0;
	//}
	//printer_count++;
	
	//std::cout<<rHapticOri.coeff(0, 0)<<" h"<<rHapticOri(1, 1)<<" "<<rHapticOri(2, 2)<<std::endl;
}







void COpenGLControl::IK(CStaubli &m_Robot)
{
	////////////////IK �����ڵ� ///////////////////////////////////
	//m_Robot.param._jdof = 6;						// DOF ����
	//m_Robot.param._q.resize(m_Robot.param._jdof);	// DOF��ŭ q vector ũ�� ����
	//m_Robot.param._q.setZero();						// q�� 0���� �ʱ�ȭ

	//// Joint �ʱ� ��ġ ����
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

	// Cartesian ��ǥ ��������... 
	// Goal position, [299.99, 20.0, -360.63, -180.0*DtoR, 0.0, -180.0*DtoR]
	// 350.99, 20.0, -360.63...
	Vector3d rsPos_0, rsOri_0, rsPos_f, rsOri_f;
	rsPos_f(0) = 540.0,			rsPos_f(1) = 34.13,			rsPos_f(2) = -59.1;				// Position
	rsOri_f(0) = 0 * DtoR,			rsOri_f(1) = 0 * DtoR,		rsOri_f(2) = 0 * DtoR;		// Orientation -180, 0, -180

	// Desired position
	// �� �Լ��� ���� goal_T�� ���� �����Ѵ�. �� ���� ���������ν� timer�� �Լ��� ȣ��Ǵ� ���̴�.
	m_Robot.MakestaubliTransform(rsPos_f, rsOri_f, goal_T);	 // Position�� Orientation�� Homogeneous Matrix���·� ��ȯ
	

	//// Current position, [0.0, 20.0, 820.0, 0.0, 0.0, 0.0]
	//rsPos_0(0) = m_Robot.param.T06(0,3),		rsPos_0(1) = m_Robot.param.T06(1,3),		rsPos_0(2) = m_Robot.param.T06(2,3);
	//rsOri_0(0) = 0.0,							rsOri_0(1) = 0.0,							rsOri_0(2) = 0.0;

	
	//m_Robot.MakestaubliTransform(rsPos_0, rsOri_0, init_T);	// ���� �κ��� ��ġ�� ���� ������ Homogeneous Matrix���·� ��ȯ
	


	//VectorXd initq(6), currentq(6);
	//initq.setZero();
	//VectorXd dq(6);
	//dq.setZero();
	
	//m_Robot.param._q = m_Robot.param._q;				  // ���� q ���� �����Ѵ�. 
	//m_Robot.StaubliFK(&m_Robot.param, initq); // �ϴ� �ʱ� q ���� ���� �ʱ� transformation�� �����. �̳��� �����ϸ� T06�� �����
	//m_Robot.StaubliJacobian(&m_Robot.param);  // �ʱ� transformation���κ��� jacobian�� ���´�.



	LARGE_INTEGER liCounter1, liCounter2, liFrequency; // �Լ��� �ɸ��� �ð� �����ϱ� ���ؼ�
	QueryPerformanceFrequency(&liFrequency);  // retrieves the frequency of the high-resolution performance counter
	//QueryPerformanceCounter(&liCounter1);         // Start
	//QueryPerformanceCounter(&liCounter2);         // End
	//printf("Time : %f\n", (double)(liCounter2.QuadPart - liCounter1.QuadPart) / (double)liFrequency.QuadPart);

	//QueryPerformanceCounter(&liCounter1);         // Start

	//while ( (goal_T - m_Robot.param.T06).col(3).segment(0, 3).norm() > 1 ){ // 1mm resolution�� IK��. ��
	//	// IK ���
	//	m_Robot.GetInverseKin(&m_Robot.param, goal_T, dq, 0.05*(goal_T - m_Robot.param.T06).col(3).segment(0, 3).norm());	// dq ����س���.
	//	dq = dq * DtoR;										// DtoR���� ������ �ʹ� ���� Ŀ��������.

	//	
	//	// �κ� update
	//	m_Robot.param._q = m_Robot.param._q + dq;					// q �� ����

	//	// TODO, �� q�� ���
	//	cout << m_Robot.param._q(0)*RtoD << " " << m_Robot.param._q(1)*RtoD << " " << m_Robot.param._q(2)*RtoD << " " << m_Robot.param._q(3)*RtoD << " " 
	//		<< m_Robot.param._q(4)*RtoD << " " << m_Robot.param._q(5)*RtoD << endl;
	//	
	//	m_Robot.StaubliFK(&m_Robot.param, m_Robot.param._q);		// q ���� ���� FK �� ���� => ������ transform�� ���ؿ��� ���ؼ�.
	//	m_Robot.StaubliJacobian(&m_Robot.param);					// ���ں�� ����
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