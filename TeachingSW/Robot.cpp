#include "Precompiled.h"  
#include "Robot.h"


T_Robot::T_Robot(void)
{
	Initialization();
}


T_Robot::~T_Robot(void)
{
	if(A)
		delete A;
	if(T)
		delete T;
}

// ���� �ʱ�ȭ �Լ�
void T_Robot::Initialization()
{
	A = NULL;	// A01, A12, A23, ...
	T = NULL;	// T01, T02, T03, ...

	// etc parameters
	dof = 0;
	end = 0;
	q.setZero();
	Jacobian.setZero();
	kinType = CRAIG;
	
	// DH parameter init
	a.setZero();
	alpha.setZero();
	d.setZero();
	theta.setZero();

	// Offset parameter init
	x_offset = 0.0;
	y_offset = 0.0;
	z_offset = 0.0;
	
	// Tool parameter init
	t_a = 0.0;
	t_alpha = 0.0;
	t_d = 0.0;
	t_theta = 0.0;

	// Orientation Notation setting
	oriType = Exyz;

	// ZYZ Euler angle�� ȸ����Ų rotation matrix(AMIRO end-effector�� ���� ��ȯ H)
	T0E.setZero();
	T_Tooloffset.setIdentity();
	
	bTool = FALSE;

	// ���θ�ī �ⱸ���� �ʱ� Transform
	preH.setIdentity();
	postH.setIdentity();

	griptime = 0.5;
	releasetime = 0.5;
}



void T_Robot::SetDH(VectorXd _a, VectorXd _alpha, VectorXd _d, VectorXd _theta, unsigned int _dof, BOOL _kinType)
{
	// DH parameter vector ũ��� DOF�� ���� ������ �˻�
	if(_dof != _a.size() || _dof != _alpha.size() || _dof != _d.size() || _dof != _theta.size()) {
		cout << "Parameter size and DOF should be equal.." << endl;
		return;
	}
	
	dof = _dof;									// DOF setting
	A = new Matrix4d[dof];						// A Matrix setting
	T = new Matrix4d[dof];						// T Matrix setting
	Jacobian.resize(6,dof);						// Jacobian setting, 6xN
	q.resize(dof);	q.setZero();				// q setting
	kinType = _kinType;							// Type of kinematics
	
	a.resize(dof);		a.setZero();			// DH parameter setting
	alpha.resize(dof);	alpha.setZero();
	d.resize(dof);		d.setZero();
	theta.resize(dof);	theta.setZero();

	a		= _a;
	alpha	= _alpha;
	d		= _d;
	theta	= _theta;
}


void T_Robot::SetToolDH(double _a, double _alpha, double _d, double _theta, BOOL isTool)
{
	bTool = isTool;

	t_a		= _a;
	t_alpha = _alpha;
	t_d		= _d;
	t_theta = _theta;

	T_Tooloffset = GetHTransform(t_a, t_alpha, t_d, t_theta);
}

void T_Robot::SetOffset(double _x_offset, double _y_offset, double _z_offset)
{
	x_offset = _x_offset;
	y_offset = _y_offset;
	z_offset = _z_offset;
}

void T_Robot::SetQ(int index, double rad)
{
	if(dof <= 0){
		cout << "DH parameter is not setup" << endl;
		return;
	}

	if(index < 0 || index > dof-1) {
		cout << "Wrong index!" << endl;
		return;
	}

	this->q(index) = rad;
	CalcFK();
}

void T_Robot::SetQ(VectorXd q_rad)
{
	if(dof <= 0){
		cout << "DH parameter is not setup" << endl;
		return;
	}

	if(q_rad.size() != dof) {
		cout << "Wrong size!" << endl;
		return;
	}
	
	q = q_rad;
	CalcFK();
}

// PreH Matrix����. �⺻ ���� Identity Matrix
void T_Robot::SetPreH(double _a, double _alpha, double _d, double _theta)
{
	preH = GetHTransform(_a, _alpha, _d, _theta);
}

void T_Robot::SetPostH(double _a, double _alpha, double _d, double _theta)
{
	postH = GetHTransform(_a, _alpha, _d, _theta);
}

void T_Robot::SetPostH(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double a13,
					  double a20, double a21, double a22, double a23, double a30, double a31, double a32, double a33)
{
	//_postH(0,2) = 1.0;
	//_postH(1,1) = -1.0;
	//_postH(2,0) = 1.0;
	//_postH(0,3) = 145.5;	// Left arm : 0.1542 , Right Arm : 0.1455

	postH(0,0) = a00; postH(0,1) = a01; postH(0,2) = a02; postH(0,3) = a03;
	postH(1,0) = a10; postH(1,1) = a11; postH(1,2) = a12; postH(1,3) = a13;
	postH(2,0) = a20; postH(2,1) = a21; postH(2,2) = a22; postH(2,3) = a23;
	postH(3,0) = a30; postH(3,1) = a31; postH(3,2) = a32; postH(3,3) = a33;

}



void T_Robot::SetOriType(char* str)
{
	if(!strcmp(str, "Exyz"))
	{
		oriType = Exyz;
	}
	else if(!strcmp(str, "Ezyz"))
	{
		oriType = Ezyz;
	}
	else if(!strcmp(str, "Estaubli"))
	{
		oriType = Estaubli;
	}
	else
	{
		cout << "Wrong notation!" << endl;
		return;
	}
}



int T_Robot::GetDOF()
{
	if(dof <= 0){
		cout << "DH parameter is not setup" << endl;
		return NULL;
	}

	return dof;
}

Matrix4d T_Robot::GetA(int iq)
{
	if(dof <= 0){
		cout << "DH parameter is not setup" << endl;
	}

	return A[iq];
}

Matrix4d T_Robot::GetT(int iq)
{
	if(dof <= 0){
		cout << "DH parameter is not setup" << endl;
	}

	return T[iq];
}

MatrixXd T_Robot::GetJacobian()
{
	if(dof <= 0){
		cout << "DH parameter is not setup" << endl;
	}

	return Jacobian;
}

Matrix4d T_Robot::GetTool()
{
	return T0E;
}


MatrixXd T_Robot::GetDH()
{
	MatrixXd DH;
	DH.resize(dof, 4);

	DH.col(0) = a;			// mm
	DH.col(1) = alpha;		// rad
	DH.col(2) = d;			// mm
	DH.col(3) = theta;		// rad
	
	return DH;
}


double T_Robot::GetOffset(int index)
{
	if(index < 0 || index > 2) {
		cout << "Wrong index!" << endl;
		return NULL;
	}
	
	switch(index) {
	case 0: 
		return x_offset;
	case 1:
		return y_offset;
	case 2:
		return z_offset;
	}
	
	return NULL;	// dummy return
}

bool T_Robot::isTool()
{
	return bTool;
}


Matrix4d *T_Robot::GetpT()
{
	return T;
}

double T_Robot::GetQ(int index)
{
	if(dof <= 0){
		cout << "DH parameter is not setup" << endl;
		return NULL;
	}

	if(index < 0 || index > dof-1) {
		cout << "Wrong index!" << endl;
		return NULL;
	}

	return this->q(index);
}

VectorXd T_Robot::GetQ()
{
	if(dof <= 0){
		cout << "DH parameter is not setup" << endl;
	}
	
	return q;
}

// End-effector�� position ����
Vector3d T_Robot::GetPosition()
{
	return GetTool().col(3).segment(0,3);
}

// End-effector�� orientation ����
Vector3d T_Robot::GetOrientation(char *str)
{
	Vector3d oriVec;
	oriVec.setZero();
	
	if(!strcmp(str, "Exyz")) 
	{
		oriVec = GetOrientation_eXYZ(GetTool().topLeftCorner(3,3));
	}
	else if(!strcmp(str, "Ezyz"))
	{
		oriVec = GetOrientation_eZYZ(GetTool().topLeftCorner(3,3));
	}
	else 
	{
		cout << "This orientation does not exist! " << endl;
	}

	return oriVec;
}

Vector3d T_Robot::GetOrientation()
{
	Vector3d oriVec;
	oriVec.setZero();

	if(oriType == Exyz)
	{
		oriVec = GetOrientation_eXYZ(GetTool().topLeftCorner(3,3));
	}
	else if(oriType == Ezyz)
	{
		oriVec = GetOrientation_eZYZ(GetTool().topLeftCorner(3,3));
	}
	else if(oriType == Estaubli)
	{
		oriVec = GetEulerAngleRad(GetTool().topLeftCorner(3,3));
	}

	return oriVec;
}

// �߰�!
VectorXd T_Robot::GetPosNOri()
{
	VectorXd V = VectorXd::Zero(6);
	
	// position
	V.segment(0,3) = GetTool().col(3).segment(0,3);

	// orientation
	V.segment(3,3) = GetOrientation();

	return V;
}


// �Էµ� DH �Ķ���Ϳ� �´� H matrix�� ��������
Matrix4d T_Robot::GetHTransform(double a, double alpha, double d, double theta)
{
	// craig text p.75.
	Matrix4d mat_Transform;
	mat_Transform.setIdentity();


	if(kinType == CRAIG) {
		// craig ��� (��ȯ ����: d -> theta -> a -> alpha)
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
	}
	else if(kinType == PAUL) {
		//// paul ��� (��ȯ ����: alpha -> a -> theta -> d)
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
	}

	return mat_Transform;
}


// DH ��� �׸��� ���� ����Ʈ ���
MatrixXd T_Robot::GetPointsforDraw()	// TODO
{
	// [x1,y1,z1, x2,y2,z2]
	//MatrixXd P = MatrixXd::Zero(GetDOF(),6);

	//Matrix4d tempT;
	//for(int i=0; i<dof; i++){

	//	//if(kinType == CRAIG)		A[i] = GetHTransform(a(i), alpha(i), 0.0, 0.0);
	//	//else if(kinType == PAUL)	A[i] = GetHTransform(0.0, 0.0, d(i), theta(i)+q(i));

	//	if(i==0) {
	//		// Base ��ǥ�� ȸ�����Ѿ� �� �� preH�� �̿���
	//		//T[i] = preH * A[i];	// ���� ��ī AMIRO�� ��� base frame�� 180�� ȸ�� ��Ŵ

	//		if(kinType == CRAIG)		tempT = preH * GetHTransform(a(i), alpha(i), 0.0, 0.0);
	//		else if(kinType == PAUL)	tempT = preH * GetHTransform(0.0, 0.0, d(i), theta(i)+q(i));
	//	}else{
	//		//T[i] = T[i-1]*A[i];	
	//		if(kinType == CRAIG)		tempT = preH * GetHTransform(a(i-1), alpha(i-1), 0.0, 0.0) * preH * GetHTransform(a(i), alpha(i), 0.0, 0.0);
	//		else if(kinType == PAUL)	tempT = preH * GetHTransform(a(i-1), alpha(i-1), 0.0, 0.0) * preH * GetHTransform(0.0, 0.0, d(i), theta(i)+q(i));
	//	}
	//	P.row(i).segment(0,3) = tempT.col(3).segment(0,3);	// first point



	//	A[i] = GetHTransform(a(i), alpha(i), d(i), theta(i)+q(i));
	//	if(i==0) {
	//		// Base ��ǥ�� ȸ�����Ѿ� �� �� preH�� �̿���
	//		//T[i] = preH * A[i];	// ���� ��ī AMIRO�� ��� base frame�� 180�� ȸ�� ��Ŵ
	//		
	//		tempT = preH * GetHTransform(a(i), alpha(i), d(i), theta(i)+q(i));
	//				}else{
	//		//T[i] = T[i-1]*A[i];
	//		tempT = preH * GetHTransform(a(i), alpha(i), d(i), theta(i)+q(i)) * preH * GetHTransform(a(i), alpha(i), d(i), theta(i)+q(i));
	//	}
	//	P.row(i).segment(3,3) = tempT.col(3).segment(0,3);	// second point 
	//}

	///////////////////////
	//for(int i=0; i<dof; i++){

	//	if(kinType == CRAIG)		A[i] = GetHTransform(a(i), alpha(i), 0.0, 0.0);
	//	else if(kinType == PAUL)	A[i] = GetHTransform(0.0, 0.0, d(i), theta(i)+q(i));

	//	if(i==0) {
	//		// Base ��ǥ�� ȸ�����Ѿ� �� �� preH�� �̿���
	//		T[i] = preH * A[i];	// ���� ��ī AMIRO�� ��� base frame�� 180�� ȸ�� ��Ŵ
	//	}else{
	//		T[i] = T[i-1]*A[i];	
	//	}
	//	P.row(i).segment(0,3) = T[i].col(3).segment(0,3);	// first point



	//	A[i] = GetHTransform(a(i), alpha(i), d(i), theta(i)+q(i));
	//	if(i==0) {
	//		// Base ��ǥ�� ȸ�����Ѿ� �� �� preH�� �̿���
	//		T[i] = preH * A[i];	// ���� ��ī AMIRO�� ��� base frame�� 180�� ȸ�� ��Ŵ
	//	}else{
	//		T[i] = T[i-1]*A[i];
	//	}
	//	P.row(i).segment(3,3) = T[i].col(3).segment(0,3);	// second point 

	//}

    // [x1,y1,z1, x2,y2,z2]
    MatrixXd P = MatrixXd::Zero(GetDOF()+isTool(),6);    // tool�� ������ dof+1, ������ �� dof

	Matrix4d tA, tT, preT;

    for(int i=0; i<dof; i++){
        tA.setIdentity();
        tT.setIdentity();

        if(kinType == CRAIG)          tA = GetHTransform(a(i), alpha(i), 0.0, 0.0);
        else if(kinType == PAUL)      tA = GetHTransform(0.0, 0.0, d(i), theta(i)+q(i));

        if(i==0) {
                // Base ��ǥ�� ȸ�����Ѿ� �� �� preH�� �̿���
                tT = preH * tA;        // ���� ��ī AMIRO�� ��� base frame�� 180�� ȸ�� ��Ŵ
        }else{
                tT = preT*tA;  
        }
        P.row(i).segment(0,3) = tT.col(3).segment(0,3);      // first point



        tA = GetHTransform(a(i), alpha(i), d(i), theta(i)+q(i));
        if(i==0) {
                // Base ��ǥ�� ȸ�����Ѿ� �� �� preH�� �̿���
                tT = preH * tA;        // ���� ��ī AMIRO�� ��� base frame�� 180�� ȸ�� ��Ŵ
        }else{
                tT = preT*tA;
        }
        P.row(i).segment(3,3) = tT.col(3).segment(0,3);      // second point 
               
        preT = tT;
    }
        
    // Tool�� �����Ǿ� ���� ��� Tool�� ���� ��ǥ ����
    if(isTool())
    {
        P.row(P.rows()-1).segment(0,3) = P.row(P.rows()-2).segment(0,3);
        P.row(P.rows()-1).segment(3,3) = GetTool().col(3).segment(0,3);
    }
        
    return P;
}



// q���� ���� H matrix(T01, T02, ... T0N�� ����Ѵ�.)
void T_Robot::CalcFK()
{
	for(int i=0; i<dof; i++){
		A[i] = GetHTransform(a(i), alpha(i), d(i), theta(i)+this->q(i));
		if(i==0) {
			// Base ��ǥ�� ȸ�����Ѿ� �� �� preH�� �̿���
			T[i] = preH * A[i];	// ���� ��ī AMIRO�� ��� base frame�� 180�� ȸ�� ��Ŵ
		}else{
			T[i] = T[i-1]*A[i];
		}
	}

	// Tool�� �����Ǿ� ���� ���..(T_Tooloffset�� DH�� ���ǵ� tool�̰� postH�� AMIRO ������ �־� ���.. )
	if(bTool) {
		T0E = T[dof-1]*T_Tooloffset;
		T0E.col(3).segment(0,3) = T0E.col(3).segment(0,3) + T0E.topLeftCorner(3,3) * postH.col(3).segment(0,3);	// position
		T0E.topLeftCorner(3,3) = T0E.topLeftCorner(3,3) * postH.topLeftCorner(3,3);		// orientation
	}else {
		T0E = T[dof-1];
	}
}

// Damped Least Square Method
void T_Robot::CalcIK(Matrix4d &goal_T, VectorXd &_dq, double _damp_param)
{
	//// Desired position�� Current position������ Error��
	VectorXd err(3);
	err = goal_T.col(3).segment(0,3) - GetTool().col(3).segment(0,3);	// x, y, z���� ���� �͸�

	VectorXd _dx(6);

	// orientation (direction cosine) direction cosine vector (x, y, z�ະ)
	Vector3d vRxc, vRyc, vRzc,    vRxd, vRyd, vRzd;	// c: current, d: desired
	vRxc = GetTool().col(0).segment(0, 3),	vRyc = GetTool().col(1).segment(0, 3),	vRzc = GetTool().col(2).segment(0, 3);	// ���� �κ��� direction cosine
	vRxd = goal_T.col(0).segment(0, 3),		vRyd = goal_T.col(1).segment(0, 3),		vRzd = goal_T.col(2).segment(0, 3);	// goal_T �� direction cosine
			
	////// Generate dx.
	//// Method 1
	// goal_T�� ���� �κ� position ���� ���̿��ٰ� ���� �κ��� direction cosine column�� dot product ����
	//_dx(0) = vRxc.dot(err);
	//_dx(1) = vRyc.dot(err);
	//_dx(2) = vRzc.dot(err);
	
	//_dx(3) = 0.5*(vRzc.dot(vRyd) - vRzd.dot(vRyc));
	//_dx(4) = 0.5*(vRxc.dot(vRzd) - vRxd.dot(vRzc));
	//_dx(5) = 0.5*(vRyc.dot(vRxd) - vRyd.dot(vRxc));

	////// Change Coordinate.(orientation�� �ϴ� 0���� ����?)
	//MatrixXd Rot(6, 6);
	//Rot.setZero();
	//for(int i=0; i<3; i++){
	//	Rot.col(i).segment(0, 3) = GetTool().col(i).segment(0, 3);
	//	Rot.col(i+3).segment(3, 3) = GetTool().col(i).segment(0, 3);
	//}

	//_dx = Rot * _dx;
	
	//// Method 2
	//// CRAIG ���
	_dx(0) = err(0);
	_dx(1) = err(1);
	_dx(2) = err(2);
	
	Vector3d ori2;
	ori2 = 0.5*(vRxc.cross(vRxd) + vRyc.cross(vRyd) + vRzc.cross(vRzd));

	_dx(3) = ori2(0);
	_dx(4) = ori2(1);
	_dx(5) = ori2(2);
		
	MatrixXd _mat_Identity(6, 6);
	_mat_Identity.setIdentity();
	
	//if(_damp_param < 1) _damp_param = 1;
	// dq = J'inv(J*J'+damp*I)*dx
	_dq.resize(dof);
	
	// TODO -------------------------------------------------
	//ColPivHouseholderQR<MatrixXd> qr(GetJacobian());
	//_dq = qr.solve(_dx);
	// TODO -------------------------------------------------
	
	_dq = GetJacobian().transpose() * (GetJacobian() * GetJacobian().transpose() + _damp_param*_damp_param*_mat_Identity).inverse() * _dx;
}

void T_Robot::CalcIK_custom(MatrixXd _Jacobian, VectorXd _dx, VectorXd &_dq, double _damp_param)
{
	// dx�� ���ڷ� �������� ����ڰ� dx�� �����ؾ� �Ѵٴ� ���ε�, �̸� ���ؼ� get phi ���� ���� ��������. (�̹� �ֱ� ��)
	// �������� ���� IK�� ����. JLA�� custom�� ������ �Ѵٴ� ���� �޶����±�.
	// �̷��� ����� ����, with torso�� without torso�� ��� jacobian�� �����ָ� �Ǵ� ��Ȳ�� �ȴ�.
	// ��, �׷��� dq�� size ������ ������ �װ͵� �� �����غ���.
}
// Damped Least Square Method
void T_Robot::CalcIK_JLA(Matrix4d &goal_T, VectorXd &_dq, double _damp_param, VectorXd _Midjoint, MatrixXd _weight)
{
	//// Desired position�� Current position������ Error��
	VectorXd err(3);
	err = goal_T.col(3).segment(0,3) - GetTool().col(3).segment(0,3);	// x, y, z���� ���� �͸�

	VectorXd _dx(6);

	// orientation (direction cosine) direction cosine vector (x, y, z�ະ)
	Vector3d vRxc, vRyc, vRzc,    vRxd, vRyd, vRzd;	// c: current, d: desired
	vRxc = GetTool().col(0).segment(0, 3),	vRyc = GetTool().col(1).segment(0, 3),	vRzc = GetTool().col(2).segment(0, 3);	// ���� �κ��� direction cosine
	vRxd = goal_T.col(0).segment(0, 3),		vRyd = goal_T.col(1).segment(0, 3),		vRzd = goal_T.col(2).segment(0, 3);	// goal_T �� direction cosine
			
	////// Generate dx.
	//// Method 1
	// goal_T�� ���� �κ� position ���� ���̿��ٰ� ���� �κ��� direction cosine column�� dot product ����
	//_dx(0) = vRxc.dot(err);
	//_dx(1) = vRyc.dot(err);
	//_dx(2) = vRzc.dot(err);
	
	//_dx(3) = 0.5*(vRzc.dot(vRyd) - vRzd.dot(vRyc));
	//_dx(4) = 0.5*(vRxc.dot(vRzd) - vRxd.dot(vRzc));
	//_dx(5) = 0.5*(vRyc.dot(vRxd) - vRyd.dot(vRxc));

	////// Change Coordinate.(orientation�� �ϴ� 0���� ����?)
	//MatrixXd Rot(6, 6);
	//Rot.setZero();
	//for(int i=0; i<3; i++){
	//	Rot.col(i).segment(0, 3) = GetTool().col(i).segment(0, 3);
	//	Rot.col(i+3).segment(3, 3) = GetTool().col(i).segment(0, 3);
	//}

	//_dx = Rot * _dx;


	//// Method 2
	//// CRAIG ���
	_dx(0) = err(0);
	_dx(1) = err(1);
	_dx(2) = err(2);
	
	Vector3d ori2;
	ori2 = 0.5*(vRxc.cross(vRxd) + vRyc.cross(vRyd) + vRzc.cross(vRzd));

	_dx(3) = ori2(0);
	_dx(4) = ori2(1);
	_dx(5) = ori2(2);
	
	
	MatrixXd _mat_Identity(6, 6);
	_mat_Identity.setIdentity();
	

	//if(_damp_param < 1) _damp_param = 1;
	// dq = J'inv(J*J'+damp*I)*dx
	_dq.resize(dof);
	
	// TODO -------------------------------------------------
	//ColPivHouseholderQR<MatrixXd> qr(GetJacobian());
	//_dq = qr.solve(_dx);
	// TODO -------------------------------------------------
	

	_dq = GetJacobian().transpose() * (GetJacobian() * GetJacobian().transpose() + _damp_param*_damp_param*_mat_Identity).inverse() * _dx;
	
	////////////      null space velocity �����ֱ�       //////////////////////
	MatrixXd _Ident;
	_Ident.setIdentity(GetJacobian().rows(), GetJacobian().rows());
	//ColPivHouseholderQR<MatrixXd> _qr(_Robot.GetJacobian());
	MatrixXd _pseudo_inv_J;
	_pseudo_inv_J.resize(GetJacobian().cols(), GetJacobian().rows());
	_pseudo_inv_J = GetJacobian().transpose() * (GetJacobian() * GetJacobian().transpose() + _damp_param*_damp_param*_Ident).inverse();

	// Null Velocity ����
	VectorXd Nullvel(GetDOF());
	Nullvel.setZero();

	Nullvel = _weight * (_Midjoint - GetQ());

	// Null Velocity ����
	MatrixXd _Ident2;
	_Ident2.setIdentity(GetDOF(), GetDOF());
	_dq = _dq + (_Ident2 - _pseudo_inv_J * GetJacobian())*Nullvel;
}


// Position�� Orientation������ 4x4 Matrix(Direction cos + position)���·� �������.
// Orientation�� radian���� �־�� �Ѵ�.
void T_Robot::MakeRobotTransform(Vector3d Pos, Vector3d Ori, Matrix4d &rTransform, char* str)
{
	rTransform.setIdentity();
	
	double alpha, beta, gamma;
	alpha = Ori(0), beta = Ori(1), gamma = Ori(2);

	rTransform(0, 3) = Pos(0);	// x
	rTransform(1, 3) = Pos(1);	// y
	rTransform(2, 3) = Pos(2);	// z
	

	if(!strcmp(str, "Exyz"))	// ���ڰ� ��ġ�ϸ� 0�̹Ƿ� !�� �ٿ��ش�. 
	{
		// direction cos (Rxyz Euler Angle)
		rTransform(0, 0) = cos(beta) * cos(gamma);
		rTransform(0, 1) = -cos(beta) * sin(gamma);
		rTransform(0, 2) = sin(beta);
		rTransform(1, 0) = sin(alpha) * sin(beta) * cos(gamma) + cos(alpha) * sin(gamma);
		rTransform(1, 1) = -sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
		rTransform(1, 2) = -sin(alpha) * cos(beta);
		rTransform(2, 0) = -cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
		rTransform(2, 1) = cos(alpha) * sin(beta) * sin(gamma) + sin(alpha) * cos(gamma);
		rTransform(2, 2) = cos(alpha) * cos(beta);
	}
	else if(!strcmp(str, "Ezyz"))
	{
		rTransform(0, 0) = cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma);
        rTransform(0, 1) = -cos(alpha)*cos(beta)*sin(gamma) - sin(alpha)*cos(gamma);
        rTransform(0, 2) = cos(alpha)*sin(beta);
        rTransform(1, 0) = sin(alpha)*cos(beta)*cos(gamma) + cos(alpha)*sin(gamma);
		rTransform(1, 1) = -sin(alpha)*cos(beta)*sin(gamma) + cos(alpha)*cos(gamma);
        rTransform(1, 2) = sin(alpha)*sin(beta);
        rTransform(2, 0) = -sin(beta)*cos(gamma);
        rTransform(2, 1) = sin(beta)*sin(gamma);
        rTransform(2, 2) = cos(beta);

	}
	else if(!strcmp(str, "Estaubli"))
	{
		// direction cos (Rxyz Euler Angle)
		rTransform(0, 0) = cos(beta) * cos(gamma);
		rTransform(0, 1) = -cos(beta) * sin(gamma);
		rTransform(0, 2) = sin(beta);
		rTransform(1, 0) = sin(alpha) * sin(beta) * cos(gamma) + cos(alpha) * sin(gamma);
		rTransform(1, 1) = -sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
		rTransform(1, 2) = -sin(alpha) * cos(beta);
		rTransform(2, 0) = -cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
		rTransform(2, 1) = cos(alpha) * sin(beta) * sin(gamma) + sin(alpha) * cos(gamma);
		rTransform(2, 2) = cos(alpha) * cos(beta);
	}
	else {
		cout << "Not matched rotation!" << endl;
	}
	
}


void T_Robot::MakeRobotTransform(Vector3d Pos, Vector3d Ori, Matrix4d &rTransform)
{
	rTransform.setIdentity();
	
	double alpha, beta, gamma;
	alpha = Ori(0), beta = Ori(1), gamma = Ori(2);

	rTransform(0, 3) = Pos(0);	// x
	rTransform(1, 3) = Pos(1);	// y
	rTransform(2, 3) = Pos(2);	// z
	

	if(oriType == Exyz)	// ���ڰ� ��ġ�ϸ� 0�̹Ƿ� !�� �ٿ��ش�. 
	{
		// direction cos (Rxyz Euler Angle)
		rTransform(0, 0) = cos(beta) * cos(gamma);
		rTransform(0, 1) = -cos(beta) * sin(gamma);
		rTransform(0, 2) = sin(beta);
		rTransform(1, 0) = sin(alpha) * sin(beta) * cos(gamma) + cos(alpha) * sin(gamma);
		rTransform(1, 1) = -sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
		rTransform(1, 2) = -sin(alpha) * cos(beta);
		rTransform(2, 0) = -cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
		rTransform(2, 1) = cos(alpha) * sin(beta) * sin(gamma) + sin(alpha) * cos(gamma);
		rTransform(2, 2) = cos(alpha) * cos(beta);
	}
	else if(oriType == Ezyz)
	{
		// direction cos (Rzyz)
		rTransform(0, 0) = cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma);
        rTransform(0, 1) = -cos(alpha)*cos(beta)*sin(gamma) - sin(alpha)*cos(gamma);
        rTransform(0, 2) = cos(alpha)*sin(beta);
        rTransform(1, 0) = sin(alpha)*cos(beta)*cos(gamma) + cos(alpha)*sin(gamma);
		rTransform(1, 1) = -sin(alpha)*cos(beta)*sin(gamma) + cos(alpha)*cos(gamma);
        rTransform(1, 2) = sin(alpha)*sin(beta);
        rTransform(2, 0) = -sin(beta)*cos(gamma);
        rTransform(2, 1) = sin(beta)*sin(gamma);
        rTransform(2, 2) = cos(beta);

	}
	else if(oriType == Estaubli)
	{
		// direction cos (Rxyz Euler Angle)
		rTransform(0, 0) = cos(beta) * cos(gamma);
		rTransform(0, 1) = -cos(beta) * sin(gamma);
		rTransform(0, 2) = sin(beta);
		rTransform(1, 0) = sin(alpha) * sin(beta) * cos(gamma) + cos(alpha) * sin(gamma);
		rTransform(1, 1) = -sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
		rTransform(1, 2) = -sin(alpha) * cos(beta);
		rTransform(2, 0) = -cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
		rTransform(2, 1) = cos(alpha) * sin(beta) * sin(gamma) + sin(alpha) * cos(gamma);
		rTransform(2, 2) = cos(alpha) * cos(beta);
	}
}


// Jacobian ��� �Լ�
void T_Robot::CalcJacobian()
{
	// SetDH �Լ��� ȣ�� ���� �ʾ����� ������ �߻���Ű�� ����
	if(dof == 0) {
		cout << "DH parameter is not setup" << endl;
		return;
	}

	//Jacobian.resize(6, dof);
	VectorXd _q(dof);
	
	if(kinType == CRAIG) {
		Matrix4d *AT = new Matrix4d[dof];
	
		for(int i=0; i<dof; i++){
			AT[i] = T[i];
		}
		
		Vector3d *P = new Vector3d[dof];
		
		// Position difference
		for(int i=0; i<dof; i++){
			P[i].setZero();
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
			ep(0) = T0E(0,3)-P[i](0);	// End-effector�� �� Joint position���� �Ÿ�
			ep(1) = T0E(1,3)-P[i](1);
			ep(2) = T0E(2,3)-P[i](2);
	
			// z0 X [Pn-P0], z1 X [Pn-P1],... 
			tmp = z.cross(ep);
	
			Jacobian(0, i) = tmp(0);	// x
			Jacobian(1, i) = tmp(1);	// y
			Jacobian(2, i) = tmp(2);	// z
			Jacobian(3, i) = z(0);		// alpha
			Jacobian(4, i) = z(1);		// beta
			Jacobian(5, i) = z(2);		// gamma
		}

		delete AT;
		delete P;
	} 
	else if(kinType == PAUL) {
		Matrix4d iMat; iMat.setIdentity(4,4);
		Matrix4d *AT = new Matrix4d[dof+1];

		for(int i=0; i<dof+1; i++){
			if(i==0) {
				AT[i] = iMat;	// identity matrix
			} else {
				AT[i] = T[i-1];
			}
		}
	
		Vector3d *P = new Vector3d[dof+1];
	
		// Position difference
		for(int i=1; i<dof+1; i++){
			P[i].setZero();
			P[i](0) = AT[i](0,3)+x_offset;
			P[i](1) = AT[i](1,3)+y_offset;
			P[i](2) = AT[i](2,3)+z_offset;
		}

		Vector3d z, ep, tmp;
		for(int i=1; i<dof+1; i++){
			// z-axis
			z(0) = AT[i-1](0,2);
			z(1) = AT[i-1](1,2);
			z(2) = AT[i-1](2,2);

			// difference between end-effector pos and each joint pos
			ep(0) = P[dof](0)-P[i-1](0);
			ep(1) = P[dof](1)-P[i-1](1);
			ep(2) = P[dof](2)-P[i-1](2);

			tmp = z.cross(ep);

			Jacobian(0, i-1) = tmp(0);	// x
			Jacobian(1, i-1) = tmp(1);	// y
			Jacobian(2, i-1) = tmp(2);	// z
			Jacobian(3, i-1) = z(0);	// alpha
			Jacobian(4, i-1) = z(1);	// beta
			Jacobian(5, i-1) = z(2);	// gamma
		}

		delete AT;
		delete P;
	}
}


Vector3d T_Robot::GetEulerAngleRad(Matrix3d rotMat)
{
	Vector3d eAngle;

	//��� 2. Handbook of Robotics���� ã�� ��.. psi = alpha, theta = beta, phi = gamma
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

	return eAngle;
}


Vector3d T_Robot::GetOrientation_eXYZ(Matrix3d R)
{
		Vector3d rxryrz;
        
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
        */ // ���� ����� ������ �ذ��� ���� �Ʒ� ����̴�. ���� ��Ŀ����� Ư�� ������ǿ� �������� �� ������ �߻��Ѵ�.
        
        if ( R(0, 2) < 1){
               if ( R(0, 2) > -1) // -1< ... <1
               {
                       rxryrz[1] = asin(R(0, 2));
                       rxryrz[0] = atan2(-R(1, 2), R(2, 2));
                       rxryrz[2] = atan2(-R(0, 1), -R(0, 0));
               }
               else // <= -1 .. numerical problem���� -1���� �۰� ���� ���� ����
               {
                       // Not a unique solution : rxryrz_R[2] - rxryrz_R[0] = atan2 (r10 ,r11)
                       rxryrz[1] = -M_PI/2;
                       rxryrz[0] = -atan2(R(1, 0), R(1, 1));
                       rxryrz[2] = 0;
               }
        }
        else // 1 <= .. numerical problem���� 1���� ũ�� ���� ���� ����
        {
               // Not a unique solution : rxryrz_R[2] + rxryrz_R[0] = atan2 (r10 ,r11)
               rxryrz[1] = M_PI/2;
               rxryrz[0] = atan2(R(1, 0), R(1, 1));
               rxryrz[2] = 0;
        }

		return rxryrz;
}


Vector3d T_Robot::GetOrientation_eZYZ(Matrix3d R)
{
	Vector3d rxryrz;
        
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
    */ // ���� ����� ������ �ذ��� ���� �Ʒ� ����̴�. ���� ��Ŀ����� Ư�� ������ǿ� �������� �� ������ �߻��Ѵ�.

    if ( R(2, 2) < +1){
            if ( R(2, 2) > -1) // -1< ... <1
            {
                    rxryrz[1] = acos(R(2, 2));
                    rxryrz[0] = atan2(R(1, 2), R(0, 2));
                    rxryrz[2] = atan2(R(2, 1), -R(2, 0));
            }
            else // <= -1 .. numerical problem���� -1���� �۰� ���� ���� ����
            {
                    // Not a unique solution : rxryrz_R[2] - rxryrz_R[0] = atan2 (r10 ,r11)
                    rxryrz[1] = M_PI;
                    rxryrz[0] = -atan2(R(1, 0), R(1, 1));
                    rxryrz[2] = 0;
            }
    }
    else // 1 <= .. numerical problem���� 1���� ũ�� ���� ���� ����
    {
            // Not a unique solution : rxryrz_R[2] + rxryrz_R[0] = atan2 (r10 ,r11)
            rxryrz[1] = 0;
            rxryrz[0] = atan2(R(1, 0), R(1, 1));
            rxryrz[2] = 0;
    }

	return rxryrz;
}

