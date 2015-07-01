//#include "StdAfx.h"
#include "Precompiled.h" // 컴파일 시간 단축을 위해..
#include "ViaPoint.h"


CViaPoint::CViaPoint(void)
{
	index = -1;
	dof = 0;
	
	nToolcnt = 0;
	hash_numindx = 0;
	nReleasecnt = 0;
	nGripcnt = 0;
	hash_gripindx = 0;
	hash_releaseindx = 0;
	hash_gripindxV = 0;
	hash_releaseindxV = 0;

	subhash_GRindx = 0;

	onRecord = false;
}


CViaPoint::~CViaPoint(void)
{
}

// 설정된 로봇 모델을 이용하여 초기화 하기
void CViaPoint::initViaPointClass(T_Robot &robot)
{
	dof = robot.GetDOF();	// dof 설정

	// Matrix 및 index 초기화
	traj.resize(10000, robot.GetDOF());
	idx = 0;
}



//// List를 이용하여 구현하기 
void CViaPoint::SetVal(UINT y, UINT x, double val)
{
	if(y>GetNum() || x>DIM) {
		printf("[SetVal]wrong index \n");
		return;
	}

	// 원소가 없을 경우 하나 push
	if(GetNum() < y+1) {
		viapoint item;
		list_vp.push_back(item);
	}

	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }
	iterPos->p[x] = val;
}

void CViaPoint::SetMat(UINT y, Matrix4d &hMat, VectorXd pVec, VectorXd joint)
{
	if(pVec.size() != DIM || joint.size() != dof) {
		cout << "Wrong pos or joint size!" << endl;
		return;
	}

	// 원소가 없을 경우 하나 push
	if(GetNum() < y+1) {
		viapoint item;
		list_vp.push_back(item);
	}
	
	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	cout << y << endl;
	while(cnt<y) { ++iterPos; cnt++; }
	
	// position & orientation
	for(int i=0; i<DIM; i++) {
		iterPos->p[i] = pVec(i);
	}

	// joint
	iterPos->j.resize(dof);
	for(int i=0; i<dof; i++) {
		iterPos->j[i] = joint(i);
	}

	// Homogeneous Matrix
	iterPos->hMat = hMat;

}


// 현재 로봇 팔끝의 위치를 티칭 한다.
void CViaPoint::SetRobotVP(UINT y, T_Robot &robot, char* hash)
{
	int dof = robot.GetDOF();
	
	VectorXd _q, _pos;
	Matrix4d _hMat;
	_pos.resize(6);
	_q.resize(dof);

	


	if(_pos.size() != DIM || _q.size() != dof) {
		cout << "Wrong pos or joint size!" << endl;
		return;
	}
	
	// 원소가 없을 경우 하나 push
	if(GetNum() < y+1) {
		viapoint item;
		list_vp.push_back(item);
	}

	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }


	// open, openv or close, closev 일 경우 이전 via-point 사용
	if(hash == "Open" || hash == "OpenV" || hash == "Close" || hash == "CloseV") {
		--iterPos;
		_q = iterPos->j;
		for(int i=0; i<_pos.size(); i++)
			_pos(i) = iterPos->p[i];
		_hMat = iterPos->hMat;
		++iterPos;
	}else {
		_q = robot.GetQ() * RtoD;
		_pos = robot.GetPosNOri();
		_hMat = robot.GetTool();
	}
	
	// position & orientation
	for(int i=0; i<DIM; i++) {
		iterPos->p[i] = _pos(i);
	}

	// joint
	iterPos->j.resize(dof);
	for(int i=0; i<dof; i++) {
		iterPos->j[i] = _q(i);
	}

	// Homogeneous Matrix
	iterPos->hMat = _hMat;
	
	// Hash Number
	iterPos->hashNum[0] = '\0';		// 널문자로 버퍼 비우기
	strcpy(iterPos->hashNum, hash);
}

/** Grid touch 방식으로 via point 저장 */
void CViaPoint::SetRobotVP_grid(UINT y, T_Robot *_nominal_robot, char* hash)
{
	int dof = _nominal_robot->GetDOF();
	
	VectorXd _q, _pos;
	Matrix4d _hMat;
	_pos.resize(6);
	_q.resize(dof);	


	if(_pos.size() != DIM || _q.size() != dof) {
		cout << "Wrong pos or joint size!" << endl;
		return;
	}
	
	// 원소가 없을 경우 하나 push
	if(GetNum() < y+1) {
		viapoint item;
		list_vp.push_back(item);
	}

	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }


	// open, openv or close, closev 일 경우 이전 via-point 사용
	if(hash == "Open" || hash == "OpenV" || hash == "Close" || hash == "CloseV") {
		--iterPos;
		_q = iterPos->j;
		for(int i=0; i<_pos.size(); i++)
			_pos(i) = iterPos->p[i];
		_hMat = iterPos->hMat;
		++iterPos;
	}else {
		//터치일 때에는 이 부분을 어떻게 해 줄지 고민이 필요하다. 지금은 정보가 없으므로 zero로..
		_q = _nominal_robot->GetQ() * RtoD;
		_pos = _nominal_robot->GetPosNOri();
		_hMat.setZero() = _nominal_robot->GetTool();
	}
	
	// position & orientation
	for(int i=0; i<DIM; i++) {
		iterPos->p[i] = _pos(i);
	}

	// joint
	iterPos->j.resize(dof);
	for(int i=0; i<dof; i++) {
		iterPos->j[i] = _q(i);
	}

	// Homogeneous Matrix
	iterPos->hMat = _hMat;
	
	// Hash Number
	iterPos->hashNum[0] = '\0';		// 널문자로 버퍼 비우기
	strcpy(iterPos->hashNum, hash);
}

void CViaPoint::SetHash(UINT y, char* hash)
{
	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }

	iterPos->hashNum[0] = '\0';
	strcpy(iterPos->hashNum, hash);
}

void CViaPoint::SetIndex(int param)
{
	index = param;
}


BOOL CViaPoint::SetChange(int idx1, int idx2)
{
	if(idx1 < 0 || idx1 >= GetNum() || idx2 < 0 || idx2 >= GetNum())
	{
		cout << "out of index" << endl;
		return FALSE;
	}

	double temp[DIM];
	list<viapoint>::iterator iterPos1 = list_vp.begin();
	int cnt = 0;
	while(cnt<idx1) { ++iterPos1; cnt++; }

	list<viapoint>::iterator iterPos2 = list_vp.begin();
	cnt = 0;
	while(cnt<idx2) { ++iterPos2; cnt++; }

	
	std::swap(*iterPos1, *iterPos2);	// *iter 로 지정해 줘야 iterator가 가리키는 원소 끼리 바뀐다!!!
	
	
	return TRUE;
}

void CViaPoint::SetClear()
{
	list<viapoint>::iterator iterPos = list_vp.begin();
	list_vp.erase(iterPos, list_vp.end());
}



void CViaPoint::PushValArray(double *pVal)
{
	viapoint item;
	Matrix4d H = Matrix4d::Identity();
	
	int dim = 12+dof;
	int i; 
	
	// make H matrix
	for(i=0; i<12; i++)
	{
		int y = i%3;	// mod
		int x = floor((double)i/3);
		item.hMat(y, x) = pVal[i];
	}

	// copy to joint variable
	item.j.resize(dof);
	for(i; i<dim; i++)
	{
		item.j(i-12) = pVal[i];
	}
	
	// cartesian position
	for(i=0; i<3; i++) item.p[i] = item.hMat(i,3);		// position
	Vector3d ori;
	T_Robot rb;
	ori = rb.GetOrientation_eZYZ(item.hMat.topLeftCorner(3,3));	// TODO!!!
	for(i=0; i<3; i++) item.p[i+3] = ori(i)*RtoD;		// orientation

	list_vp.push_back(item);
}


void CViaPoint::DeleteArray(UINT y)
{
	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }
	list_vp.erase(iterPos);
}

void CViaPoint::DeleteAllArray()
{
	list_vp.clear();
}



void CViaPoint::InsertPoint(UINT index)
{
	if(index < 0 || index > GetNum()) {
		printf("Wrong index!, index: %d, GetNum: %d \n", index, GetNum());
		return;
	}

	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<index) { ++iterPos; cnt++; }

	viapoint v;
	list_vp.insert(iterPos, v);
}


double CViaPoint::GetVal(UINT y, UINT x)
{
	if(y>GetNum() || x>DIM) {
		printf("[Getval]wrong index \n");
	}

	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }
	return iterPos->p[x];
}

Matrix4d CViaPoint::GetHmatrix(UINT y)
{
	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }

	return iterPos->hMat;
}

VectorXd CViaPoint::GetJoint(UINT y)
{
	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }

	return iterPos->j;
}

char* CViaPoint::GetHash(UINT y)
{
	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }
	
	return iterPos->hashNum;
}

int CViaPoint::GetNum()
{
	return list_vp.size();
}

int CViaPoint::GetIndex()
{
	return index;
}

BOOL CViaPoint::isEmpty()
{
	if(list_vp.empty())
	{
		//cout << "empty! " << endl;
		return TRUE;
	}else
	{
		//cout << "not empty! " << endl;
		return FALSE;
	}
}


void CViaPoint::ShowList()
{
	for(list<viapoint>::iterator iterPos = list_vp.begin(); iterPos != list_vp.end(); ++iterPos)
	{
		// position & orientation
		for(int i=0; i<DIM; i++)
		{
			printf("%.2lf ", iterPos->p[i]);
		}

		// joint
		for(int i=0; i<dof; i++)
		{
			printf("%.2lf ", iterPos->j[i]);
		}
		printf("\n");

		// H matrix
		cout << iterPos->hMat << endl;

		// Hash information
		cout << iterPos->hashNum << endl;
	}
	
	printf("nRow: %d \n", GetNum());
}

list<viapoint> CViaPoint::GetList()
{
	return list_vp;
}

int CViaPoint::GetObjV_ID(UINT y)
{
	list<viapoint>::iterator iterPos = list_vp.begin();
	int cnt = 0;
	while(cnt<y) { ++iterPos; cnt++; }
	
	return iterPos->objV_ID;
}

bool CViaPoint::decode_hash_table()
{
	// To do... _gripindx와 _releaseindx의 위치만 찾아낸다?
	int indx = 0;

	for(list<viapoint>::iterator iterPos = list_vp.begin(); iterPos != list_vp.end(); ++iterPos)
	{
		// Hash information
		indx++;
		if(!strcmp(iterPos->hashNum,"Open")){
			cout<<"Find Open!!!"<<endl;
			hash_releaseindx = indx;
			nReleasecnt++;
		}
		if(!strcmp(iterPos->hashNum,"Close")){
			cout<<"Find Close!!!"<<endl;
			hash_gripindx = indx;
			nGripcnt++;
		}
		if(!strcmp(iterPos->hashNum,"OpenV")){
			cout<<"Find Open with Vison!!!"<<endl;
			hash_releaseindxV = indx;
			nReleasecnt++;
		}
		if(!strcmp(iterPos->hashNum,"CloseV")){
			cout<<"Find Close with Vison!!!"<<endl;
			hash_gripindxV = indx;
			nGripcnt++;
		}
		hash_numindx = indx;
	}
	cout<<"Hash table decoded!"<<endl;
	return true;
}


void CViaPoint::Convert2Matrix(MatrixXd &T_data, MatrixXd &T_jdata)	// Linked List 변수를 Matrix 타입으로 변환
{
	
	T_data.resize(GetNum(), DIM);
	T_jdata.resize(GetNum(), dof);
	
	T_data.setZero();
	T_jdata.setZero();
	
	int row = 0;
	for(list<viapoint>::iterator iterPos = list_vp.begin(); iterPos != list_vp.end(); ++iterPos)
	{
		// position & orientation
		for(int i=0; i<DIM; i++)
		{
			T_data(row, i) = iterPos->p[i];
		}

		// joint
		for(int i=0; i<dof; i++)
		{
			T_jdata(row, i) = iterPos->j[i];
		}

		row++;
	}

	/*cout << T_data << endl;
	cout << T_jdata << endl;*/
}