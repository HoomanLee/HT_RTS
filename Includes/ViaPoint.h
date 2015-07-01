#pragma once

#include "Precompiled.h" // 컴파일 시간 단축을 위해..
#include "Robot.h"

#define nVP		1000	// TODO
#define DIM		6		// TODO


struct viapoint{

	double		p[DIM];			// x, y, z, rx, ry, rz
	Matrix4d	hMat;			// homogeneous Matrix
	VectorXd	j;				// joint
	char		hashNum[10];	// Hash number

	int			objV_ID;
	
	// 생성자
	viapoint() {
		for(int i=0; i<DIM; i++) p[i]=0.0f;

		hMat = Matrix4d::Zero();
		hMat(3,3) = 1.0;

		memset(hashNum, 0, 10 * sizeof(char));
		objV_ID = -1;
	}
};

// Via-Point control을 위한 클래스
class CViaPoint
{
public:
	CViaPoint(void);
	~CViaPoint(void);

	void		initViaPointClass(T_Robot &robot);

	void		SetVal(UINT y, UINT x, double val);								// 원소 단위로 삽입
	void		SetMat(UINT y, Matrix4d &hMat, VectorXd pVec, VectorXd joint);	// Homogeneous Matrix 및 벡터 단위로 삽입
	void		SetRobotVP(UINT y, T_Robot &robot, char* hash);
	
	/*@ Grid touch 방식으로 via point 저장 */
	void		SetRobotVP_grid(UINT y, T_Robot *_nominal_robot, char* hash);

	void		SetIndex(int param);
	BOOL		SetChange(int idx1, int idx2);
	void		SetClear();
	void		SetHash(UINT y, char* hash);
	

	double		GetVal(UINT y, UINT x);
	int			GetNum();
	int			GetIndex();
	Matrix4d	GetHmatrix(UINT y);
	VectorXd	GetJoint(UINT y);
	char*		GetHash(UINT y);
	BOOL		isEmpty();			// List가 비었는지 확인하는 함수
	list<viapoint>	GetList();
	int			GetObjV_ID(UINT y);
	
	void		Convert2Matrix(MatrixXd &T_data, MatrixXd &T_jdata);	// Linked List 변수를 Matrix 타입으로 변환
	void		ShowList();
	void		PushValArray(double *pVal);
	void		InsertPoint(UINT index);
	void		DeleteArray(UINT y);
	void		DeleteAllArray();

	int			nToolcnt;			// Tool 조작 count. (hash table 읽어올 때 필요함)
	int			nGripcnt;			// Tool 조작 nGripcnt count. (hash table 읽어올 때 필요함)
	int			nReleasecnt;		// Tool 조작 nReleasecnt count. (hash table 읽어올 때 필요함)
	

	int			hash_numindx;			// hash table 총 몇 개인지
	int			hash_gripindx;			// hash table 중에서 몇 번째가 open인지 close인지.. 10개 이상 없다고 가정하고..
	int			hash_releaseindx;	
	int			hash_gripindxV;			// hash table 중에서 몇 번째가 open인지 close인지. with Vision
	int			hash_releaseindxV;

	int			subhash_GRindx;

	VectorXd	vp_Gapproach_q;
	VectorXd	vp_Greach_q;
	VectorXd	vp_Gleave_q;

	VectorXd	vp_Rapproach_q;
	VectorXd	vp_Rreach_q;
	VectorXd	vp_Rleave_q;

	
	bool			onRecord;		// 궤적 저장을 위한 flag

	/** Decode the via points */
	bool decode_hash_table();


private: 
	list<viapoint> list_vp;
	int index;
	int dof;


public:
	// TODO
	MatrixXd		traj;			// 궤적 저장을 위한 matrix
	unsigned int	idx;			// 궤적 저장을 위한 index

};
