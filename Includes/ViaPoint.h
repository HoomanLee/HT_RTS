#pragma once

#include "Precompiled.h" // ������ �ð� ������ ����..
#include "Robot.h"

#define nVP		1000	// TODO
#define DIM		6		// TODO


struct viapoint{

	double		p[DIM];			// x, y, z, rx, ry, rz
	Matrix4d	hMat;			// homogeneous Matrix
	VectorXd	j;				// joint
	char		hashNum[10];	// Hash number

	int			objV_ID;
	
	// ������
	viapoint() {
		for(int i=0; i<DIM; i++) p[i]=0.0f;

		hMat = Matrix4d::Zero();
		hMat(3,3) = 1.0;

		memset(hashNum, 0, 10 * sizeof(char));
		objV_ID = -1;
	}
};

// Via-Point control�� ���� Ŭ����
class CViaPoint
{
public:
	CViaPoint(void);
	~CViaPoint(void);

	void		initViaPointClass(T_Robot &robot);

	void		SetVal(UINT y, UINT x, double val);								// ���� ������ ����
	void		SetMat(UINT y, Matrix4d &hMat, VectorXd pVec, VectorXd joint);	// Homogeneous Matrix �� ���� ������ ����
	void		SetRobotVP(UINT y, T_Robot &robot, char* hash);
	
	/*@ Grid touch ������� via point ���� */
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
	BOOL		isEmpty();			// List�� ������� Ȯ���ϴ� �Լ�
	list<viapoint>	GetList();
	int			GetObjV_ID(UINT y);
	
	void		Convert2Matrix(MatrixXd &T_data, MatrixXd &T_jdata);	// Linked List ������ Matrix Ÿ������ ��ȯ
	void		ShowList();
	void		PushValArray(double *pVal);
	void		InsertPoint(UINT index);
	void		DeleteArray(UINT y);
	void		DeleteAllArray();

	int			nToolcnt;			// Tool ���� count. (hash table �о�� �� �ʿ���)
	int			nGripcnt;			// Tool ���� nGripcnt count. (hash table �о�� �� �ʿ���)
	int			nReleasecnt;		// Tool ���� nReleasecnt count. (hash table �о�� �� �ʿ���)
	

	int			hash_numindx;			// hash table �� �� ������
	int			hash_gripindx;			// hash table �߿��� �� ��°�� open���� close����.. 10�� �̻� ���ٰ� �����ϰ�..
	int			hash_releaseindx;	
	int			hash_gripindxV;			// hash table �߿��� �� ��°�� open���� close����. with Vision
	int			hash_releaseindxV;

	int			subhash_GRindx;

	VectorXd	vp_Gapproach_q;
	VectorXd	vp_Greach_q;
	VectorXd	vp_Gleave_q;

	VectorXd	vp_Rapproach_q;
	VectorXd	vp_Rreach_q;
	VectorXd	vp_Rleave_q;

	
	bool			onRecord;		// ���� ������ ���� flag

	/** Decode the via points */
	bool decode_hash_table();


private: 
	list<viapoint> list_vp;
	int index;
	int dof;


public:
	// TODO
	MatrixXd		traj;			// ���� ������ ���� matrix
	unsigned int	idx;			// ���� ������ ���� index

};
