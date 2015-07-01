#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include <memory.h>
using namespace std;

class hMatrix
{
public:
	int row;//����� �� ���� 
	int col;//����� �� ���� 
	double **M_pData;//����� �����͸� �����ϴ� ������
	hMatrix();//�⺻ ������
	hMatrix(int n, int m);//n*m ũ���� ����� �����ϴ� ������
	hMatrix(hMatrix &opr);//���� ������ 
	~hMatrix();

	hMatrix& hMatrix::transpose();
	hMatrix operator*(const hMatrix& opr) const;//���� ������ 
	hMatrix& operator=(const hMatrix& opr);
	int GetRow() const{return row;}
	int GetCol() const{return col;}
	double element(int n, int m) const;//(n,m) element�� ��
	void displayhMatrix();//��� ����ϴ� �Լ� 
	void SetElement(int n, int m, double value);//(n,m) element �� �����ϴ� �Լ�
	void ReadhMatrix();//�Լ��� ��Ҹ� �Է¹޾� ����� �������ִ� �Լ�
	void SET(int n, int m, double *val);//����� ������ �迭�� �ʱ�ȭ ���ִ� �Լ�
	hMatrix hMatrix::Set_T(hMatrix R, hMatrix T);	
	
};

#endif

