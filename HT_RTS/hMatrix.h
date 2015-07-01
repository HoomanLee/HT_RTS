#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include <memory.h>
using namespace std;

class hMatrix
{
public:
	int row;//행렬의 행 개수 
	int col;//행렬의 열 개수 
	double **M_pData;//행렬의 데이터를 저장하는 포인터
	hMatrix();//기본 생성자
	hMatrix(int n, int m);//n*m 크기의 행렬을 생성하는 생성자
	hMatrix(hMatrix &opr);//복사 생성자 
	~hMatrix();

	hMatrix& hMatrix::transpose();
	hMatrix operator*(const hMatrix& opr) const;//곱셈 연산자 
	hMatrix& operator=(const hMatrix& opr);
	int GetRow() const{return row;}
	int GetCol() const{return col;}
	double element(int n, int m) const;//(n,m) element의 값
	void displayhMatrix();//행렬 출력하는 함수 
	void SetElement(int n, int m, double value);//(n,m) element 값 설정하는 함수
	void ReadhMatrix();//함수의 요소를 입력받아 행렬을 설정해주는 함수
	void SET(int n, int m, double *val);//행렬을 임의의 배열로 초기화 해주는 함수
	hMatrix hMatrix::Set_T(hMatrix R, hMatrix T);	
	
};

#endif

