// *************************************
// Kernel Density Estimation Fucntion
// written by t.w.kim
// *************************************


#include "StdAfx.h"
#include "Kde.h"
#include <math.h>

CKde::CKde(void)
{
	m_kdeMat = NULL;
	m_Kernel = NULL;
	m_width = 0;
	m_height = 0;
	m_maxVal = 0.0;


	static bool flag = false;	// TODO

	// 객체 생성 시 처음 한 번만 초기화
	if(m_Kernel == NULL && flag == false) {
		initGaussianMask(4);	// sigma: 4, size: 4*8+1 = 33 x 33
		flag = true;
	}
}


CKde::~CKde(void)
{
}


void CKde::initKDEmask(int width, int height, double sigma)
{
	memAlloc(width, height);
	initGaussianMask(sigma);
}

void CKde::releaseKDEMask()
{
	memDelete();
	delGaussianMask();
}


void CKde::memAlloc(int width, int height)
{
	if(m_kdeMat!=NULL) {
		memDelete();
		//delete m_kdeMat;
		//m_kdeMat = NULL;
	}

	if(width > 0 && height > 0 && m_kdeMat==NULL) {
		int i,j;
		m_width = width;
		m_height = height;

		// memory allocation of KDE matrix
		m_kdeMat = new double*[m_height];
		for(i=0; i<m_height; i++)
		{
			m_kdeMat[i] = new double[m_width];
			memset(m_kdeMat[i], 0, sizeof(double)*m_width);	// 0으로 초기화 
		}

	}
}


void CKde::memDelete()
{
	if(m_kdeMat != NULL) {
		for(int i=0; i<m_height; i++)
			delete [] m_kdeMat[i];
		delete [] m_kdeMat;

		m_kdeMat = NULL;
	}
}


void CKde::initGaussianMask(double sigma)
{
	register int i, j, x, y;
	

	// Generation of 1-dimensional Gaussian mask
	int dim = (int)max(3.0, 2*4*sigma+1.0);
	if(dim % 2 == 0) dim++;	// To make odd number size
	int dim2 = (int)dim/2;	// 버림 연산

	kw = dim;
	kh = dim;
	
	double s = 2.0*sigma*sigma;
	double sum = 0;

	// 메모리 할당...
	m_Kernel = new double*[kh];
	for(i=0; i<kh; i++)
	{
		m_Kernel[i] = new double[kw];
		memset(m_Kernel[i], 0, sizeof(double)*kw);
	}


	for(y=-dim2; y<=dim2; y++)
	for(x=-dim2; x<=dim2; x++)
	{
		m_Kernel[y+dim2][x+dim2] = (exp(-(x*x + y*y)/s))/(PI*s); 
		sum += m_Kernel[y+dim2][x+dim2];
	}

	// Normalize
	for(j=0; j<kh; j++)
	for(i=0; i<kw; i++)
		m_Kernel[j][i] /= sum;

	
	//// print
	//for(j=0; j<h; j++)
	//{
	//	for(i=0; i<w; i++)
	//	{
	//		printf("%lf ", m_Kernel[j][i]);
	//	}
	//	printf("\n");
	//}
	
}


void CKde::delGaussianMask()
{
	if(m_Kernel != NULL) {
		// 메모리 해제 -------------
		for(int i=0; i<kh; i++)
			delete [] m_Kernel[i];
		delete [] m_Kernel;
	}
	m_Kernel = NULL;
}



void CKde::setSumKDEmask(int x, int y)
{
	double** ptr = GetPtr();	// get KDE pointer

	register int i,j;
	int tx = x;
	int ty = y;

	// x, y 좌표를 kernel mask와 overlap했을 때의 좌측 상단 좌표로 옮김
	tx = tx - (int)(kw/2);
	ty = ty - (int)(kh/2);

	for(j=0; j<kh; j++)	// kernel size
	for(i=0; i<kw; i++)
	{
		x = tx + i;
		y = ty + j;
		if(x>=0 && x<m_width && y>=0 && y<m_height) {
			ptr[y][x] += m_Kernel[j][i];
		}
	}
}


// threshold 값 이상의 point들의 평균 좌표 리턴
CPoint CKde::GetThresMeanCoordi(double thres)
{
	CPoint thresMeanPoint;
	double **ptr = GetPtr();

	// find mean coordinate of values which above threshold
	register int i,j;
	int mx=0, my=0, cnt=1;
	for(j=0; j<m_height; j++)
	for(i=0; i<m_width; i++)
	{
		if(ptr[j][i] >= thres) {
			mx += i;
			my += j;
			cnt++;
		}
	}

	if(cnt > 1) cnt--;

	thresMeanPoint.x = (LONG)(mx/cnt);
	thresMeanPoint.y = (LONG)(my/cnt);

	return thresMeanPoint;
}


// Clip영역 내에서 threshold 값 이상의 point들의 평균 좌표 리턴
CPoint CKde::GetThresMeanCoordiClip(double thres, CPoint cp, int Area)
{
	CPoint thresMeanPoint;
	double **ptr = GetPtr();
	int half = (int)Area/2;

	// find mean coordinate of values which above threshold
	register int i,j;
	int mx=0, my=0, cnt=1;
	for(j=0; j<m_height; j++)
	for(i=0; i<m_width; i++)
	{
		int nx = abs(cp.x-i);
		int ny = abs(cp.y-j);

		if(ptr[j][i] >= thres && nx < half && ny < half) {
			mx += i;
			my += j;
			cnt++;
		}
	}

	if(cnt > 1) cnt--;

	thresMeanPoint.x = (LONG)(mx/cnt);
	thresMeanPoint.y = (LONG)(my/cnt);

	return thresMeanPoint;
}





// 전체 이미지에서 threshold 값 이상의 point들의 이진 이미지 리턴
CDib*	CKde::GetThresBinary(double thres)
{
	CDib* pDib = new CDib;
	pDib->Allocate(m_width, m_height, 8);
	pDib->SetGrayPalette();

	double **ptr = GetPtr();
	register int i,j;
	for(j=0; j<m_height; j++)
	for(i=0; i<m_width; i++)
	{
		if(ptr[j][i] >= thres) {
			*pDib->GetPointer(i,j) = 255;
		}
	}

	return pDib;
}


// Clip영역 내에서 threshold 값 이상의 point들의 이진 이미지 리턴
CDib*	CKde::GetThresBinaryClip(double thres, CPoint cp, int Area)
{
	CDib* pDib = new CDib;
	pDib->Allocate(m_width, m_height, 8);
	pDib->SetGrayPalette();

	double **ptr = GetPtr();
	int half = (int)Area/2;
	register int i,j;
	for(j=0; j<m_height; j++)
	for(i=0; i<m_width; i++)
	{
		int nx = abs(cp.x-i);
		int ny = abs(cp.y-j);

		if(ptr[j][i] >= thres && nx < half && ny < half) {
			*pDib->GetPointer(i,j) = 255;
		}
	}

	return pDib;
}









CPoint CKde::GetMaxCoordi()
{
	CPoint maxPoint;
	double** ptr = GetPtr();
	register int i,j;
	
	// find maximum value
	double maxval = 0.0;
	int mx=0, my=0;
	for(j=0; j<m_height; j++) {
		for(i=0; i<m_width; i++)
		{
			if(ptr[j][i] > maxval) {
				maxval = ptr[j][i];
				mx = i;	my = j;
			}
		}
	}

	maxPoint.x = mx;
	maxPoint.y = my;

	m_maxVal = maxval;

	return maxPoint;
}

double CKde::GetMaxValue()
{
	GetMaxCoordi();
	return m_maxVal;	
}


double CKde::GetValue(int x, int y)
{
	double** ptr = GetPtr();

	return ptr[y][x];
}