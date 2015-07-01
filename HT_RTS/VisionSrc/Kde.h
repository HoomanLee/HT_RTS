// *************************************
// Kernel Density Estimation Fucntion
// written by t.w.kim
// *************************************

#pragma once
class CKde
{
public:
	CKde(void);
	~CKde(void);

	void	memAlloc(int width, int height);
	void	memDelete();
	void	initGaussianMask(double sigma);
	void	delGaussianMask();
	void	initKDEmask(int width, int height, double sigma);
	void	releaseKDEMask();

	void	setSumKDEmask(int x, int y);
	CPoint	GetMaxCoordi();
	double	GetMaxValue();
	double	GetValue(int x, int y);
	CPoint	GetThresMeanCoordi(double thres);							// 전체 이미지에서 threshold 값 이상의 point들의 평균 좌표 리턴
	CPoint	GetThresMeanCoordiClip(double thres, CPoint cp, int Area);	// Clip영역 내에서 threshold 값 이상의 point들의 평균 좌표 리턴
	CDib*	GetThresBinary(double thres);								// 전체 이미지에서 threshold 값 이상의 point들의 이진 이미지 리턴
	CDib*	GetThresBinaryClip(double thres, CPoint cp, int Area);		// Clip영역 내에서 threshold 값 이상의 point들의 이진 이미지 리턴
	
	
public:
	inline LONG Width();
	inline LONG Height();
	inline double** GetPtr();

private:
	double** m_kdeMat;
	int m_width;
	int m_height;
	double** m_Kernel;
	int kw;
	int kh;
	double m_maxVal;

};


// Define inline function
inline LONG CKde::Width()
{
        return m_width;
}


inline LONG CKde::Height()
{
        return m_height;
}


inline double** CKde::GetPtr()
{
	if(m_kdeMat == NULL) return NULL;

	return m_kdeMat;
}


