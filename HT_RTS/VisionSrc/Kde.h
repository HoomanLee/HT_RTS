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
	CPoint	GetThresMeanCoordi(double thres);							// ��ü �̹������� threshold �� �̻��� point���� ��� ��ǥ ����
	CPoint	GetThresMeanCoordiClip(double thres, CPoint cp, int Area);	// Clip���� ������ threshold �� �̻��� point���� ��� ��ǥ ����
	CDib*	GetThresBinary(double thres);								// ��ü �̹������� threshold �� �̻��� point���� ���� �̹��� ����
	CDib*	GetThresBinaryClip(double thres, CPoint cp, int Area);		// Clip���� ������ threshold �� �̻��� point���� ���� �̹��� ����
	
	
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


