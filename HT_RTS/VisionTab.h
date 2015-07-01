#pragma once


#include "KinectConnection.h"
#include "imagedisp.h"
#include "stdafx.h"

//#include "Precompiled.h"
//#include <Dense> // ?? 여기서 하면 오류, stdafx에서 include하면 잘됨... why????

//using namespace Eigen;

#define TS		0.15		// saturation threshold
#define Ti		0.25		// intensity threshold

#define STZOFFSET 375+16	// Staubli 바닥에서 원점 좌표 까지의 길이 + 판 두께

#define RED		0
#define GREEN	1
#define BLUE	2
#define YELLOW	3
#define NUMBOX	4


// kinect frame 원점으로부터 카메라 중심 좌표까지의 x, y 방향 offset 값
// Image2World 변환 OFFSETX: 155, OFFSETY: 161+200
#define OFFSETX	388-4		// offset은 실측으로 잡아낸 오차
#define OFFSETY	240-27



enum VisionClass
{
	None,
	Color_box,
	Calibration,
	Test,
};

const CString VisionClass[] = {"None", "Color_box", "Calibration", "Test"};


// CVisionTab 대화 상자입니다.

class CVisionTab : public CDialogEx
{
	DECLARE_DYNAMIC(CVisionTab)

public:
	CVisionTab(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CVisionTab();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_VISION };


private:
	// 작업 쓰레드
	CWinThread* pThread;
	CWinThread* pThreadIP;


private:
	CDib*	pBufColor;
	CDib*	pBufDepth;
	CDib*	pColor;
	CDib*	pDepth;
	CDib*	pColorClip;
	CDib*	pDepthClip;
	
	UINT	m_visionMode;
	// intrincsic Matrix K
	MatrixXd K;

	// Staubli matching matrix
	MatrixXd M_st;

	UINT m_binThres;





public:
	CKinectConnection	kinect;
	CLargeText			m_FrameRate;
	CLargeText			m_FrameRateIP;
	CPoint				m_ClipCenter;
	CPoint				m_GlobalCenter;

	bool clip;
	
	// 임시 변수 ~~~~~
	bool m_flag;	// TODO
	CDib* pTestImage;	// TODO

	CImageDisp	m_DispRGB;
	CImageDisp	m_DispDepth;
	CImageDisp	m_imgProcess;
	CRect		m_ClipRect;
	CButton		m_ImgClipBtn;
	Coordi		m_cbWorldPoint[NUMBOX];	// World point of each color boxes, To send
	ViaPoint	m_calibPoint;
	

	static HANDLE m_updateEvent;
	static HANDLE m_GetCimgEvent;
	static HANDLE m_GetDimgEvent;
	static HANDLE m_Mutex;

	CRITICAL_SECTION cs;
	CCriticalSection g_CS;

	CEvent g_event;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.



private:
	static UINT ThreadUpdateKinectImage(LPVOID pParam);
	static UINT ThreadImageProcessing(LPVOID pParam);
	void CalcTime();
	void CalcTimeIP();

	DECLARE_MESSAGE_MAP()


public:
	void updateColorStream();
	void updateDepthStream();
	void doingImageProcess();

	CDib* GetColorImage();
	CDib* GetDepthImage();
	CDib* GetColorClipImage(RECT *pRect);
	CDib* GetDepthClipImage(RECT *pRect);

	//// Vision Algorithm implementation ----------------------
	CDib*			ColorBoxRecog(CDib* pImg);
	TargetObject	CellPhonePartRecog(CDib* pImg);

public:
	virtual BOOL OnInitDialog();
	afx_msg void OnBnClickedClipimage();
	
	
	CComboBox		m_Combo_VisionClass;
	afx_msg void	OnCbnSelchangeVisionclasscombo();
	afx_msg void	OnBnClickedDoCalib();
	CImageDisp		m_testView;
	CListCtrl		m_CalibList;
	

	UINT			GetVisionMode();
	
	afx_msg void OnBnClickedClearCalib();
	CSpinButtonCtrl m_SpinBin;
	afx_msg void OnDeltaposSpinBinarization(NMHDR *pNMHDR, LRESULT *pResult);
};




