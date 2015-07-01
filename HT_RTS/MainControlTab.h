#pragma once

#include "LargeText.h"
#include "afxcmn.h"
#include "afxwin.h"
#include "Light.h"


// 키보드 후킹을 위한 선언문들
#define DEF_DLL_NAME "KeyHook.dll"
#define DEF_HOOKSTART "HookStart"
#define DEF_HOOKSTOP "HookStop"

typedef void (*PFN_HOOKSTART) ();
typedef void (*PFN_HOOKSTOP) ();


// OpenGL class header in MFC controls...
#include "OpenGLControl.h"


// CMainControlTab 대화 상자입니다.

class CMainControlTab : public CDialogEx
{
	DECLARE_DYNAMIC(CMainControlTab)

public:
	CMainControlTab(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CMainControlTab();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_MAINCONTROL };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()


// Large Text 멤버변수
public:
	// Cartesian
	CLargeText		m_LabelX;
	CLargeText		m_LabelY;
	CLargeText		m_LabelZ;
	CLargeText		m_LabelRX;
	CLargeText		m_LabelRY;
	CLargeText		m_LabelRZ;

	CLargeText		m_WorldX;
	CLargeText		m_WorldY;
	CLargeText		m_WorldZ;
	CLargeText		m_WorldRX;
	CLargeText		m_WorldRY;
	CLargeText		m_WorldRZ;


	// Joint 1
	CLargeText		m_LabelJ1;
	CLargeText		m_LabelJ2;
	CLargeText		m_LabelJ3;
	CLargeText		m_LabelJ4;
	CLargeText		m_LabelJ5;
	CLargeText		m_LabelJ6;
	CLargeText		m_LabelJ7;
	CLargeText		m_LabelJ8;
	CLargeText		m_LabelJ9;
	
	CLargeText		m_Joint1;
	CLargeText		m_Joint2;
	CLargeText		m_Joint3;
	CLargeText		m_Joint4;
	CLargeText		m_Joint5;
	CLargeText		m_Joint6;
	CLargeText		m_Joint7;
	CLargeText		m_Joint8;
	CLargeText		m_Joint9;


	// Tool, Cartesian 2
	CLargeText		m_LabelTX;
	CLargeText		m_LabelTY;
	CLargeText		m_LabelTZ;
	CLargeText		m_LabelTRX;
	CLargeText		m_LabelTRY;
	CLargeText		m_LabelTRZ;

	CLargeText		m_ToolX;
	CLargeText		m_ToolY;
	CLargeText		m_ToolZ;
	CLargeText		m_ToolRX;
	CLargeText		m_ToolRY;
	CLargeText		m_ToolRZ;


	// Joint 2
	CLargeText		m_LabelJ1_2;
	CLargeText		m_LabelJ2_2;
	CLargeText		m_LabelJ3_2;
	CLargeText		m_LabelJ4_2;
	CLargeText		m_LabelJ5_2;
	CLargeText		m_LabelJ6_2;
	CLargeText		m_LabelJ7_2;
	CLargeText		m_LabelJ8_2;
	CLargeText		m_LabelJ9_2;
	
	CLargeText		m_Joint1_2;
	CLargeText		m_Joint2_2;
	CLargeText		m_Joint3_2;
	CLargeText		m_Joint4_2;
	CLargeText		m_Joint5_2;
	CLargeText		m_Joint6_2;
	CLargeText		m_Joint7_2;
	CLargeText		m_Joint8_2;
	CLargeText		m_Joint9_2;


	virtual BOOL OnInitDialog();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	



// OpenGL 멤버 변수
public:
	HGLRC	m_hRC;	// opengl handle to render context
	HDC		m_hDC;


private:
	COpenGLControl	m_oglWindow;
	int ListclickX;
	int ListclickY;

	//// Exo 
	static UINT T_getExoData(LPVOID _lParam);
	//// Omni
	static UINT T_getOmniData(LPVOID _lParam);
	//// Compute Algorithms (IK, ...)
	static UINT T_Compute(LPVOID _lParam);

public:

	// 각 Movement섹션에서 step 설정을 위한 변수들
	float m_WorldPosStep;
	float m_WorldOriStep;
	float m_JointStep;
	float m_JointStep2;
	float m_ToolPosStep;
	float m_ToolOriStep;

	//CViaPoint vp;	// Via-point control class instance

	CSpinButtonCtrl m_SpinWorldPosStep;
	CSpinButtonCtrl m_SpinWorldOriStep;
	CSpinButtonCtrl m_SpinJointStep;
	CSpinButtonCtrl m_SpinToolPosStep;
	CSpinButtonCtrl m_SpinToolOriStep;
	CSpinButtonCtrl m_SpinJointStep2;

	CListCtrl m_vpList;
	CListCtrl m_vpList2;
	CLight m_Light;
	CLight m_Light2;
	CLight m_RecordLED;

	afx_msg void OnDeltaposSpinWpstep(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnDeltaposSpinWostep(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnDeltaposSpinJstep(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnDeltaposSpinTpstep(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnDeltaposSpinTostep(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnBnClickedJ1plus();
	afx_msg void OnBnClickedJ1minus();
	afx_msg void OnBnClickedJ2plus();
	afx_msg void OnBnClickedJ2minus();
	afx_msg void OnBnClickedJ3plus();
	afx_msg void OnBnClickedJ3minus();
	afx_msg void OnBnClickedJ4plus();
	afx_msg void OnBnClickedJ4minus();
	afx_msg void OnBnClickedJ5plus();
	afx_msg void OnBnClickedJ5minus();
	afx_msg void OnBnClickedJ6plus();
	afx_msg void OnBnClickedJ6minus();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedXplus();
	afx_msg void OnBnClickedXminus();
	afx_msg void OnBnClickedYplus();
	afx_msg void OnBnClickedYminus();
	afx_msg void OnBnClickedZplus();
	afx_msg void OnBnClickedZminus();
	afx_msg void OnBnClickedRxplus();
	afx_msg void OnBnClickedRxminus();
	afx_msg void OnBnClickedRyplus();
	afx_msg void OnBnClickedRyminus();
	afx_msg void OnBnClickedRzplus();
	afx_msg void OnBnClickedRzminus();
	afx_msg void OnBnClickedOricheck();
	afx_msg void OnBnClickedLoadviapoint();
	afx_msg void OnBnClickedDrawviapoint();
	afx_msg void OnLvnItemchangedViapointlist(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedPlayback();
	afx_msg void OnBnClickedRunning();



	// Robot 초기화를 위한 함수(DH 설정 등...)
	void	initRobotModel();
	void	vpShowUpdate();
	void	vpHideUpdate();
	void	vpListUpdate(int index);


	afx_msg void OnBnClickedJ7plus();
	afx_msg void OnBnClickedJ7minus();
	afx_msg void OnBnClickedJ8plus();
	afx_msg void OnBnClickedJ8minus();
	afx_msg void OnBnClickedJ9plus();
	afx_msg void OnBnClickedJ9minus();
	afx_msg void OnDeltaposSpinJstep2(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedTxplus();
	afx_msg void OnBnClickedTxminus();
	afx_msg void OnBnClickedTyplus();
	afx_msg void OnBnClickedTyminus();
	afx_msg void OnBnClickedTzplus();
	afx_msg void OnBnClickedTzminus();
	afx_msg void OnBnClickedTrxplus();
	afx_msg void OnBnClickedTrxminus();
	afx_msg void OnBnClickedTryplus();
	afx_msg void OnBnClickedTryminus();
	afx_msg void OnBnClickedTrzplus();
	afx_msg void OnBnClickedTrzminus();
	afx_msg void OnBnClickedJ1plus2();
	afx_msg void OnBnClickedJ1minus2();
	afx_msg void OnBnClickedJ2plus2();
	afx_msg void OnBnClickedJ2minus2();
	afx_msg void OnBnClickedJ3plus2();
	afx_msg void OnBnClickedJ3minus2();
	afx_msg void OnBnClickedJ4plus2();
	afx_msg void OnBnClickedJ4minus2();
	afx_msg void OnBnClickedJ5plus2();
	afx_msg void OnBnClickedJ5minus2();
	afx_msg void OnBnClickedJ6plus2();
	afx_msg void OnBnClickedJ6minus2();
	afx_msg void OnBnClickedJ7plus2();
	afx_msg void OnBnClickedJ7minus2();
	afx_msg void OnBnClickedJ8plus2();
	afx_msg void OnBnClickedJ8minus2();
	afx_msg void OnBnClickedJ9plus2();
	afx_msg void OnBnClickedJ9minus2();
	afx_msg void OnBnClickedSaveviapoint();
	afx_msg void OnBnClickedAddvia();
	afx_msg void OnBnClickedUp();
	afx_msg void OnBnClickedUp2();
	afx_msg void OnBnClickedDeletevia();
	afx_msg void OnNMDblclkViapointlist(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedTeachhere();
	afx_msg void OnBnClickedDn();
	afx_msg void OnBnClickedDn2();
	afx_msg void OnBnClickedTeachhere2();
	afx_msg void OnBnClickedAddvia2();
	afx_msg void OnBnClickedDeletevia2();
	afx_msg void OnLvnItemchangedViapointlist2(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnNMDblclkViapointlist2(NMHDR *pNMHDR, LRESULT *pResult);

	afx_msg void OnBnClickedInitexo();
	afx_msg void OnBnClickedInitomni();
	afx_msg void OnBnClickedTrajRecord();
	
	afx_msg void OnBnClickedClearvia();
	afx_msg void OnBnClickedClearvia2();
	afx_msg void OnBnClickedTopview();
	afx_msg void OnBnClickedFrontview();
	afx_msg void OnBnClickedLeftview();
	afx_msg void OnBnClickedViewup();
	afx_msg void OnBnClickedViewdown();
	afx_msg void OnBnClickedViewleft();
	afx_msg void OnBnClickedViewright();
};
