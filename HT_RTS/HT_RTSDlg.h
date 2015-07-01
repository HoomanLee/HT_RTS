
// HT_RTSDlg.h : 헤더 파일
//

#include "stdafx.h"

//#pragma once
//#include "afxcmn.h"

// Tab header file include
#include "MainControlTab.h"
#include "IOTab.h"
#include "VisionTab.h"
#include "ProgramTab.h"
//#include "afxwin.h"
#include "ConnectSocket.h"


// CHT_RTSDlg 대화 상자
class CHT_RTSDlg : public CDialogEx
{
// 생성입니다.
public:
	CHT_RTSDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

	// Tab 추가
	CMainControlTab		m_MainControlTab;
	CIOTab				m_IOTab;
	CVisionTab			m_VisionTab;
	CProgramTab			m_ProgramTab;
	
	CWnd*				m_pwndShow;	// 현재 화면에 보일 윈도우의 포인터를 저장
	
	CString				m_IPAddr;
	bool				m_onConnect;

	// 로봇과 통신을 위한 스레드
	CWinThread* pThreadSend;
	CWinThread* pThreadRecv;
	
	
	CConnectSocket		m_Socket;
	Coordi				*m_RobotPos;	// 로봇의 현재 위치 정보 포함


private:
	static UINT ThreadSendToServer(LPVOID pParam);
	static UINT ThreadReceiveFromServer(LPVOID pParam);

	//// Net thread
	static UINT T_Network(LPVOID _lParam);
	CWinThread* pNet_Thread;


// 대화 상자 데이터입니다.
	enum { IDD = IDD_HT_RTS_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	CTabCtrl m_Tab;
	afx_msg void OnTcnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult);
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnBnClickedOk();
	afx_msg void OnCbnSelchangeComboIpaddr();
	afx_msg void OnBnClickedConnect2server();
	CComboBox m_Combo_IPAddr;
	CButton m_ButtonCon2Sev;
};
