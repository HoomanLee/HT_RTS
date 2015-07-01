
// HT_RTSDlg.h : ��� ����
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


// CHT_RTSDlg ��ȭ ����
class CHT_RTSDlg : public CDialogEx
{
// �����Դϴ�.
public:
	CHT_RTSDlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

	// Tab �߰�
	CMainControlTab		m_MainControlTab;
	CIOTab				m_IOTab;
	CVisionTab			m_VisionTab;
	CProgramTab			m_ProgramTab;
	
	CWnd*				m_pwndShow;	// ���� ȭ�鿡 ���� �������� �����͸� ����
	
	CString				m_IPAddr;
	bool				m_onConnect;

	// �κ��� ����� ���� ������
	CWinThread* pThreadSend;
	CWinThread* pThreadRecv;
	
	
	CConnectSocket		m_Socket;
	Coordi				*m_RobotPos;	// �κ��� ���� ��ġ ���� ����


private:
	static UINT ThreadSendToServer(LPVOID pParam);
	static UINT ThreadReceiveFromServer(LPVOID pParam);

	//// Net thread
	static UINT T_Network(LPVOID _lParam);
	CWinThread* pNet_Thread;


// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_HT_RTS_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.


// �����Դϴ�.
protected:
	HICON m_hIcon;

	// ������ �޽��� �� �Լ�
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
