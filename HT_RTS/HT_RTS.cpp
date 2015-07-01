
// HT_RTS.cpp : ���� ���α׷��� ���� Ŭ���� ������ �����մϴ�.
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "HT_RTSDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CHT_RTSApp

BEGIN_MESSAGE_MAP(CHT_RTSApp, CWinApp)
	//ON_COMMAND(ID_HELP, &CWinApp::OnHelp)
END_MESSAGE_MAP()


// ������ CHT_RTSApp ��ü�Դϴ�.

CHT_RTSApp theApp;


// CHT_RTSApp ����

//int _rModel[] = {AMIRO_R, AMIRO_L};
int _rModel[] = {STAUBLI};


CHT_RTSApp::CHT_RTSApp()
{
	// �ٽ� ���� ������ ����
	m_dwRestartManagerSupportFlags = AFX_RESTART_MANAGER_SUPPORT_RESTART;

	// TODO: ���⿡ ���� �ڵ带 �߰��մϴ�.
	// InitInstance�� ��� �߿��� �ʱ�ȭ �۾��� ��ġ�մϴ�.
	
	// TeachingSW �޸� �Ҵ�
	T_SW = new TeachingSW_HM;

	numRobot = sizeof(_rModel)/sizeof(int);

	rModel = _rModel;
	
	T_SW->m_robot = new Robot[numRobot];
	T_SW->nominal_robot = new Robot[numRobot];
	T_SW->m_vp = new ViaPoint[numRobot];

}

CHT_RTSApp::~CHT_RTSApp()
{
	//if(T_SW)
	//	delete T_SW;
}

// CHT_RTSApp �ʱ�ȭ

BOOL CHT_RTSApp::InitInstance()
{
	// ���� ���α׷� �Ŵ��佺Ʈ�� ComCtl32.dll ���� 6 �̻��� ����Ͽ� ���־� ��Ÿ����
	// ����ϵ��� �����ϴ� ���, Windows XP �󿡼� �ݵ�� InitCommonControlsEx()�� �ʿ��մϴ�.
	// InitCommonControlsEx()�� ������� ������ â�� ���� �� �����ϴ�.
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// ���� ���α׷����� ����� ��� ���� ��Ʈ�� Ŭ������ �����ϵ���
	// �� �׸��� �����Ͻʽÿ�.
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();


	AfxEnableControlContainer();

	// ��ȭ ���ڿ� �� Ʈ�� �� �Ǵ�
	// �� ��� �� ��Ʈ���� ���ԵǾ� �ִ� ��� �� �����ڸ� ����ϴ�.
	CShellManager *pShellManager = new CShellManager;

	// ǥ�� �ʱ�ȭ
	// �̵� ����� ������� �ʰ� ���� ���� ������ ũ�⸦ ���̷���
	// �Ʒ����� �ʿ� ���� Ư�� �ʱ�ȭ
	// ��ƾ�� �����ؾ� �մϴ�.
	// �ش� ������ ����� ������Ʈ�� Ű�� �����Ͻʽÿ�.
	// TODO: �� ���ڿ��� ȸ�� �Ǵ� ������ �̸��� ����
	// ������ �������� �����ؾ� �մϴ�.
	SetRegistryKey(_T("���� ���� ���α׷� �����翡�� ������ ���� ���α׷�"));

	CHT_RTSDlg dlg;
	m_pMainWnd = &dlg;
	INT_PTR nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// TODO: ���⿡ [Ȯ��]�� Ŭ���Ͽ� ��ȭ ���ڰ� ������ �� ó����
		//  �ڵ带 ��ġ�մϴ�.
	}
	else if (nResponse == IDCANCEL)
	{
		// TODO: ���⿡ [���]�� Ŭ���Ͽ� ��ȭ ���ڰ� ������ �� ó����
		//  �ڵ带 ��ġ�մϴ�.
	}

	// ������ ���� �� �����ڸ� �����մϴ�.
	if (pShellManager != NULL)
	{
		delete pShellManager;
	}

	// ��ȭ ���ڰ� �������Ƿ� ���� ���α׷��� �޽��� ������ �������� �ʰ�  ���� ���α׷��� ���� �� �ֵ��� FALSE��
	// ��ȯ�մϴ�.
	return FALSE;
}

int CHT_RTSApp::GetNumRobot()
{
	return numRobot;
}


BOOL CHT_RTSApp::PreTranslateMessage(MSG* pMsg)
{
	// TODO: ���⿡ Ư��ȭ�� �ڵ带 �߰� ��/�Ǵ� �⺻ Ŭ������ ȣ���մϴ�.

	if(pMsg->message == WM_KEYDOWN) {
		
		// Function Ű �˻� �� tab ����
		switch(pMsg->wParam)
		{
		case 112: 
			pDlg->m_Tab.SetCurSel(0);	// Ŀ�� ����
			NMHDR nmhdr;				// ���� �޽���
			nmhdr.code = TCN_SELCHANGE;
			nmhdr.idFrom = IDC_TAB1;
			nmhdr.hwndFrom = pDlg->m_Tab.m_hWnd;
			SendMessage(pDlg->m_hWnd, WM_NOTIFY, IDC_TAB1, (LPARAM)&nmhdr);
			break;
		case 113: 
			pDlg->m_Tab.SetCurSel(1);
			nmhdr.code = TCN_SELCHANGE;
			nmhdr.idFrom = IDC_TAB1;
			nmhdr.hwndFrom = pDlg->m_Tab.m_hWnd;
			SendMessage(pDlg->m_hWnd, WM_NOTIFY, IDC_TAB1, (LPARAM)&nmhdr);
			break;
		case 114: 
			pDlg->m_Tab.SetCurSel(2);
			nmhdr.code = TCN_SELCHANGE;
			nmhdr.idFrom = IDC_TAB1;
			nmhdr.hwndFrom = pDlg->m_Tab.m_hWnd;
			SendMessage(pDlg->m_hWnd, WM_NOTIFY, IDC_TAB1, (LPARAM)&nmhdr);
			break;
		case 115: 
			pDlg->m_Tab.SetCurSel(3);
			nmhdr.code = TCN_SELCHANGE;
			nmhdr.idFrom = IDC_TAB1;
			nmhdr.hwndFrom = pDlg->m_Tab.m_hWnd;
			SendMessage(pDlg->m_hWnd, WM_NOTIFY, IDC_TAB1, (LPARAM)&nmhdr);
			break;
		}
	}


	return CWinApp::PreTranslateMessage(pMsg);
}
