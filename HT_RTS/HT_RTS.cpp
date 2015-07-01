
// HT_RTS.cpp : 응용 프로그램에 대한 클래스 동작을 정의합니다.
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


// 유일한 CHT_RTSApp 개체입니다.

CHT_RTSApp theApp;


// CHT_RTSApp 생성

//int _rModel[] = {AMIRO_R, AMIRO_L};
int _rModel[] = {STAUBLI};


CHT_RTSApp::CHT_RTSApp()
{
	// 다시 시작 관리자 지원
	m_dwRestartManagerSupportFlags = AFX_RESTART_MANAGER_SUPPORT_RESTART;

	// TODO: 여기에 생성 코드를 추가합니다.
	// InitInstance에 모든 중요한 초기화 작업을 배치합니다.
	
	// TeachingSW 메모리 할당
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

// CHT_RTSApp 초기화

BOOL CHT_RTSApp::InitInstance()
{
	// 응용 프로그램 매니페스트가 ComCtl32.dll 버전 6 이상을 사용하여 비주얼 스타일을
	// 사용하도록 지정하는 경우, Windows XP 상에서 반드시 InitCommonControlsEx()가 필요합니다.
	// InitCommonControlsEx()를 사용하지 않으면 창을 만들 수 없습니다.
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// 응용 프로그램에서 사용할 모든 공용 컨트롤 클래스를 포함하도록
	// 이 항목을 설정하십시오.
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();


	AfxEnableControlContainer();

	// 대화 상자에 셸 트리 뷰 또는
	// 셸 목록 뷰 컨트롤이 포함되어 있는 경우 셸 관리자를 만듭니다.
	CShellManager *pShellManager = new CShellManager;

	// 표준 초기화
	// 이들 기능을 사용하지 않고 최종 실행 파일의 크기를 줄이려면
	// 아래에서 필요 없는 특정 초기화
	// 루틴을 제거해야 합니다.
	// 해당 설정이 저장된 레지스트리 키를 변경하십시오.
	// TODO: 이 문자열을 회사 또는 조직의 이름과 같은
	// 적절한 내용으로 수정해야 합니다.
	SetRegistryKey(_T("로컬 응용 프로그램 마법사에서 생성된 응용 프로그램"));

	CHT_RTSDlg dlg;
	m_pMainWnd = &dlg;
	INT_PTR nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// TODO: 여기에 [확인]을 클릭하여 대화 상자가 없어질 때 처리할
		//  코드를 배치합니다.
	}
	else if (nResponse == IDCANCEL)
	{
		// TODO: 여기에 [취소]를 클릭하여 대화 상자가 없어질 때 처리할
		//  코드를 배치합니다.
	}

	// 위에서 만든 셸 관리자를 삭제합니다.
	if (pShellManager != NULL)
	{
		delete pShellManager;
	}

	// 대화 상자가 닫혔으므로 응용 프로그램의 메시지 펌프를 시작하지 않고  응용 프로그램을 끝낼 수 있도록 FALSE를
	// 반환합니다.
	return FALSE;
}

int CHT_RTSApp::GetNumRobot()
{
	return numRobot;
}


BOOL CHT_RTSApp::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.

	if(pMsg->message == WM_KEYDOWN) {
		
		// Function 키 검사 후 tab 변경
		switch(pMsg->wParam)
		{
		case 112: 
			pDlg->m_Tab.SetCurSel(0);	// 커서 설정
			NMHDR nmhdr;				// 통지 메시지
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
