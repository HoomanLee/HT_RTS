
// HT_RTSDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "HT_RTSDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif



CString ipAddr[] = {"127.0.0.1", "129.254.90.68", "192.168.0.3", "192.168.0.4"};	// Taewoo PC, Hooman PC




// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CHT_RTSDlg 대화 상자



// Constructor
CHT_RTSDlg::CHT_RTSDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CHT_RTSDlg::IDD, pParent)
	, m_onConnect(FALSE)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_pwndShow = NULL;

	pNet_Thread = NULL;
}

void CHT_RTSDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TAB1, m_Tab);
	DDX_Control(pDX, IDC_COMBO_IPADDR, m_Combo_IPAddr);
	DDX_Control(pDX, IDC_CONNECT2SERVER, m_ButtonCon2Sev);
}

BEGIN_MESSAGE_MAP(CHT_RTSDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_NOTIFY(TCN_SELCHANGE, IDC_TAB1, &CHT_RTSDlg::OnTcnSelchangeTab1)
	ON_BN_CLICKED(IDOK, &CHT_RTSDlg::OnBnClickedOk)
	ON_CBN_SELCHANGE(IDC_COMBO_IPADDR, &CHT_RTSDlg::OnCbnSelchangeComboIpaddr)
	ON_BN_CLICKED(IDC_CONNECT2SERVER, &CHT_RTSDlg::OnBnClickedConnect2server)
END_MESSAGE_MAP()


// CHT_RTSDlg 메시지 처리기

BOOL CHT_RTSDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}	
	}

	// 이 대화 상자의 아이콘을 설정합니다. 응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.W
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.


	// App포인터에 Dlg 포인터 대입
	((CHT_RTSApp*)AfxGetApp())->pDlg = this;



	// Tab에 Bitmap icon 추가 코드
	/*CBitmap Bmp;
	Bmp.LoadBitmapW(IDB_BITMAP1);

	static CImageList ImgList;
	ImgList.Create(16,16,ILC_COLOR24 | ILC_MASK,7,0);
	ImgList.Add(&Bmp, RGB(192,192,192));
	m_Tab.SetImageList(&ImgList);*/

	const int nTab = 4;
	CString strTmp[nTab] = {_T("[F1] Main Control"),	// Main Control
							_T("[F2] Vision"),			// Vision
							_T("[F3] I/O"),				// I/O
							_T("[F4] Program")			// Program
														// and so on..add if you want to add another tab!
							};
	
	for(int i=0; i<nTab; i++)
	{
		m_Tab.InsertItem(i,strTmp[i],i);
	}
	


	CRect rect;
	m_Tab.GetClientRect(&rect);

	// 각 Tab을 만들때 속성 설정
	// : Boader를 None으로, Style을 Child로 설정할 것...
	m_MainControlTab.Create(IDD_MAINCONTROL, &m_Tab);
	m_MainControlTab.SetWindowPos(NULL, 5, 25, rect.Width()-10, rect.Height()-30, SWP_SHOWWINDOW | SWP_NOZORDER);
	
	m_VisionTab.Create(IDD_VISION, &m_Tab);
	m_VisionTab.SetWindowPos(NULL, 5, 25, rect.Width()-10, rect.Height()-30, SWP_NOZORDER);

	m_IOTab.Create(IDD_INPUT_OUTPUT, &m_Tab);
	m_IOTab.SetWindowPos(NULL, 5, 25, rect.Width()-10, rect.Height()-30, SWP_NOZORDER);

	m_ProgramTab.Create(IDD_PROGRAM, &m_Tab);
	m_ProgramTab.SetWindowPos(NULL, 5, 25, rect.Width()-10, rect.Height()-30, SWP_NOZORDER);

	
	m_MainControlTab.ShowWindow(SW_SHOW);
	m_pwndShow = &m_MainControlTab;	// main control tab이 처음에 보이도록 초기화
	m_MainControlTab.SetFocus();
	

	
	// IP Address ComboBox에 추가
	for(int i=0; i<sizeof(ipAddr)/sizeof(CString); i++)
	{
		m_Combo_IPAddr.AddString(ipAddr[i]);
	}
	m_IPAddr = ipAddr[m_Combo_IPAddr.SetCurSel(0)];
	

	// 통신 쓰레드 초기화
	/////// 소켓 통신을 위한 초기 설정
	if(!AfxSocketInit()) {
		AfxMessageBox("Windwos 소켓 초기화에 실패했습니다.");
	} else {
		// 서버와 통신하기위한 스레드 생성, 초기 멈춤 상태로 생성
		pThreadSend = AfxBeginThread((AFX_THREADPROC)CHT_RTSDlg::ThreadSendToServer, (LPVOID)this, 0, CREATE_SUSPENDED);
		pThreadRecv = AfxBeginThread((AFX_THREADPROC)CHT_RTSDlg::ThreadReceiveFromServer, (LPVOID)this, 0, CREATE_SUSPENDED);
		pThreadSend->SuspendThread();
		pThreadRecv->SuspendThread();
	}
	
	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CHT_RTSDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다. 문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CHT_RTSDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CHT_RTSDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


// 새로운 Tab을 선택했을때 발생하는 메시지
void CHT_RTSDlg::OnTcnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_pwndShow != NULL)
	{
		m_pwndShow->ShowWindow(SW_HIDE);
		m_pwndShow = NULL;
	}

	int nIndex = m_Tab.GetCurSel();
	switch(nIndex)
	{
	case 0:
		m_MainControlTab.ShowWindow(SW_SHOW);
		m_pwndShow = &m_MainControlTab;
		m_MainControlTab.SetFocus();
		break;
	case 1:
		m_VisionTab.ShowWindow(SW_SHOW);
		m_pwndShow = &m_VisionTab;
		m_VisionTab.SetFocus();
		break;
	case 2:
		m_IOTab.ShowWindow(SW_SHOW);
		m_pwndShow = &m_IOTab;
		m_IOTab.SetFocus();
		break;
	case 3:
		m_ProgramTab.ShowWindow(SW_SHOW);
		m_pwndShow = &m_ProgramTab;
		m_ProgramTab.SetFocus();
		break;
	}

	*pResult = 0;
}


BOOL CHT_RTSDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	// WM_KEYDOWN 먼저 인식
	if(pMsg->message == WM_KEYDOWN)
	{
		// 여기의 if문은 다이얼로그에서 Enter키와 ESC키를 누를 때
		// 창이 닫히는 것을 방지하기 위한 구문
		if(pMsg->wParam == VK_RETURN || pMsg->wParam == VK_ESCAPE)
		{
			return TRUE;
		}
		
		// Function 키 검사 후 tab 변경
		switch(pMsg->wParam)
		{
		case 112: 
			this->m_Tab.SetCurSel(0);	// 커서 설정
			NMHDR nmhdr;				// 통지 메시지
			nmhdr.code = TCN_SELCHANGE;
			nmhdr.idFrom = IDC_TAB1;
			nmhdr.hwndFrom = m_Tab.m_hWnd;
			SendMessage(WM_NOTIFY, IDC_TAB1, (LPARAM)&nmhdr);
			break;
		case 113: 
			this->m_Tab.SetCurSel(1);
			nmhdr.code = TCN_SELCHANGE;
			nmhdr.idFrom = IDC_TAB1;
			nmhdr.hwndFrom = m_Tab.m_hWnd;
			SendMessage(WM_NOTIFY, IDC_TAB1, (LPARAM)&nmhdr);
			break;
		case 114: 
			this->m_Tab.SetCurSel(2);
			nmhdr.code = TCN_SELCHANGE;
			nmhdr.idFrom = IDC_TAB1;
			nmhdr.hwndFrom = m_Tab.m_hWnd;
			SendMessage(WM_NOTIFY, IDC_TAB1, (LPARAM)&nmhdr);
			break;
		case 115: 
			this->m_Tab.SetCurSel(3);
			nmhdr.code = TCN_SELCHANGE;
			nmhdr.idFrom = IDC_TAB1;
			nmhdr.hwndFrom = m_Tab.m_hWnd;
			SendMessage(WM_NOTIFY, IDC_TAB1, (LPARAM)&nmhdr);
			break;
		}
		
	}

	return CDialogEx::PreTranslateMessage(pMsg);
}


void CHT_RTSDlg::OnBnClickedOk()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	CDialogEx::OnOK();
}


void CHT_RTSDlg::OnCbnSelchangeComboIpaddr()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int index = m_Combo_IPAddr.GetCurSel();
	m_IPAddr = ipAddr[index];
}


void CHT_RTSDlg::OnBnClickedConnect2server()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	//if(!m_onConnect) {
	//	m_Socket.CloseSocket();
	//	if(m_Socket.initClientSocket(m_IPAddr, 2311)){
	//		m_onConnect = TRUE;
	//		m_ButtonCon2Sev.SetWindowTextA("Disconnected");
	//	} else {
	//		AfxMessageBox("ERROR: Failed to connect server!");
	//	}
	//	
	//} else {
	//	m_Socket.CloseSocket();
	//	m_onConnect = FALSE;
	//	m_ButtonCon2Sev.SetWindowTextA("Connect to Server");
	//}

	char* _IPAddr = m_IPAddr.GetBuffer(m_IPAddr.GetLength());
	if(!theApp.T_SW->_tcpip.bConnected){
		if( theApp.T_SW->_tcpip.connect_HM(_IPAddr, POS_ORIDATA, 2000) == SOCK_CONNECT &&  
			theApp.T_SW->_tcpip.connect_TBtn(_IPAddr, 2012) == SOCK_CONNECT){
			theApp.T_SW->_tcpip.bConnected = true;
			
			//if ( pNet_Thread == NULL){
			//	pNet_Thread =  AfxBeginThread(T_Network, &theApp, 0, 0, CREATE_SUSPENDED);
			//}
			//pNet_Thread->ResumeThread();
			

			m_ButtonCon2Sev.SetWindowTextA("Disconnected");
			cout<<"Connected to server"<<endl;
		}
		else{
			AfxMessageBox("ERROR: Failed to connect to server!");
		}
	}
	else{
		//pNet_Thread->SuspendThread();
		theApp.T_SW->_tcpip.Net_Disconnect();
		theApp.T_SW->_tcpip.bConnected = false;
		m_ButtonCon2Sev.SetWindowTextA("Connect to Server");
	}	
}




//////////////////////////////////////////////
// 통신 쓰레드
UINT CHT_RTSDlg::T_Network(LPVOID _lParam)
{
	while(1)
	{
		//WaitForSingleObject(theApp.T_SW->m_omni.event_sequence, 10/*INFINITE*/); // event가 signaled 상태가 되기를 기다린다.

		//cout<<OMST->T_ND.temp_pos_ori[0]<<"  "<<OMST->T_ND.temp_pos_ori[1]<<"  "<<OMST->T_ND.temp_pos_ori[2]<<endl;
		//cout<<OMST->T_ND.temp_pos_ori[3]<<"  "<<OMST->T_ND.temp_pos_ori[4]<<"  "<<OMST->T_ND.temp_pos_ori[5]<<endl;

	
		///* send */
		///* playback일 때에는 같은 소켓이지만 joint 값을 보낸다 */
		//if(theApp.T_SW->_tcpip.bConnected){
		//	if(theApp.T_SW->T_ND.temp_pos_ori[6] == true){
		//		for(int i=0; i<6; i++)
		//			theApp.T_SW->T_ND.temp_pos_ori[i] = theApp.T_SW->des_q(i) * H_RtoD;
		//		theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
		//	}
		//	else
		//		theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
		//	/* receive */
		//	theApp.T_SW->_tcpip.RecvData(CONTAINER, theApp.T_SW->T_ND.recv_container); // 조금 더 많은 내용을 포함한 데이터를 받자.		

		//	for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
		//		theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->T_ND.recv_container[i+6] * H_DtoR);
		//}

		

		////비전 데이터는 테스트용으로 그냥 임의의 점(초기위치)을 주자.
		//OMST->T_ND.temp_vision[0] = OMST->T_ND.recv_vision[0];//357.17; // x
		//OMST->T_ND.temp_vision[1] = OMST->T_ND.recv_vision[1];//20.0;   // y
		//OMST->T_ND.temp_vision[2] = OMST->T_ND.recv_vision[2];//-216.3; // z
		//OMST->T_ND.temp_vision[3] = OMST->T_ND.recv_vision[3];//-180.0; // rz

		Sleep(0);
	}
}


// Send, static 함수이므로 포인터를 받아와야..

// PC에서 로봇에 move 명령을 내릴 땐 지속적으로 보낼 필요 없이 버튼 누를 때만 커맨드를 보내면 됨
// OMNI일 땐 continous하게 명령을 보내야 한다. 이를 구현할 것!!
UINT CHT_RTSDlg::ThreadSendToServer(LPVOID pParam)
{
	CHT_RTSDlg *dlg = (CHT_RTSDlg*)pParam;
	while(1)
	{
		if(dlg->m_onConnect && dlg->m_RobotPos->BtnEvent) {	// PC 제어
			dlg->m_Socket.SendCoordData(dlg->m_RobotPos);
			dlg->m_RobotPos->BtnEvent = FALSE;
		}else if(dlg->m_onConnect && dlg->m_RobotPos->inputmode == OMNI)		// OMNI 제어
		{
			dlg->m_Socket.SendCoordData(dlg->m_RobotPos);
		}
		Sleep(0);
	}
}


// Receive, 여기서 display update까지 구현
UINT CHT_RTSDlg::ThreadReceiveFromServer(LPVOID pParam)
{
	CHT_RTSDlg *dlg = (CHT_RTSDlg*)pParam;
	while(1)
	{
		if(dlg->m_onConnect) {
			//dlg->m_Socket.ReceiveData(
		}
		Sleep(0);
	}
}