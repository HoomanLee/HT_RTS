
// HT_RTSDlg.cpp : ���� ����
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "HT_RTSDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif



CString ipAddr[] = {"127.0.0.1", "129.254.90.68", "192.168.0.3", "192.168.0.4"};	// Taewoo PC, Hooman PC




// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

// �����Դϴ�.
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


// CHT_RTSDlg ��ȭ ����



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


// CHT_RTSDlg �޽��� ó����

BOOL CHT_RTSDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.

	// IDM_ABOUTBOX�� �ý��� ��� ������ �־�� �մϴ�.
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

	// �� ��ȭ ������ �������� �����մϴ�. ���� ���α׷��� �� â�� ��ȭ ���ڰ� �ƴ� ��쿡��
	//  �����ӿ�ũ�� �� �۾��� �ڵ����� �����մϴ�.
	SetIcon(m_hIcon, TRUE);			// ū �������� �����մϴ�.W
	SetIcon(m_hIcon, FALSE);		// ���� �������� �����մϴ�.

	// TODO: ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.


	// App�����Ϳ� Dlg ������ ����
	((CHT_RTSApp*)AfxGetApp())->pDlg = this;



	// Tab�� Bitmap icon �߰� �ڵ�
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

	// �� Tab�� ���鶧 �Ӽ� ����
	// : Boader�� None����, Style�� Child�� ������ ��...
	m_MainControlTab.Create(IDD_MAINCONTROL, &m_Tab);
	m_MainControlTab.SetWindowPos(NULL, 5, 25, rect.Width()-10, rect.Height()-30, SWP_SHOWWINDOW | SWP_NOZORDER);
	
	m_VisionTab.Create(IDD_VISION, &m_Tab);
	m_VisionTab.SetWindowPos(NULL, 5, 25, rect.Width()-10, rect.Height()-30, SWP_NOZORDER);

	m_IOTab.Create(IDD_INPUT_OUTPUT, &m_Tab);
	m_IOTab.SetWindowPos(NULL, 5, 25, rect.Width()-10, rect.Height()-30, SWP_NOZORDER);

	m_ProgramTab.Create(IDD_PROGRAM, &m_Tab);
	m_ProgramTab.SetWindowPos(NULL, 5, 25, rect.Width()-10, rect.Height()-30, SWP_NOZORDER);

	
	m_MainControlTab.ShowWindow(SW_SHOW);
	m_pwndShow = &m_MainControlTab;	// main control tab�� ó���� ���̵��� �ʱ�ȭ
	m_MainControlTab.SetFocus();
	

	
	// IP Address ComboBox�� �߰�
	for(int i=0; i<sizeof(ipAddr)/sizeof(CString); i++)
	{
		m_Combo_IPAddr.AddString(ipAddr[i]);
	}
	m_IPAddr = ipAddr[m_Combo_IPAddr.SetCurSel(0)];
	

	// ��� ������ �ʱ�ȭ
	/////// ���� ����� ���� �ʱ� ����
	if(!AfxSocketInit()) {
		AfxMessageBox("Windwos ���� �ʱ�ȭ�� �����߽��ϴ�.");
	} else {
		// ������ ����ϱ����� ������ ����, �ʱ� ���� ���·� ����
		pThreadSend = AfxBeginThread((AFX_THREADPROC)CHT_RTSDlg::ThreadSendToServer, (LPVOID)this, 0, CREATE_SUSPENDED);
		pThreadRecv = AfxBeginThread((AFX_THREADPROC)CHT_RTSDlg::ThreadReceiveFromServer, (LPVOID)this, 0, CREATE_SUSPENDED);
		pThreadSend->SuspendThread();
		pThreadRecv->SuspendThread();
	}
	
	return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
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

// ��ȭ ���ڿ� �ּ�ȭ ���߸� �߰��� ��� �������� �׸�����
//  �Ʒ� �ڵ尡 �ʿ��մϴ�. ����/�� ���� ����ϴ� MFC ���� ���α׷��� ��쿡��
//  �����ӿ�ũ���� �� �۾��� �ڵ����� �����մϴ�.

void CHT_RTSDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �������� �׸��ϴ�.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ����ڰ� �ּ�ȭ�� â�� ���� ���ȿ� Ŀ���� ǥ�õǵ��� �ý��ۿ���
//  �� �Լ��� ȣ���մϴ�.
HCURSOR CHT_RTSDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


// ���ο� Tab�� ���������� �߻��ϴ� �޽���
void CHT_RTSDlg::OnTcnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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
	// TODO: ���⿡ Ư��ȭ�� �ڵ带 �߰� ��/�Ǵ� �⺻ Ŭ������ ȣ���մϴ�.
	// WM_KEYDOWN ���� �ν�
	if(pMsg->message == WM_KEYDOWN)
	{
		// ������ if���� ���̾�α׿��� EnterŰ�� ESCŰ�� ���� ��
		// â�� ������ ���� �����ϱ� ���� ����
		if(pMsg->wParam == VK_RETURN || pMsg->wParam == VK_ESCAPE)
		{
			return TRUE;
		}
		
		// Function Ű �˻� �� tab ����
		switch(pMsg->wParam)
		{
		case 112: 
			this->m_Tab.SetCurSel(0);	// Ŀ�� ����
			NMHDR nmhdr;				// ���� �޽���
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	CDialogEx::OnOK();
}


void CHT_RTSDlg::OnCbnSelchangeComboIpaddr()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int index = m_Combo_IPAddr.GetCurSel();
	m_IPAddr = ipAddr[index];
}


void CHT_RTSDlg::OnBnClickedConnect2server()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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
// ��� ������
UINT CHT_RTSDlg::T_Network(LPVOID _lParam)
{
	while(1)
	{
		//WaitForSingleObject(theApp.T_SW->m_omni.event_sequence, 10/*INFINITE*/); // event�� signaled ���°� �Ǳ⸦ ��ٸ���.

		//cout<<OMST->T_ND.temp_pos_ori[0]<<"  "<<OMST->T_ND.temp_pos_ori[1]<<"  "<<OMST->T_ND.temp_pos_ori[2]<<endl;
		//cout<<OMST->T_ND.temp_pos_ori[3]<<"  "<<OMST->T_ND.temp_pos_ori[4]<<"  "<<OMST->T_ND.temp_pos_ori[5]<<endl;

	
		///* send */
		///* playback�� ������ ���� ���������� joint ���� ������ */
		//if(theApp.T_SW->_tcpip.bConnected){
		//	if(theApp.T_SW->T_ND.temp_pos_ori[6] == true){
		//		for(int i=0; i<6; i++)
		//			theApp.T_SW->T_ND.temp_pos_ori[i] = theApp.T_SW->des_q(i) * H_RtoD;
		//		theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
		//	}
		//	else
		//		theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
		//	/* receive */
		//	theApp.T_SW->_tcpip.RecvData(CONTAINER, theApp.T_SW->T_ND.recv_container); // ���� �� ���� ������ ������ �����͸� ����.		

		//	for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
		//		theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->T_ND.recv_container[i+6] * H_DtoR);
		//}

		

		////���� �����ʹ� �׽�Ʈ������ �׳� ������ ��(�ʱ���ġ)�� ����.
		//OMST->T_ND.temp_vision[0] = OMST->T_ND.recv_vision[0];//357.17; // x
		//OMST->T_ND.temp_vision[1] = OMST->T_ND.recv_vision[1];//20.0;   // y
		//OMST->T_ND.temp_vision[2] = OMST->T_ND.recv_vision[2];//-216.3; // z
		//OMST->T_ND.temp_vision[3] = OMST->T_ND.recv_vision[3];//-180.0; // rz

		Sleep(0);
	}
}


// Send, static �Լ��̹Ƿ� �����͸� �޾ƿ;�..

// PC���� �κ��� move ����� ���� �� ���������� ���� �ʿ� ���� ��ư ���� ���� Ŀ�ǵ带 ������ ��
// OMNI�� �� continous�ϰ� ����� ������ �Ѵ�. �̸� ������ ��!!
UINT CHT_RTSDlg::ThreadSendToServer(LPVOID pParam)
{
	CHT_RTSDlg *dlg = (CHT_RTSDlg*)pParam;
	while(1)
	{
		if(dlg->m_onConnect && dlg->m_RobotPos->BtnEvent) {	// PC ����
			dlg->m_Socket.SendCoordData(dlg->m_RobotPos);
			dlg->m_RobotPos->BtnEvent = FALSE;
		}else if(dlg->m_onConnect && dlg->m_RobotPos->inputmode == OMNI)		// OMNI ����
		{
			dlg->m_Socket.SendCoordData(dlg->m_RobotPos);
		}
		Sleep(0);
	}
}


// Receive, ���⼭ display update���� ����
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