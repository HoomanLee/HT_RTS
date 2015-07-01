
#include "StdAfx.h"
#include "OpenGLControl.h"
#include "glut.h"


#include "HT_RTSDlg.h"

COpenGLControl::COpenGLControl(void)
{
	m_fPosX = 0.0f;		// X position of model in camera view
	m_fPosY = -150.0f;		// Y position of model in camera view
	m_fZoom = 1000.0f;	// Zoom on model in camera view
	m_fRotX = 0.0f;		// Rotation on model in camera view
	m_fRotY	= 0.0f;		// Rotation on model in camera view
	m_bIsMaximized = false;
	m_AltOn = FALSE;
	m_ShiftOn = FALSE;

	// desired view position
	m_desfPosX = 0.0f;
	m_desfPosY = 0.0f;
	m_desfZoom = 10.0f;
	m_desfRotX = m_fRotX;
	m_desfRotY = m_fRotY;
	m_gain = 0.15;
	m_thres = 0.001;
	m_viewFlag = FALSE;
	m_viaShow = FALSE;
	angle = 0.0f;

	// for vT
	//worldPos.setZero();
	worldPos = VectorXd::Zero(6);
	worldPos_grid.setZero();
	m_gap = 0;
	Teachstep = 0;
	Teaching_data_grid.resize(6);
	Teaching_data_grid.setZero();


	setInitialView();
	TextIntialize();	// TODO
	
	//// TeachingSW �޸� �Ҵ�
	//T_SW = new TeachingSW_HM;

	////// ***************** Robot �ʱ�ȭ ********************** //
	////// �κ� ���� �����ϱ� ���� �ǵ���� �ϴ� �Ķ���͵�
	////// 1: rModel (���ʺ��� NUM_ROBOT��ŭ �����)
	////// 2: SetinitGoal �Լ��� initX ����

	////int rModel[] = {AMIRO_R, AMIRO_L};
	//int rModel[] = {STAUBLI};
	//numRobot = sizeof(rModel)/sizeof(int);
	
	//T_SW->m_robot = new Robot[numRobot];
	//T_SW->m_vp = new ViaPoint[numRobot];
	onProcessing = new BOOL[theApp.GetNumRobot()];
	goal_T = new Matrix4d[theApp.GetNumRobot()];
	selectedVP = new int[theApp.GetNumRobot()];		// ���õ� ������(via point)

	int rType[10];

	for(int i=0; i<theApp.GetNumRobot(); i++)
	{
		rType[i] = theApp.rModel[i];
		RobotSetup(rType[i], theApp.T_SW->m_robot[i]);
		RobotSetup(rType[i], theApp.T_SW->nominal_robot[i]);  //�ʿ��� �� IK ���� Ǯ�� ���� nominal system�� �Բ� setup
		onProcessing[i] = FALSE;
		goal_T[i].setZero();
		selectedVP[i] = -1;
	}
	
	onIKstart = FALSE;

	m_RobotPos.des_WX = VectorXd::Zero(6);	// desired world coordinate
	m_RobotPos.des_TX = VectorXd::Zero(6);	// desired tool coordinate
}


COpenGLControl::~COpenGLControl(void)
{
	if(onProcessing)
		delete onProcessing;
	if(selectedVP)
		delete selectedVP;
	if(goal_T)
		delete goal_T;
}

void COpenGLControl::RobotSetup(int robot, T_Robot &_m_Robot)
{
	if(robot == AMIRO_R)
	{
		cout << "AMIRO Right Arm Setup!" << endl;
		//// Set DH parameter for AMIRO Right Arm -----------------------------------------------------
		int dof = 9;
		VectorXd a, alpha, d, theta;
		a.resize(dof); alpha.resize(dof); d.resize(dof); theta.resize(dof);

		//// ���θ�ī ver
		// Torso
		a(0) = 0.0,			alpha(0) = 0.0,			d(0) = 570.0,		theta(0) = 0.0;
		a(1) = 0.0,			alpha(1) = M_PI/2,		d(1) = 0.0,		theta(1) = M_PI/2;

		// Right Arm
		a(2) = 400.0,		alpha(2) =  M_PI,		d(2) = 208.0,			theta(2) =  M_PI;
		a(3) = 30.0,		alpha(3) = -M_PI/2,		d(3) = 0.0,				theta(3) = -M_PI/2;
		a(4) = 30.0,		alpha(4) = -M_PI/2,		d(4) = 330.0,/*300*/	theta(4) =  M_PI;
		a(5) = 32.0,		alpha(5) = -M_PI/2,		d(5) = 0.0,				theta(5) =  M_PI;
		a(6) = 32.0,		alpha(6) = -M_PI/2,		d(6) = 330.0,/*300*/	theta(6) =  0.0;
		a(7) = 0.0,			alpha(7) =  M_PI/2,		d(7) = 0.0,				theta(7) =  M_PI/2;
		a(8) = 95.0,		alpha(8) = -M_PI/2,		d(8) = 0.0,				theta(8) =  0.0;

		_m_Robot.SetDH(a, alpha, d, theta, dof, CRAIG);
		_m_Robot.SetPreH(0.0, 0.0, 0.0, M_PI);
		_m_Robot.SetOriType("Ezyz");

		// Tool DH (a, alpha, d, theta)
		_m_Robot.SetToolDH(0.0, 0.0, 0.0, 0.0, TRUE);
		_m_Robot.SetPostH(0,0,1,145.5,  0,-1,0,0,  1,0,0,0,  0,0,0,0);
		_m_Robot.CalcFK();
	}
	else if(robot == AMIRO_L)
	{
		cout << "AMIRO Left Arm Setup!" << endl;
		//// Set DH parameter for AMIRO Left Arm -----------------------------------------------------
		int dof = 9;
		VectorXd a, alpha, d, theta;
		a.resize(dof); alpha.resize(dof); d.resize(dof); theta.resize(dof);

		//// ���θ�ī ver
		// Torso
		a(0) = 0.0,			alpha(0) = 0.0,			d(0) = 570.0,		theta(0) = 0.0;
		a(1) = 0.0,			alpha(1) = M_PI/2,		d(1) = 0.0,			theta(1) = M_PI/2;

		// Left Arm
		a(2) = 400.0,		alpha(2) =  0.0,		d(2) = 208.0,			theta(2) = M_PI;
		a(3) = 30.0,		alpha(3) = M_PI/2,		d(3) = 0.0,				theta(3) = M_PI/2;
		a(4) = 30.0,		alpha(4) = M_PI/2,		d(4) = 330.0,/*300*/	theta(4) = M_PI;
		a(5) = 32.0,		alpha(5) = M_PI/2,		d(5) = 0.0,				theta(5) = M_PI;
		a(6) = 32.0,		alpha(6) = M_PI/2,		d(6) = 330.0,/*300*/	theta(6) = M_PI;
		a(7) = 0.0,			alpha(7) = M_PI/2,		d(7) = 0.0,				theta(7) = M_PI/2;
		a(8) = 95.0,		alpha(8) = M_PI/2,		d(8) = 0.0,				theta(8) = 0.0;

		_m_Robot.SetDH(a, alpha, d, theta, dof, CRAIG);
		_m_Robot.SetPreH(0.0, 0.0, 0.0, M_PI);
		_m_Robot.SetOriType("Ezyz");

		// Tool DH (a, alpha, d, theta)
		_m_Robot.SetToolDH(0.0, 0.0, 0.0, 0.0, TRUE);
		_m_Robot.SetPostH(0,0,1,154.2,  0,-1,0,0,  1,0,0,0,  0,0,0,0);
		_m_Robot.CalcFK();
	}
	else if((robot == STAUBLI)) 
	{
		cout << "STAUBLI Setup!" << endl;
		//// Set DH parameter for STAUBLI -----------------------------------------------------
		VectorXd a, alpha, d, theta;
		int dof = 6;
		a.resize(dof); alpha.resize(dof); d.resize(dof); theta.resize(dof);

		a(0) = 0.0,		alpha(0) = -M_PI/2,		d(0) = 0.0,			theta(0) = 0.0;
		a(1) = 290.0,	alpha(1) = 0.0,			d(1) = 0.0,			theta(1) = -M_PI/2;
		a(2) = 0.0,		alpha(2) = M_PI/2,		d(2) = 20.0,		theta(2) = M_PI/2;
		a(3) = 0.0,		alpha(3) = -M_PI/2,		d(3) = 310.0,		theta(3) = 0.0;
		a(4) = 0.0,		alpha(4) = M_PI/2,		d(4) = 0.0,			theta(4) = 0.0;
		a(5) = 0.0,		alpha(5) = 0.0,			d(5) = 70.0,		theta(5) = 0.0;	//_d(5) = 70+150

		_m_Robot.SetDH(a, alpha, d, theta, dof, PAUL);
		_m_Robot.SetOffset(0.0, 0.0, 375.0);
		_m_Robot.SetOriType("Estaubli");

		// Tool DH (a, alpha, d, theta)
		_m_Robot.SetToolDH(0.0, 0.0, 150, 0.0, TRUE);
		_m_Robot.CalcFK();
	}
	else
	{
		cout << "This robot does not exist!" << endl;
	}

	m_RobotPos.des_q = VectorXd::Zero(_m_Robot.GetDOF());	// desired joint coordinate
}

//int COpenGLControl::GetNumRobot()
//{
//	return numRobot;
//}




void COpenGLControl::SetinitGoal()
{
	// Cartesian ��ǥ ��������... Amiro x, y, z, rx, ry, rz

	//// Staubli (������ degree) �����϶�
	double initX[1][6] = {476.61, 20.0, -214.12, 0.0, 158.1, 0.0};
	
	double unit_compensate = 1000;
	//// AMIRO (������ degree) �����϶�
	double initX2[2][6] = {0.565812 * unit_compensate, -0.311854 * unit_compensate, 0.355924 * unit_compensate, 45.314683, 120.525547, 166.419746,
						   0.571082 * unit_compensate, 0.306525 * unit_compensate, 0.351505 * unit_compensate, 134.685317, -120.525547, 13.580254 };
						   //110.713, -338.739, 110.574, 55.53, 130.0, 164.48,
						   //110.713, 338.739, 110.574, 30.0, -150.0, 90.0};
	
	
	Vector3d goalPos, goalOri;
	for(int i=0; i<theApp.GetNumRobot(); i++)
	{
		for(int j=0; j<3; j++)
		{
			if(theApp.numRobot == 1) {
				goalPos(j) = initX[i][j];
				goalOri(j) = initX[i][j+3] * DtoR;
			}else if(theApp.numRobot == 2) {
				goalPos(j) = initX2[i][j];
				goalOri(j) = initX2[i][j+3] * DtoR;
			}
		}
		
		theApp.T_SW->m_robot[i].MakeRobotTransform(goalPos, goalOri, goal_T[i]);
	}

	LARGE_INTEGER liCounter1, liCounter2, liFrequency; // �Լ��� �ɸ��� �ð� �����ϱ� ���ؼ�
	QueryPerformanceFrequency(&liFrequency);  // retrieves the frequency of the high-resolution performance counter
	
	//QueryPerformanceCounter(&liCounter2);         // End
	printf("Time : %f\n", (double)(liCounter2.QuadPart - liCounter1.QuadPart) / (double)liFrequency.QuadPart);
}


void COpenGLControl::SetGoalPos(int nRobot, Vector3d pos, Vector3d ori)
{
	if(nRobot < 0 || nRobot > theApp.GetNumRobot()-1) {
		cout << "Out of index!" << endl;
		return;
	}
	theApp.T_SW->m_robot[nRobot].MakeRobotTransform(pos, ori, goal_T[nRobot]);	 // Position�� Orientation�� Homogeneous Matrix���·� ��ȯ
}

void COpenGLControl::SetGoalPos(int nRobot, Transform<double, 3, 2> *_HT_target)
{
	//HT_target�� omni thread�� �Բ� �����ϸ� �޸� �浹 �� ���� ������, omni thread���� �ƿ� ���� �����ع�����.
	for(int r=0; r<3; r++){
		for(int c=0; c<3; c++)
			goal_T[nRobot](r, c) = _HT_target->linear()(r, c);
		goal_T[nRobot](r, 3) = _HT_target->translation()(r);
	}
}


void COpenGLControl::SetDesPosition(int coordinate, float increment, int nRobot)
{
	// coordinate shoud be 0~5
	if(coordinate < 0 || coordinate > 5)
	{
		printf("Wrong coordinate! \n");
		return;
	}

	if(nRobot < 0 || nRobot > theApp.GetNumRobot()-1){
		cout << "Wrong Robot Index! (SetDesPosition)" << endl;
		return;
	}

	// Set desired position
	Matrix3d tmpA;
	Vector3d desPos, desOri;

	// Position ---
	if(coordinate < 3) {
		// position
		desPos = theApp.T_SW->m_robot[nRobot].GetPosition();
		desPos(coordinate) = desPos(coordinate)+increment;

		// orientation
		desOri = theApp.T_SW->m_robot[nRobot].GetOrientation();
	} else {	// Orientation ---
		// position
		desPos = theApp.T_SW->m_robot[nRobot].GetPosition();

		// orientation
		coordinate = coordinate - 3;
		desOri = theApp.T_SW->m_robot[nRobot].GetOrientation();
		desOri(coordinate) = desOri(coordinate)+increment*DtoR;
	}
	
	// �� �Լ��� ���� desired position and orientation�� goal_T�� ���� �����Ѵ�
	theApp.T_SW->m_robot[nRobot].MakeRobotTransform(desPos, desOri, goal_T[nRobot]);
}


void COpenGLControl::SetDesPositionByJoint(int jNum, float increment)
{
	// jNum shoud be 0~5
	if(jNum < 0 || jNum > 5){
		printf("Wrong jNum! \n");
		return;
	}

	// Calc FK
}

void COpenGLControl::SetDesPositionByJoint()
{
	// Calc FK
	for(int i=0; i<theApp.numRobot; i++)
	{
		theApp.T_SW->m_robot[i].CalcFK();
		goal_T[i] = theApp.T_SW->m_robot[i].GetTool();
	}
}

void COpenGLControl::oglCreate(CRect rect, CWnd *parent)
{
	CString className = AfxRegisterWndClass(CS_HREDRAW | CS_VREDRAW | CS_OWNDC, NULL, (HBRUSH)GetStockObject(BLACK_BRUSH), NULL);

	CreateEx(0, className, "OpenGL", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN, rect, parent, 0);

	// Set initial variables' values
	m_oldWindow	   = rect;
	m_originalRect = rect;

	hWnd = parent;
}

BEGIN_MESSAGE_MAP(COpenGLControl, CWnd)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_WM_TIMER()
	ON_WM_SIZE()
	ON_WM_MOUSEMOVE()
	ON_WM_RBUTTONDOWN()
	ON_WM_LBUTTONDOWN()
END_MESSAGE_MAP()


void COpenGLControl::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰��մϴ�.
	// �׸��� �޽����� ���ؼ��� CWnd::OnPaint()��(��) ȣ������ ���ʽÿ�.

	ValidateRect(NULL);
}


int COpenGLControl::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CWnd::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  ���⿡ Ư��ȭ�� �ۼ� �ڵ带 �߰��մϴ�.

	oglInitialize();

	return 0;
}


void COpenGLControl::oglInitialize(void)
{
	// Initial Setup:

	static PIXELFORMATDESCRIPTOR pfd =
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		32, // bit depth
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		16, // z-buffer depth
		0, 0, 0, 0, 0, 0, 0,
	};

	// Get device context only once.
	hdc = GetDC()->m_hDC;
	
	// Pixel format.
	m_nPixelFormat = ChoosePixelFormat(hdc, &pfd);
	SetPixelFormat(hdc, m_nPixelFormat, &pfd);

	// Create the OpenGL Rendering Context.
	hrc = wglCreateContext(hdc);
	wglMakeCurrent(hdc, hrc);

	// Basic Setup:
	// Set color to use when clearing the background.
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);	// RGBA: Black
	glClearDepth(1.0f);

	// Turn on backface culling
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	
	// Turn on depth testing
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	
	/* Outline Text ��� */
	OLTextInitialize();
	
	
	//// Run Thread for display
	//pThread = AfxBeginThread((AFX_THREADPROC)COpenGLControl::ThreadUpdateView, (LPVOID)this);
	//if(pThread==NULL)	printf("Failed to being thread! \n");

	// Send draw request
	OnDraw(NULL);
}


void COpenGLControl::OnDraw(CDC *pDC)
{
	// TODO: Camera controls.
	glLoadIdentity();

	// ī�޶� ��ġ: (1,1,1), 
	// ī�޶� ����: (0,0,0)�� �ٶ󺸵��� ����,  
	// ������(ī�޶��� ���� ����): (0,0,1)
	//gluLookAt(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);	// View point ��ȯ

	// orhthographic view ī�޶� �Ķ����
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-800.0f*(m_fZoom/1000), 800.0f*(m_fZoom/1000), -800.0f*(m_fZoom/1000), 800.0f*(m_fZoom/1000), -2000.0f, 4000.0f);	
	glMatrixMode(GL_MODELVIEW);


	glTranslatef(m_fPosX, m_fPosY, 0.0f);
	glRotatef(-90.0, 0.0f, 0.0f, 1.0f);		// �κ� ��ǥ�� ���� ���ϰ� ���߱� ����, x���� ȭ�� ������ ��������..
	glRotatef(m_fRotX, 0.0f, 1.0f, 0.0f);	// ���콺 �¿� ���������� y�� ȸ��
	glRotatef(m_fRotY, 0.0f, 0.0f, 1.0f);	// ���콺 ���Ʒ� ���������� z�� ȸ��
	

	// Perspective view ī�޶� �Ķ����
	//glTranslatef(-m_fZoom, -m_fZoom, -m_fZoom);
	//glTranslatef(m_fPosX, m_fPosY, 0.0f);
	//glRotatef(m_fRotX, 1.0f, 0.0f, 0.0f);
	//glRotatef(m_fRotY, 0.0f, 1.0f, 0.0f);
}


// Ÿ�̸�
void COpenGLControl::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	
	switch (nIDEvent)
	{
		case 1:
		{
			// Clear color and depth buffer bits
			glClear(GL_COLOR_BUFFER_BIT |  GL_DEPTH_BUFFER_BIT);

			//if(m_viewFlag) {
			//	m_fPosX += m_gain*(m_desfPosX-m_fPosX);
			//	//printf("value: %f \n", m_desfPosX-m_fPosX);
			//		if(abs(m_desfPosX-m_fPosX) < m_thres) {
			//			printf("Viewflag FALSE\n");
			//			m_viewFlag=FALSE;
			//		}
			//}

			if( (abs(m_desfRotX-m_fRotX) > m_thres || abs(m_desfRotY-m_fRotY) > m_thres) && m_viewFlag) {
				m_fRotX += m_gain*(m_desfRotX - m_fRotX);
				m_fRotY += m_gain*(m_desfRotY - m_fRotY);
				OnDraw(NULL);
			}else {
				m_viewFlag = FALSE;
				m_desfRotX = m_fRotX;
				m_desfRotY = m_fRotY;
			}

			//// Draw OpenGL scene
			//oglDrawScene();
			oglDrawBasis();
			oglDrawGrid();
			
			for(int i=0; i<theApp.GetNumRobot(); i++) oglDrawRobot(theApp.T_SW->m_robot[i]);
			
			if(m_viaFlag) oglDrawViaTeaching();

			if(m_viaShow) oglDrawViaPoint();

			glFlush();

			// Swap buffers
			SwapBuffers(hdc);
		}
		break;

		default:
			break;
	}


	CWnd::OnTimer(nIDEvent);
}

BOOL COpenGLControl::GetonProcess(int nRobot)
{
	if(nRobot < 0 || nRobot > theApp.GetNumRobot()-1)
	{
		cout << "Wrong Robot Index! (GetonProcess)" << endl;
		return NULL;
	}

	return onProcessing[nRobot];
}



void COpenGLControl::OnSize(UINT nType, int cx, int cy)
{
	CWnd::OnSize(nType, cx, cy);

	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰��մϴ�.
	if (0 >= cx || 0 >= cy || nType == SIZE_MINIMIZED) return;

	// Map the OpenGL coordinates.
	glViewport(0, 0, cx, cy);

	// Projection view
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	
	//gluPerspective(35.0f, (float)cx / (float)cy, 0.01f, 2000.0f);	// perspective view
	glOrtho(-800.0f, 800.0f, -800.0f, 800.0f, -2000.0f, 3000.0f);	// orhthographic view
	

	// Model view
	glMatrixMode(GL_MODELVIEW);

	switch (nType)
	{
		// If window resize token is "maximize"
		case SIZE_MAXIMIZED:
		{
			// Get the current window rect
			GetWindowRect(m_rect);

			// Move the window accordingly
			MoveWindow(6, 6, cx - 14, cy - 14);

			// Get the new window rect
			GetWindowRect(m_rect);

			// Store our old window as the new rect
			m_oldWindow = m_rect;

			break;
		}

		// If window resize token is "restore"
		case SIZE_RESTORED:
		{
			// If the window is currently maximized
			if (m_bIsMaximized)
			{
				// Get the current window rect
				GetWindowRect(m_rect);

				// Move the window accordingly (to our stored old window)
				MoveWindow(m_oldWindow.left, m_oldWindow.top - 18, m_originalRect.Width() - 4, m_originalRect.Height() - 4);

				// Get the new window rect
				GetWindowRect(m_rect);

				// Store our old window as the new rect
				m_oldWindow = m_rect;
			}
			break;
		}
	}

}

// ������ü �׸���
void COpenGLControl::oglDrawBasis(void)
{
	// Wireframe Mode
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	
	glLineWidth(3.5f);
	float s = 4.5f;	// scale constant
	
	glBegin(GL_LINES);
		// x
		glColor3f(s*1.0f,0.0f,0.0f);	// Red
        glVertex3f(0.0f,0.0f,0.0f);
        glVertex3f(s*15.0f,0.0f,0.0f);
		
		// X Text
		glVertex3f(s*(15.0f+2.0f),0.0f,s*2.0f);
		glVertex3f(s*15.0f,0.0f,s*-2.0f);
		glVertex3f(s*15.0f,0.0f,s*2.0f);
		glVertex3f(s*(15.0f+2.0f),0.0f,s*-2.0f);

		// y
		glColor3f(0.0f,s*1.0f,0.0f);	// Green
        glVertex3f(0.0f,0.0f,0.0f);
        glVertex3f(0.0f,s*15.0f,0.0f);

		// Y Text
		glVertex3f(s*-0.8f,s*(15.0f+2.5f),0.0f);
		glVertex3f(0.0f,s*(15.0f+1.5f),0.0f);
		glVertex3f(s*0.8f,s*(15.0f+2.5f),0.0f);
		glVertex3f(0.0f,s*(15.0f+1.5f),0.0f);
		glVertex3f(0.0f,s*(15.0f+1.5f),0.0f);
		glVertex3f(0.0f,s*(15.0f+0.5f),0.0f);

        
		// z
		glColor3f(0.0f,0.0f,s*1.0f);	// Blue
        glVertex3f(0.0f,0.0f,0.0f);
        glVertex3f(0.0f,0.0f,s*15.0f);

		// Z Text
		glVertex3f(s*-1.0f,0.0f,s*(15.0f+0.5f));
        glVertex3f(s*1.0f,0.0f,s*(15.0f+0.5f));
		glVertex3f(s*1.0f,0.0f,s*(15.0f+0.5f));
		glVertex3f(s*-1.0f,0.0f,s*(15.0f+2.5f));
		glVertex3f(s*-1.0f,0.0f,s*(15.0f+2.5f));
		glVertex3f(s*1.0f,0.0f,s*(15.0f+2.5f));
    glEnd();
}


// ����긮 �׸���
void COpenGLControl::oglDrawRobot(T_Robot &_m_Robot)
{
	//// �� ��ǰ���� ���� �ٸ� ��ȯ�� �Ͼ�� �ϸ� ���� ���� �ٸ� ��ĵ��� ����Ǿ�� �Ѵ�. 
	//// glPushMatrix�� �� ������ �����ϰ� �ذ��ϱ� ���� ��� �����̴�. 
	//// ��� ������ ���� �ٸ� ��Ȳ�� ���� ���� �ٸ� ��ĵ��� �����Ϸ� �� �� ��ĵ��� �Ͻ�������
	//// ������ �α� ���� �����̴�. 
	//glPushMatrix();

	//	// Wireframe Mode
	//	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//
	//	//glPointSize(10.0f);
	//	glLineWidth(5.0f);
	//	glColor3f(1.0f,1.0f,1.0f);
	//	
	//	
	//	float radius=5.0f;

	//	glBegin(GL_LINES);	//start drawing a line loop


	//		// Get Pointer of Transformation function
	//		Matrix4d *_T = _m_Robot.GetpT();

	//		// Additional parameter setting
	//		UINT dof = _m_Robot.GetDOF();
	//		
	//		float x_offset = _m_Robot.GetOffset(0);
	//		float y_offset = _m_Robot.GetOffset(1);
	//		float z_offset = _m_Robot.GetOffset(2);
	//		
	//		MatrixXd P = _m_Robot.GetPointsforDraw();


	//		//// Link �׸���---------------------------
	//		//// Base ��ũ
	//		glVertex3f(P(0,0)+x_offset, P(0,1)+y_offset, P(0,2)+z_offset);
	//		glVertex3f(P(0,3)+x_offset, P(0,4)+y_offset, P(0,5)+z_offset);

	//		//// ������
	//		for(int i=1; i<dof; i++)
	//		{	
	//			glVertex3f(P(i-1,3)+x_offset, P(i-1,4)+y_offset, P(i-1,5)+z_offset);
	//			glVertex3f(P(i,0)+x_offset, P(i,1)+y_offset, P(i,2)+z_offset);

	//			glVertex3f(P(i,0)+x_offset, P(i,1)+y_offset, P(i,2)+z_offset);
	//			glVertex3f(P(i,3)+x_offset, P(i,4)+y_offset, P(i,5)+z_offset);
	//		}

	//		// Additional Tool Coordinate Drawing
	//		Matrix4d tT = _m_Robot.GetTool();
	//		if(_m_Robot.isTool()) {
	//			glVertex3f(_T[dof-1](0,3)+x_offset, _T[dof-1](1,3)+y_offset, _T[dof-1](2,3)+z_offset);
	//			glVertex3f(tT(0,3)+x_offset, tT(1,3)+y_offset, tT(2,3)+z_offset);
	//		}else{
	//			tT = _T[dof-1];
	//		}
	//		
	//		// End-effector ��ǥ �׸���----------------
	//		int l = 50;
	//		glLineWidth(3.0f);

	//		for(int i=0; i<3; i++) {	// X(red), Y(green), Z(blue)
	//			float r = i==0 ? 1.0f : 0.0f;
	//			float g = i==1 ? 1.0f : 0.0f;
	//			float b = i==2 ? 1.0f : 0.0f;
	//			glColor3f(r, g, b);
	//			glVertex3f(tT(0,3)+x_offset, tT(1,3)+y_offset, tT(2,3)+z_offset);
	//			glVertex3f((tT(0,3)+l*tT(0,i))+x_offset, (tT(1,3)+l*tT(1,i))+y_offset, (tT(2,3)+l*tT(2,i))+z_offset);
	//		}

	//	glEnd(); //end drawing of line loop

	//	
	//	//// Joint �׸��� ---------------------------------
	//	//// Draw Joints
	//	float prex=0.0f, prey=0.0f, prez=0.0f;
	//	glColor3f(1.0f,0.0f,0.0f);
	//	glTranslatef(x_offset, y_offset, z_offset);		// base
	//	glutSolidSphere(radius, 20, 20);

	//	for(int i=0; i<dof; i++)
	//	{
	//		glTranslatef((_T[i](0,3)-prex), (_T[i](1,3)-prey), (_T[i](2,3)-prez));
	//		glutSolidSphere(radius, 20, 20);
	//		prex=_T[i](0,3); prey=_T[i](1,3); prez=_T[i](2,3);
	//	}

	//	// Tool�� �����Ǿ� �ִٸ�... 
	//	if(_m_Robot.isTool()){
	//		glTranslatef((tT(0,3)-prex), (tT(1,3)-prey), (tT(2,3)-prez));
	//		glutSolidSphere(radius, 20, 20);
	//	}

	//glPopMatrix();

	/***********************/
	// �� ��ǰ���� ���� �ٸ� ��ȯ�� �Ͼ�� �ϸ� ���� ���� �ٸ� ��ĵ��� ����Ǿ�� �Ѵ�. 
    // glPushMatrix�� �� ������ �����ϰ� �ذ��ϱ� ���� ��� �����̴�. 
    // ��� ������ ���� �ٸ� ��Ȳ�� ���� ���� �ٸ� ��ĵ��� �����Ϸ� �� �� ��ĵ��� �Ͻ�������
    // ������ �α� ���� �����̴�. 
    glPushMatrix();

    // Wireframe Mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        
    //glPointSize(10.0f);
    glLineWidth(5.0f);
    glColor3f(1.0f,1.0f,1.0f);
               
               
    float radius=5.0f;

    glBegin(GL_LINES);     //start drawing a line loop


            // Get Pointer of Transformation function
            Matrix4d *_T = _m_Robot.GetpT();

            // Additional parameter setting
            UINT dof = _m_Robot.GetDOF();
                       
            float x_offset = _m_Robot.GetOffset(0);
            float y_offset = _m_Robot.GetOffset(1);
            float z_offset = _m_Robot.GetOffset(2);
                       
            MatrixXd P = _m_Robot.GetPointsforDraw();
            int N = P.rows();
                       

            //// Link �׸���---------------------------
            //// Base ��ũ
            glVertex3f(P(0,0)+x_offset, P(0,1)+y_offset, P(0,2)+z_offset);
            glVertex3f(P(0,3)+x_offset, P(0,4)+y_offset, P(0,5)+z_offset);

            //// ������
            for(int i=1; i<N; i++)
            {       
                    glVertex3f(P(i-1,3)+x_offset, P(i-1,4)+y_offset, P(i-1,5)+z_offset);
                    glVertex3f(P(i,0)+x_offset, P(i,1)+y_offset, P(i,2)+z_offset);

                    glVertex3f(P(i,0)+x_offset, P(i,1)+y_offset, P(i,2)+z_offset);
                    glVertex3f(P(i,3)+x_offset, P(i,4)+y_offset, P(i,5)+z_offset);
            }

            //// Additional Tool Coordinate Drawing
            //Matrix4d tT = _m_Robot.GetTool();
            //if(_m_Robot.isTool()) {
            //      glVertex3f(_T[dof-1](0,3)+x_offset, _T[dof-1](1,3)+y_offset, _T[dof-1](2,3)+z_offset);
            //      glVertex3f(tT(0,3)+x_offset, tT(1,3)+y_offset, tT(2,3)+z_offset);
            //}else{
            //      tT = _T[dof-1];
            //}
                       
            // End-effector ��ǥ �׸���----------------
            int l = 50;
            glLineWidth(3.0f);

            Matrix4d tT = _m_Robot.GetTool();
            for(int i=0; i<3; i++) {      // X(red), Y(green), Z(blue)
                    float r = i==0 ? 1.0f : 0.0f;
                    float g = i==1 ? 1.0f : 0.0f;
                    float b = i==2 ? 1.0f : 0.0f;
                    glColor3f(r, g, b);
                    glVertex3f(P(N-1,3)+x_offset, P(N-1,4)+y_offset, P(N-1,5)+z_offset);
                    glVertex3f((P(N-1,3)+l*tT(0,i))+x_offset, (P(N-1,4)+l*tT(1,i))+y_offset, (P(N-1,5)+l*tT(2,i))+z_offset);
                    //glVertex3f(tT(0,3)+x_offset, tT(1,3)+y_offset, tT(2,3)+z_offset);
                    //glVertex3f((tT(0,3)+l*tT(0,i))+x_offset, (tT(1,3)+l*tT(1,i))+y_offset, (tT(2,3)+l*tT(2,i))+z_offset);
            }

    glEnd(); //end drawing of line loop

               
    //// Joint �׸��� ---------------------------------
    //// Draw Joints
    float prex=0.0f, prey=0.0f, prez=0.0f;
    glColor3f(1.0f,0.0f,0.0f);
    glTranslatef(x_offset, y_offset, z_offset);          // base
    glutSolidSphere(radius, 20, 20);

    for(int i=0; i<N; i++)
    {
            glTranslatef((P(i,3)-prex), (P(i,4)-prey), (P(i,5)-prez));
            glutSolidSphere(radius, 20, 20);
            prex=P(i,3); prey=P(i,4); prez=P(i,5);
    }

    //// Tool�� �����Ǿ� �ִٸ�... 
    //if(_m_Robot.isTool()){
    //      glTranslatef((tT(0,3)-prex), (tT(1,3)-prey), (tT(2,3)-prez));
    //      glutSolidSphere(radius, 20, 20);
    //}

    glPopMatrix();

}


void COpenGLControl::oglDrawScene(void)
{
	 // Wireframe Mode
   glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
 
   glBegin(GL_QUADS);
      // Top Side
      glVertex3f( 1.0f, 1.0f,  1.0f);
      glVertex3f( 1.0f, 1.0f, -1.0f);
      glVertex3f(-1.0f, 1.0f, -1.0f);
      glVertex3f(-1.0f, 1.0f,  1.0f);
 
      // Bottom Side
      glVertex3f(-1.0f, -1.0f, -1.0f);
      glVertex3f( 1.0f, -1.0f, -1.0f);
      glVertex3f( 1.0f, -1.0f,  1.0f);
      glVertex3f(-1.0f, -1.0f,  1.0f);
 
      // Front Side
      glVertex3f( 1.0f,  1.0f, 1.0f);
      glVertex3f(-1.0f,  1.0f, 1.0f);
      glVertex3f(-1.0f, -1.0f, 1.0f);
      glVertex3f( 1.0f, -1.0f, 1.0f);
 
      // Back Side
      glVertex3f(-1.0f, -1.0f, -1.0f);
      glVertex3f(-1.0f,  1.0f, -1.0f);
      glVertex3f( 1.0f,  1.0f, -1.0f);
      glVertex3f( 1.0f, -1.0f, -1.0f);
 
      // Left Side
      glVertex3f(-1.0f, -1.0f, -1.0f);
      glVertex3f(-1.0f, -1.0f,  1.0f);
      glVertex3f(-1.0f,  1.0f,  1.0f);
      glVertex3f(-1.0f,  1.0f, -1.0f);
 
      // Right Side
      glVertex3f( 1.0f,  1.0f,  1.0f);
      glVertex3f( 1.0f, -1.0f,  1.0f);
      glVertex3f( 1.0f, -1.0f, -1.0f);
      glVertex3f( 1.0f,  1.0f, -1.0f);
   glEnd();
}


// ������ ���̱�/����� ���� �Լ�
void COpenGLControl::viaShow()
{
	m_viaShow = TRUE;
}

void COpenGLControl::viaHide()
{
	m_viaShow = FALSE;
}



// ������(via points) �׸��� �Լ�
void COpenGLControl::oglDrawViaPoint(void)
{	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(2.0f);
	float radius=5.0f;
		
	// TODO
	//double z_offset = 375.0;	// for STAUBLI
	double z_offset = 0.0;		// for AMIRO and other Robot

	//// Draw Via Points
	for(int i=0; i<theApp.GetNumRobot(); i++)
		for(int j=0; j<theApp.T_SW->m_vp[i].GetNum(); j++)
		{
			z_offset = theApp.T_SW->m_robot[i].GetOffset(2); //for Any robot..?

			glPushMatrix();
				if(selectedVP[i]==j)
					glColor3f(1.0f,0.0f,1.0f);	// Magenta, rest
				else
					glColor3f(0.0f,1.0f,1.0f);	// Cyan, selected via-point
			
				glTranslatef(theApp.T_SW->m_vp[i].GetVal(j,0), theApp.T_SW->m_vp[i].GetVal(j,1), theApp.T_SW->m_vp[i].GetVal(j,2)+z_offset);	// TODO 375?
				glutSolidSphere(radius, 10, 10);
			glPopMatrix();
			
			// ���õ� ���� ���� �׸���
			float r = 50;
			
			// gamma -> alpha ��..
			double alpha = theApp.T_SW->m_vp[i].GetVal(j,3);	// rx
			double beta  = theApp.T_SW->m_vp[i].GetVal(j,4);	// ry
			double gamma = theApp.T_SW->m_vp[i].GetVal(j,5);	// rz

			// TODO
			//Matrix3d mat = theApp.T_SW->GetRfrom_fXYZ(alpha, beta, gamma); // RzRyRxV
			Matrix3d mat = theApp.T_SW->GetRfrom_eXYZ(alpha, beta, gamma);	// RxRyRzV
			Vector3d vec = r * mat.col(2).segment(0,3);

			// Orientation Line �׸���
			glLineWidth(5.0f);
			glBegin(GL_LINES);
				// z-axis
				glColor3f(0.0f, 0.0f, 1.0f);	// blue
				glVertex3f(theApp.T_SW->m_vp[i].GetVal(j,0),		theApp.T_SW->m_vp[i].GetVal(j,1),		 theApp.T_SW->m_vp[i].GetVal(j,2) + z_offset);
				glVertex3f(theApp.T_SW->m_vp[i].GetVal(j,0)+vec(0), theApp.T_SW->m_vp[i].GetVal(j,1)+vec(1), theApp.T_SW->m_vp[i].GetVal(j,2)+vec(2) + z_offset);
				
				// x-axis
				vec = r * mat.col(0).segment(0,3);
				glColor3f(1.0f, 0.0f, 0.0f);	// red
				glVertex3f(theApp.T_SW->m_vp[i].GetVal(j,0),		theApp.T_SW->m_vp[i].GetVal(j,1),		 theApp.T_SW->m_vp[i].GetVal(j,2) + z_offset);
				glVertex3f(theApp.T_SW->m_vp[i].GetVal(j,0)+vec(0), theApp.T_SW->m_vp[i].GetVal(j,1)+vec(1), theApp.T_SW->m_vp[i].GetVal(j,2)+vec(2) + z_offset);	

				// y-axis
				vec = r * mat.col(1).segment(0,3);
				glColor3f(0.0f, 1.0f, 0.0f);	// green
				glVertex3f(theApp.T_SW->m_vp[i].GetVal(j,0),		theApp.T_SW->m_vp[i].GetVal(j,1),		 theApp.T_SW->m_vp[i].GetVal(j,2) + z_offset);
				glVertex3f(theApp.T_SW->m_vp[i].GetVal(j,0)+vec(0), theApp.T_SW->m_vp[i].GetVal(j,1)+vec(1), theApp.T_SW->m_vp[i].GetVal(j,2)+vec(2) + z_offset);	
			glEnd();
			

			if(j>0){	// via-point�� line �׸���
				glLineWidth(2.0f);
				glColor3f(1.0f,1.0f,0.0f);	// yellow
				glBegin(GL_LINES);
					// vertex, TODO, 375
					glVertex3f(theApp.T_SW->m_vp[i].GetVal(j-1,0), theApp.T_SW->m_vp[i].GetVal(j-1,1), theApp.T_SW->m_vp[i].GetVal(j-1,2)+z_offset);	// from
					glVertex3f(theApp.T_SW->m_vp[i].GetVal(j,0), theApp.T_SW->m_vp[i].GetVal(j,1), theApp.T_SW->m_vp[i].GetVal(j,2)+z_offset);			// to
				glEnd();
			}
		}
	


	////// Draw Via Points
	//for(int j=0; j<nVP; j++)
	//{
	//	
	//	if(vp[j](0)!=0.0f || vp[j](1)!=0.0f || vp[j](2)!=0.0f)
	//	{
	//		glPushMatrix();
	//			if(selectedVP==j)
	//				glColor3f(1.0f,0.0f,1.0f);	// Magenta, rest
	//			else
	//				glColor3f(0.0f,1.0f,1.0f);	// Cyan, selected via-point

	//			glTranslatef(vp[j](0), vp[j](1), vp[j](2)+375.0f);
	//			glutSolidSphere(radius, 10, 10);
	//		glPopMatrix();

	//		if(j>0){	// via-point�� line �׸���
	//			glColor3f(1.0f,1.0f,0.0f);	// yellow

	//			glBegin(GL_LINES);
	//				// vertex
	//				glVertex3f(vp[j-1](0), vp[j-1](1), vp[j-1](2)+375.0f);	// from
	//				glVertex3f(vp[j](0), vp[j](1), vp[j](2)+375.0f);		// to
	//			glEnd();
	//			

	//			// input vector
	//			float vec[3], norm;
	//			vec[0] = vp[j](0)-vp[j-1](0);
	//			vec[1] = vp[j](1)-vp[j-1](1);
	//			vec[2] = vp[j](2)-vp[j-1](2);
	//			
	//			norm = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);	// norm of the vector

	//			// normalization
	//			float norm_v[3];
	//			norm_v[0] = vec[0]/norm;
	//			norm_v[1] = vec[1]/norm;
	//			norm_v[2] = vec[2]/norm;
	//			

	//			// Construct transformation matrix
	//			float mat44[16] =
	//			{1,0,0,0,
	//			 0,1,0,0,
	//			 0,0,1,0,
	//			 0,0,0,1};
 //
	//			mat44[0] = norm_v[0]; // [0,0]
	//			mat44[5] = norm_v[1]; // [1,1]
	//			mat44[10] = norm_v[2]; // [2,2]
	//			mat44[15] = 1.0;

	//			glPushMatrix();
	//				// Move and rotate in position:
	//				//glMultMatrixf( mat44 );

	//				float yaw, pitch, roll, axb;
	//				
	//				roll = acos(norm_v[0])*(180/M_PI);
	//				pitch = acos(norm_v[1])*(180/M_PI);
	//				yaw = acos(norm_v[2])*(180/M_PI);
	//				//printf("r: %f, p: %f, y: %f \n", roll, pitch, yaw);

	//				//glRotated(roll, 1.0f, 0.0f,0.0f);	// ���� ��ǥ�� ��������?
	//				//glRotated(pitch, 0.0f, 1.0f,0.0f);
	//				//glRotated(yaw, 0.0f, 0.0f,1.0f);
	//				//glTranslatef(vp[j](0), vp[j](1), vp[j](2)+375.0f);
	//				//glutSolidCone(10,60,16,16);
	//			glPopMatrix();
	//		}
	//	}
	//	//else if(vp[j](0)==0.0f && vp[j](1)==0.0f && vp[j](2)==0.0f)	// x,y,z�� ���� 0�� ��� �ȱ׸�
	//		//break;
	//}
}


// Grid �׸���..
void COpenGLControl::oglDrawGrid(void)
{
	// ���� ����, ��ǥ�� ������ �κ��� �ٶ� ���� �� ���� ���� �������� �����..,.
	int gWidth = 2000;		
	int gHeight = 1500;
	m_gap = 50;

	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);
	glColor3f(0.75f, 0.75f, 0.75f);
	glLineWidth(0.5);

	glBegin(GL_LINES);	
	// �� ������ �������� �ʴ� �Ķ���͸� �����ص� grid�� �߽���ǥ�� �κ��� ������ ���߱� ����
	// Horizontal lines
	for(int i=0; i<gWidth/m_gap/2+1; i++) {
		float gin = -i*m_gap;	// negative direction
		glVertex3f(-gHeight/2, (float)gin, 0.0);
		glVertex3f(gHeight/2, (float)gin, 0.0);

		if(gin==0) continue;

		float gip = i*m_gap;	// positive direction
		glVertex3f(-gHeight/2, (float)gip, 0.0);
		glVertex3f(gHeight/2, (float)gip, 0.0);
	}

	// Vertical lines
	for(int i=0; i<gHeight/m_gap/2+1; i++) {
		float gin = -i*m_gap;	// negative direction
		glVertex3f((float)gin, -gWidth/2, 0.0);
		glVertex3f((float)gin, gWidth/2, 0.0);

		if(gin==0) continue;

		float gip = i*m_gap;	// positive direction
		glVertex3f((float)gip, -gWidth/2, 0.0);
		glVertex3f((float)gip, gWidth/2, 0.0);
	}
	glEnd();
	glFlush();
}



// via point teaching ������ �׸��� ���� �Լ�
void COpenGLControl::oglDrawViaTeaching()
{
	float radius=5.0f;
	float r = 100.0;
	
	glColor3f(1.0f,0.0f,0.0f);
	int out[3];
	setGridPointbyClick(worldPos.segment(0,3) , m_gap, out);
	
	glPushMatrix();
	switch(Teachstep) {
	case 0:		// XY Plane
		glTranslatef(worldPos_grid(0), worldPos_grid(1), worldPos_grid(2));
		glutSolidSphere(radius, 20, 20);
		break;
	case 1:		// Yaw axis
		{
		// mouse��ǥ�� �ƴ� world ��ǥ �������� ���
		float x = worldPos_grid(0)-Teaching_data_grid(0);
		float y = worldPos_grid(1)-Teaching_data_grid(1);
		float theta = atan2(y, x);	// radian

		//printf("gx: %.2f, gy: %.2f, theta: %.2f \n", worldPos_grid(0), worldPos_grid(1), theta*RtoD);
		glBegin(GL_LINES);
			glColor3f(1.0f,1.0f,0.0f);
			glVertex3f(Teaching_data_grid(0),Teaching_data_grid(1),0.0f);
			glVertex3f(Teaching_data_grid(0)+r*cos(theta), Teaching_data_grid(1)+r*sin(theta), 0.0f);
		glEnd();

		// Draw Circle, ������ ��
		glPushMatrix();
			glTranslatef(Teaching_data_grid(0), Teaching_data_grid(1), Teaching_data_grid(2));
			oglDrawCircle(r);
		glPopMatrix();
		}

		glTranslatef(Teaching_data_grid(0), Teaching_data_grid(1), worldPos_grid(2));
		glutSolidSphere(radius, 20, 20);

		// Text ���, TODO
		{
			char* str = "sdflksjdlfkjsdlfkjsldkfjlskdjflskdjlkjflskdjlf";

			glPushAttrib(GL_LIST_BIT);
				glListBase(m_listBase - 32);
				glCallLists(strlen(str), GL_UNSIGNED_BYTE, str);
			glPopAttrib();
		}

		break;
	case 2:		// Z Plane
		glTranslatef(Teaching_data_grid(0), Teaching_data_grid(1), worldPos_grid(2));
		glutSolidSphere(radius, 20, 20);
		break;
	case 3:		// Pitch axis
		{
		// mouse��ǥ�� �ƴ� world ��ǥ �������� ���
		float y = worldPos_grid(1)-Teaching_data_grid(1);
		float z = worldPos_grid(2)-Teaching_data_grid(2);
		float theta = atan2(z, y);	// radian
		
		//printf("gy: %.2f, gz: %.2f, theta: %.2f \n", worldPos_grid(1), worldPos_grid(2), theta*RtoD);
		glBegin(GL_LINES);
			glColor3f(1.0f,1.0f,0.0f);
			glVertex3f(Teaching_data_grid(0), Teaching_data_grid(1), Teaching_data_grid(2));
			glVertex3f(Teaching_data_grid(0), Teaching_data_grid(1)+r*cos(theta), Teaching_data_grid(2)+r*sin(theta));
		glEnd();
		}

		// Draw Circle, ������ ��
		glPushMatrix();
			glTranslatef(Teaching_data_grid(0), Teaching_data_grid(1), Teaching_data_grid(2));
			oglDrawCircle(r, 0.01, 1);	// YZ ��鿡 �� �׸���	
		glPopMatrix();

		glTranslatef(Teaching_data_grid(0), Teaching_data_grid(1), Teaching_data_grid(2));
		glutSolidSphere(radius, 20, 20);
		break;
	default: 
		glTranslatef(worldPos_grid(0), worldPos_grid(1), worldPos_grid(2));
		glutSolidSphere(radius, 20, 20);
		break;
	}
	glPopMatrix();
	

	// Teaching�� ���� Line �׸���
	glBegin(GL_LINES);
		glColor3f(0.7f,0.5f,0.0f);
        glVertex3f(Teaching_data_grid(0),Teaching_data_grid(1),0.0f);
        glVertex3f(Teaching_data_grid(0),Teaching_data_grid(1),1500.0f);
	glEnd();
}



void COpenGLControl::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	
	// OpenGL â�� �簢 ���� �޾ƿ���
	CRect rect = m_oldWindow;

	// OpenGLâ ���� �ȿ��� ���� �� ������ ���� �ϱ�
	//// ************* 2d mouse ��ǥ <-> 3d coordinate conversion reference ************** ////
	//// http://www.opentk.com/node/480
	//// http://www.opentk.com/node/1892
	//// http://www.gamedev.net/topic/632454-incorrect-3d-coordinates-from-2d-mouse-click/
	//// ***************************************************************************************
	if(point.x >= 0 && point.x < rect.right && point.y >= 0 && point.y < rect.bottom) {

		// step1: 3d Normalized Device Coordinates
		float nx = (2.0f * point.x)/rect.Width() - 1.0f;
		float ny = 1.0f - (2.0f * point.y)/rect.Height();
		float nz = 1.0f;
		
		// step2: 4d Homogeneous Clip Coornidates
		Vector4d vec4(ny, ny, -1.0, 1.0);
		
		// step3: 4d Eye(camera) Coordinates
		int* viewport = new int[4];
		double* matrixprojection = new double[16];
		double* matrixmodeview = new double[16];
		
		glGetDoublev(GL_PROJECTION_MATRIX, matrixprojection);
		glGetDoublev(GL_MODELVIEW_MATRIX, matrixmodeview);
		glGetIntegerv(GL_VIEWPORT, viewport);
		
		Vector3d win = Vector3d(point.x, viewport[3]-point.y, 0);
		Vector3d worldPositionNear;
		gluUnProject(win(0), win(1), win(2), matrixmodeview, matrixprojection, viewport, 
			&worldPositionNear(0), &worldPositionNear(1), &worldPositionNear(2));
		
		worldPos.segment(0,3) = worldPositionNear;	// world position

		win(2) = 1.0f;

		Vector3d worldPositionFar;
		gluUnProject(win(0), win(1), win(2), matrixmodeview, matrixprojection, viewport, 
			&worldPositionFar(0), &worldPositionFar(1), &worldPositionFar(2));

		//printf("x: %d, y: %d \n", point.x, point.y);	
		//printf("nx: %.2f, ny: %.2f \n", nx, ny);
		//printf("wx: %.2f, wy: %.2f, wz: %.2f \n", worldPositionNear(0), worldPositionNear(1), worldPositionNear(2));
		//printf("3x: %d, 3y: %d, w: %d, h: %d \n", point.x, point.y, rect.Width(), rect.Height());

		
		delete[] viewport;
		delete[] matrixprojection;
		delete[] matrixmodeview;
	}
		


	int diffX = (int)(point.x - m_fLastX);
	int diffY = (int)(point.y - m_fLastY);
	m_fLastX  = (float)point.x;
	m_fLastY  = (float)point.y;


	// Left mouse button (Rotate)
	if (nFlags & MK_LBUTTON && !m_ShiftOn && !m_viaFlag)
	{
		m_fRotX += (float)0.5f * diffY;

		if ((m_fRotX > 360.0f) || (m_fRotX < -360.0f))
		{
			m_fRotX = 0.0f;
		}

		m_fRotY += (float)0.5f * diffX;

		if ((m_fRotY > 360.0f) || (m_fRotY < -360.0f))
		{
			m_fRotY = 0.0f;
		}
		//printf("Rotate x: %f y:%f\n", m_fRotX, m_fRotY);
	}

	//// Right mouse button (Zoom In/Out)
	//else if (nFlags & MK_RBUTTON)
	//{
	//	m_fZoom -= (float)0.1f * diffY;
	//	printf("Zoom In/Out \n");
	//}

	// Shift + Left mouse button (Pan)
	else if (nFlags & MK_LBUTTON && m_ShiftOn)
	{
		//printf("Pan : %lf \n", m_fZoom);
		m_fPosX += (float)0.03f * diffX*m_fZoom/10;
		m_fPosY -= (float)0.03f * diffY*m_fZoom/10;
	}

	OnDraw(NULL);

	

	CWnd::OnMouseMove(nFlags, point);
}



void COpenGLControl::setViewUP()
{
	float step = 10;
	m_fRotX += (float)0.5f * -step;

	if ((m_fRotX > 360.0f) || (m_fRotX < -360.0f))
	{
		m_fRotX = 0.0f;
	}
	OnDraw(NULL);
}

void COpenGLControl::setViewDOWN()
{
	float step = 10;
	m_fRotX += (float)0.5f * step;

	if ((m_fRotX > 360.0f) || (m_fRotX < -360.0f))
	{
		m_fRotX = 0.0f;
	}
	OnDraw(NULL);
}

void COpenGLControl::setViewLEFT()
{
	float step = 10;
	m_fRotY += (float)0.5f * -step;
	if ((m_fRotY > 360.0f) || (m_fRotY < -360.0f))
	{
		m_fRotY = 0.0f;
	}
	OnDraw(NULL);
}

void COpenGLControl::setViewRIGHT()
{
	float step = 10;
	m_fRotY += (float)0.5f * step;

	if ((m_fRotY > 360.0f) || (m_fRotY < -360.0f))
	{
		m_fRotY = 0.0f;
	}
	OnDraw(NULL);
}



// grid�� click�� ���� rough teaching �ϱ� ����.. �ָ��� ���� click�ص� grid point�� ����ֱ� ���ؼ�
void COpenGLControl::setGridPointbyClick(Vector3d _input, int _scale, int _grid_coord[])
{
	// �̳��� position�� ���ؼ��� ��ȯ����...
	for (int i=0; i<3; i++)
		_grid_coord[i] = (int)(_input(i)/_scale);

	float residual[3] = {0.0f};

	for (int i=0; i<3; i++)
		residual[i] = _input(i)/_scale - _grid_coord[i];

	for (int i=0; i<3; i++){
		if(residual[i] < 0.5)
			_grid_coord[i] = _grid_coord[i];
		else
			_grid_coord[i] = _grid_coord[i] + 1;
	
		worldPos_grid[i] = (double)_grid_coord[i] * (double)m_gap;
	}
}


void COpenGLControl::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	if(m_AltOn) {
		setInitialView();
		m_AltOn=FALSE;
		CWnd::OnRButtonDown(nFlags, point);
		return;
	}

	//// system�� �ʱ�ȭ(IK loop)�Ǿ������� ���� �˻�
	//if(!onIKstart) {
	//	MessageBox("Initialize the system first!");
	//	return;
	//}

	printf("Mouse Right button Click! \n");
	m_viaFlag = !m_viaFlag;
	if(m_viaFlag) {
		Teachstep = 0;
		setTopView();
	}

	CWnd::OnRButtonDown(nFlags, point);
}



void COpenGLControl::setInitialView(void)
{
	m_fPosX = 0.0f;		// X position of model in camera view
	m_fPosY = -150.0f;		// Y position of model in camera view
	m_fZoom = 1000.0f;	// Zoom on model in camera view
	// isometric view
	
	double iangle = atan(1/sqrt(2.0))*(180/PI);
	m_fRotX = iangle-90;		// y������ ȸ��..
	m_fRotY	= -45.0;			// z������ ȸ��..
	m_bIsMaximized = false;
	m_AltOn = FALSE;

	OnDraw(NULL);
}


void COpenGLControl::setTopView(void)
{
	m_viewFlag = TRUE;
	m_desfRotX = 0.0f;
	m_desfRotY = 0.0f;
}

void COpenGLControl::setFrontView(void)
{
	m_viewFlag = TRUE;
	m_desfRotX = -90.0f;
	m_desfRotY = 0.0f;
}

void COpenGLControl::setLeftView(void)
{
	m_viewFlag = TRUE;
	m_desfRotX = -90.0f;
	m_desfRotY = 90.0f;
}


void COpenGLControl::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	CHT_RTSDlg* dlg = (CHT_RTSDlg*)AfxGetMainWnd();
	dlg->m_MainControlTab.SetFocus();

	CWnd::OnLButtonDown(nFlags, point);

	printf("RotX: %f, RotY: %f \n", m_fRotX, m_fRotY);

	if(m_viaFlag && !m_ShiftOn) { 
		Teachstep++;

		// Teaching Step, 1: XY, 2: Z
		//if (Teachstep == 1){
		//	// ù ��° �ܰ�(XY)
		//	// 1���� xy ��� ���� ���� ���� ����
		//	Teaching_data_grid = worldPos_grid;	// Teach Line�� �׸��� ����
		//	Teaching_data_grid(2) = 0.0;

		//	// view ��ȯ
		//	setFrontView();
		//}
		//else if (Teachstep == 2){
		//	// �� ��° �ܰ�(Z)
		//	
		//	//// IK Ǯ��α�.. nominal robot model��
		//	memcpy(&theApp.T_SW->nominal_robot[0], &theApp.T_SW->m_robot[0], sizeof(Robot));
		//	if(theApp.T_SW->nominal_robot[0].GetQ() == theApp.T_SW->m_robot[0].GetQ())
		//		cout<<"Same Q!"<<endl;
		//	if(theApp.T_SW->nominal_robot[0].GetTool() == theApp.T_SW->m_robot[0].GetTool())
		//		cout<<"Same .GetTool()!"<<endl;
		//	if(theApp.T_SW->nominal_robot[0].GetDH() == theApp.T_SW->m_robot[0].GetDH())
		//		cout<<"Same .GetDH()!"<<endl;
		//	Matrix4d temp_IKGoal;
		//	Vector3d temp_posGoal;
		//	temp_posGoal = Teaching_data_grid;
		//	temp_posGoal(2) = worldPos_grid(2) - theApp.T_SW->nominal_robot[0].GetOffset(2);
		//	Vector3d temp_oriGoal;
		//	temp_oriGoal.setZero();
		//	theApp.T_SW->nominal_robot[0].MakeRobotTransform(temp_posGoal, temp_oriGoal, temp_IKGoal);
	
		//	// �Ʒ��� �Ϻκи� IK Ǯ�� ���� �� selective Matrix ���
		//	MatrixXd sMat(6, 6);
		//	sMat.setZero();
		//	sMat(0, 0) = 1.0; sMat(1, 1) = 1.0; sMat(2, 2) = 1.0;
		//	
		//	VectorXd IKdq(theApp.T_SW->nominal_robot[0].GetDOF());
		//	bool b_gtIKfin = false;
		//	double pos_res = 1.0;
		//	double ori_res = 0.01;
		//	while(!b_gtIKfin){
		//		double pos_error = (temp_IKGoal.col(3).segment(0,3) - theApp.T_SW->nominal_robot[0].GetTool().col(3).segment(0,3)).norm();
		//		Vector3d ori_error;
		//		ori_error(0) = 1.0 - temp_IKGoal.col(0).segment(0,3).dot(theApp.T_SW->nominal_robot[0].GetTool().col(0).segment(0,3));
		//		ori_error(1) = 1.0 - temp_IKGoal.col(1).segment(0,3).dot(theApp.T_SW->nominal_robot[0].GetTool().col(1).segment(0,3));
		//		ori_error(2) = 1.0 - temp_IKGoal.col(2).segment(0,3).dot(theApp.T_SW->nominal_robot[0].GetTool().col(2).segment(0,3));
		//		IKdq = theApp.T_SW->IKLoop_selective(&theApp.T_SW->nominal_robot[0], temp_IKGoal, pos_res, ori_res, pos_error, 0.0/*ori_error.norm()*/, sMat, b_gtIKfin);
		//		for(int i=0; i<theApp.T_SW->nominal_robot[0].GetDOF(); i++)
		//			theApp.T_SW->nominal_robot[0].SetQ(i, theApp.T_SW->nominal_robot[0].GetQ(i) + IKdq(i)); 
		//		
		//		if(b_gtIKfin == true){
		//			//// reach ������ via point ����.. �켱 joint space ����..
		//			//theApp.T_SW->m_vp[0].vp_Greach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
		//			//theApp.T_SW->m_vp[0].vp_Greach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK �� Ǯ�� ���� �������� ����
		//			//cout<<"IK for vision data reach to grip finished"<<theApp.T_SW->m_vp[0].vp_Greach_q<<endl;
		//			cout<<"**********************"<<endl;
		//			cout<<"calculating IK....Q1"<<theApp.T_SW->m_robot[0].GetQ()<<endl;
		//			cout<<"calculating IK....Q2"<<theApp.T_SW->nominal_robot[0].GetQ()<<endl;
		//			cout<<"calculating IK....X"<<theApp.T_SW->nominal_robot[0].GetTool().col(3)<<endl;
		//			cout<<"calculating IK....pos_error"<<pos_error<<endl;
		//		}
		//	}
		//	
		//	// via point �߰�
		//	ViaPoint *vp = &theApp.T_SW->m_vp[0];	// Right Arm

		//	if(vp->GetIndex() < 0) vp->SetIndex(0);	// List control�� ���� ������ ��..
	
		//	// vp mem ���ε�
		//	char* ch;
		//	char nullBuffer[50];	
		//	ch = itoa(vp->GetNum()+1, nullBuffer, 10);

		//	vp->SetRobotVP_grid(vp->GetNum(), &theApp.T_SW->nominal_robot[0], ch);
		//	//vp->SetRobotVP(vp->GetNum(), theApp.T_SW->m_robot[0], ch);
		//	//delete theApp.T_SW->nominal_robot[0];

		//	//vp->ShowList();

		//	LVITEM item;
		//	item.mask = LVIF_TEXT;
		//	
		//	// num ���
		//	item.iItem = vp->GetNum();
		//	item.iSubItem = 0;
		//	item.pszText = itoa(vp->GetNum(), nullBuffer, 10);
		//	theApp.pDlg->m_MainControlTab.m_vpList.InsertItem(&item);	// �ֻ� ��� �߰�
	

		//	if(theApp.GetNumRobot() == 1){
		//		//theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data, theApp.T_SW->Teaching_data_j);
		//		//theApp.T_SW->Teaching_data_j *= H_DtoR;

		//		//const char* fname_hash_p = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash.txt";
		//		//const char* fname_hash_j = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash_j.txt";
		//		//theApp.T_SW->save_hash_table(fname_hash_p, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
		//		//theApp.T_SW->save_hash_table(fname_hash_j, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
		//	}
		//	else if(theApp.GetNumRobot() == 2){
		//		theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data_R, theApp.T_SW->Teaching_data_j_R);
		//		theApp.T_SW->m_vp[1].Convert2Matrix(theApp.T_SW->Teaching_data_L, theApp.T_SW->Teaching_data_j_L);
		//		theApp.T_SW->Teaching_data_j_R *= H_DtoR;
		//		theApp.T_SW->Teaching_data_j_L *= H_DtoR;

		//		cout<<"We have to progress to save hash table for dual arm!"<<endl;
		//	}
		//
		//	// List control ���ε�
		//	theApp.pDlg->m_MainControlTab.vpListUpdate(0);	
	
		//	// Simulator update
		//	theApp.pDlg->m_MainControlTab.vpShowUpdate();
		//	printf("TeachStep: %d \n", Teachstep);

		//	Teachstep = 0; // Teaching�ܰ谡 ����.
		//	m_viaFlag = !m_viaFlag;	// �ٽ� false��..
		//	
		//	setIsometricView();
		//}

		// Teaching Step, 1: XY, 2: Z
		if (Teachstep == 1){
			// ù ��° �ܰ�(XY)
			// 1���� xy ��� ���� ���� ���� ����
			Teaching_data_grid.segment(0,3) = worldPos_grid;	// Teach Line�� �׸��� ����
			Teaching_data_grid(2) = 0.0;
		}
		else if (Teachstep == 2){
			// �� ��° �ܰ�(Yaw)
			// mouse��ǥ�� �ƴ� world ��ǥ �������� ���
			float x = worldPos_grid(0)-Teaching_data_grid(0);
			float y = worldPos_grid(1)-Teaching_data_grid(1);
			float theta = atan2(y, x);					// radian
			Teaching_data_grid(5) = (double)theta;		// rz
			setFrontView();
		}
		else if (Teachstep == 3){
			// �� ��° �ܰ�(Z)
			Teaching_data_grid(2) = worldPos_grid(2);// - theApp.T_SW->nominal_robot[0].GetOffset(2);
			
		}
		else if (Teachstep == 4){
			
			// mouse��ǥ�� �ƴ� world ��ǥ �������� ���
			float y = worldPos_grid(1)-Teaching_data_grid(1);
			float z = worldPos_grid(2)-Teaching_data_grid(2);
			float theta = atan2(z, y);				// radian
			Teaching_data_grid(3) = (double)theta;	// rx


			//// IK Ǯ��α�.. nominal robot model��
			Matrix4d temp_IKGoal;
			Vector3d temp_posGoal;
			temp_posGoal = Teaching_data_grid;
			temp_posGoal(2) = Teaching_data_grid(2) - theApp.T_SW->nominal_robot[0].GetOffset(2);
			Vector3d temp_oriGoal;
			temp_oriGoal = Teaching_data_grid.segment(3,3);
			theApp.T_SW->nominal_robot[0].MakeRobotTransform(temp_posGoal, temp_oriGoal, temp_IKGoal);
			
	
			// �Ʒ��� �Ϻκи� IK Ǯ�� ���� �� selective Matrix ���
			MatrixXd sMat(6, 6);
			sMat.setZero();
			sMat(0, 0) = 1.0; sMat(1, 1) = 1.0; sMat(2, 2) = 1.0;
			
			VectorXd IKdq(theApp.T_SW->nominal_robot[0].GetDOF());
			bool bIKfin = false;
			double pos_res = 1.0;
			double ori_res = 0.01;
			while(!bIKfin){
				double pos_error = (temp_IKGoal.col(3).segment(0,3) - theApp.T_SW->nominal_robot[0].GetTool().col(3).segment(0,3)).norm();
				Vector3d ori_error;
				ori_error(0) = 1.0 - temp_IKGoal.col(0).segment(0,3).dot(theApp.T_SW->nominal_robot[0].GetTool().col(0).segment(0,3));
				ori_error(1) = 1.0 - temp_IKGoal.col(1).segment(0,3).dot(theApp.T_SW->nominal_robot[0].GetTool().col(1).segment(0,3));
				ori_error(2) = 1.0 - temp_IKGoal.col(2).segment(0,3).dot(theApp.T_SW->nominal_robot[0].GetTool().col(2).segment(0,3));
				IKdq = theApp.T_SW->IKLoop_selective(&theApp.T_SW->nominal_robot[0], temp_IKGoal, pos_res, ori_res, pos_error, 0.0/*ori_error.norm()*/, sMat, bIKfin);
				for(int i=0; i<theApp.T_SW->nominal_robot[0].GetDOF(); i++)
					theApp.T_SW->nominal_robot[0].SetQ(i, theApp.T_SW->nominal_robot[0].GetQ(i) + IKdq(i)); 

				
				if(bIKfin == true){
					//// reach ������ via point ����.. �켱 joint space ����..
					//theApp.T_SW->m_vp[0].vp_Greach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					//theApp.T_SW->m_vp[0].vp_Greach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK �� Ǯ�� ���� �������� ����
					//cout<<"IK for vision data reach to grip finished"<<theApp.T_SW->m_vp[0].vp_Greach_q<<endl;
					cout<<"**********************"<<endl;
					cout<<"calculating IK....Q_nominal"<<theApp.T_SW->nominal_robot[0].GetQ()<<endl;
					cout<<"calculating IK....Q_original"<<theApp.T_SW->m_robot[0].GetQ()<<endl;
					cout<<"calculating IK....X"<<theApp.T_SW->nominal_robot[0].GetTool().col(3)<<endl;
				}
			}
			// via point �߰�
			ViaPoint *vp = &theApp.T_SW->m_vp[0];	// Right Arm

			if(vp->GetIndex() < 0) vp->SetIndex(0);	// List control�� ���� ������ ��..
	
			// vp mem ���ε�
			char* ch;
			char nullBuffer[50];	
			ch = itoa(vp->GetNum()+1, nullBuffer, 10);

			vp->SetRobotVP_grid(vp->GetNum(), &theApp.T_SW->nominal_robot[0], ch);
			//vp->SetRobotVP(vp->GetNum(), theApp.T_SW->m_robot[0], ch);
			//delete theApp.T_SW->nominal_robot[0];

			//vp->ShowList();

			LVITEM item;
			item.mask = LVIF_TEXT;
			
			// num ���
			item.iItem = vp->GetNum();
			item.iSubItem = 0;
			item.pszText = itoa(vp->GetNum(), nullBuffer, 10);
			theApp.pDlg->m_MainControlTab.m_vpList.InsertItem(&item);	// �ֻ� ��� �߰�
	
			if(theApp.GetNumRobot() == 1){
				//theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data, theApp.T_SW->Teaching_data_j);
				//theApp.T_SW->Teaching_data_j *= H_DtoR;

				//const char* fname_hash_p = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash.txt";
				//const char* fname_hash_j = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash_j.txt";
				//theApp.T_SW->save_hash_table(fname_hash_p, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
				//theApp.T_SW->save_hash_table(fname_hash_j, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
			}
			else if(theApp.GetNumRobot() == 2){
				theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data_R, theApp.T_SW->Teaching_data_j_R);
				theApp.T_SW->m_vp[1].Convert2Matrix(theApp.T_SW->Teaching_data_L, theApp.T_SW->Teaching_data_j_L);
				theApp.T_SW->Teaching_data_j_R *= H_DtoR;
				theApp.T_SW->Teaching_data_j_L *= H_DtoR;

				cout<<"We have to progress to save hash table for dual arm!"<<endl;
			}


			// orientation ����
			// via point�� ���� ���� ������ ��� �� �ȵǴ� �� ����...
			// ���� ������ �� ĭ�� �и��� ����!
			for(int i=0; i<theApp.GetNumRobot(); i++) {
				int index = theApp.T_SW->m_vp[i].GetNum()-1;
				index = index>0?index:0;
				theApp.T_SW->m_vp[i].SetVal(index, 3, Teaching_data_grid(3));
				theApp.T_SW->m_vp[i].SetVal(index, 4, Teaching_data_grid(4));
				theApp.T_SW->m_vp[i].SetVal(index, 5, Teaching_data_grid(5));
				printf("index: %d \n ", index );
			}

		
			// List control ���ε�
			theApp.pDlg->m_MainControlTab.vpListUpdate(0);	
	
			// Simulator update
			theApp.pDlg->m_MainControlTab.vpShowUpdate();
			printf("TeachStep: %d \n", Teachstep);



			Teachstep = 0; // Teaching�ܰ谡 ����.
			m_viaFlag = !m_viaFlag;	// �ٽ� false��..
			
			setIsometricView();
		}
	}
}


// Left, Right, Front, Top view ���� �ڵ�..
void COpenGLControl::setIsometricView()
{
	// �̳� ������ �� ���߿�....
	m_viewFlag = TRUE;
	double iangle = atan(1/sqrt(2.0))*(180/PI);
	m_desfRotX = iangle-90;		// y������ ȸ��..
	m_desfRotY	= -45.0;			// z������ ȸ��..
}



// ------------------------------------------------------------------------------------
// OpenGL�󿡼� Text ����� ���� �Լ� ����
void COpenGLControl::TextIntialize()
{
	// Text ��� �ʱ�ȭ -----------------
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	m_listBase = TextCreateBitmapFont("Comic Sans MS", 48);
}

unsigned int COpenGLControl::TextCreateBitmapFont(char *fontName, int fontsize)
{
	HFONT hFont;
	unsigned int base;
	
	base = glGenLists(96);	// 96���� ���ڵ��� ���� ���÷��� ��ϵ��� ����

	if(stricmp(fontName, "symbol")==0)
	{
		hFont = CreateFont(14, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, SYMBOL_CHARSET, 
		OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FF_DONTCARE | DEFAULT_PITCH, fontName);
	}
	else
	{
		hFont = CreateFont(14, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, ANSI_CHARSET, 
		OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FF_DONTCARE | DEFAULT_PITCH, fontName);
	}

	if(!hFont) return 0;

	SelectObject(hdc, hFont);
	wglUseFontBitmaps(hdc, 32, 96, base);

	return base;
}

void COpenGLControl::TextPrintString(unsigned int base, char *str)
{
	if((base == 0) || (str==NULL))
		return;

	glPushAttrib(GL_LIST_BIT);
		glListBase(base - 32);
		glCallLists(strlen(str), GL_UNSIGNED_BYTE, str);
	glPopAttrib();
}


void COpenGLControl::TextClearFont(unsigned int base)
{
	if(base != 0) glDeleteLists(base, 96);
}


// �� �׸���... 
void COpenGLControl::oglDrawCircle(float radius, float delta_theta, unsigned int plane_num)
{
	if(plane_num > 2) {
		printf("Wrong plane!! \n");
		return;
	}
		
	glBegin( GL_LINE_LOOP ); // OR GL_LINE_LOOP
	for( float angle = 0; angle < 2*PI; angle += delta_theta ) {
		switch(plane_num) {
			case 0:		// XY plane
				glVertex3f( radius*cos(angle), radius*sin(angle), 0 );
				break;
			case 1:		// YZ plane
				glVertex3f( 0, radius*cos(angle), radius*sin(angle));
				break;
			case 2:		// ZX plane
				glVertex3f(radius*sin(angle), 0, radius*cos(angle) );
				break;
		}
	}
	glEnd();
}




// ------------------------------------------------------------------------------------
// Outline Text ����� ���� �Լ� ����
void COpenGLControl::OLTextInitialize()
{
	//glClearColor(0.0f, 0.0f, 0.0f, 0.0f);	// ���������� �����.

	glShadeModel(GL_SMOOTH);				// �Ų����� ���̵��� ���
	glEnable(GL_DEPTH_TEST);				// ������ ���� ����
	//glEnable(GL_LIGHT0);					// Light0�� �Ҵ�.
	//glEnable(GL_LIGHTING);				// ������ Ȱ��ȭ
	glEnable(GL_COLOR_MATERIAL);			// ���� ������ Ȱ��ȭ

	m_listBase = OLTextCreateFont("Arial", 5, 0.25f);		// 10pt Arial �۲� ����
}

unsigned int COpenGLControl::OLTextCreateFont(char *fontName, int fontsize, float depth)
{
	HFONT hFont;
	unsigned int base;
	
	base = glGenLists(256);	// 96���� ���ڵ��� ���� ���÷��� ��ϵ��� ����

	if(stricmp(fontName, "symbol")==0)
	{
		hFont = CreateFont(14, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, SYMBOL_CHARSET, 
		OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FF_DONTCARE | DEFAULT_PITCH, fontName);
	}
	else
	{
		hFont = CreateFont(14, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, ANSI_CHARSET, 
		OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FF_DONTCARE | DEFAULT_PITCH, fontName);
	}

	if(!hFont) return 0;

	SelectObject(hdc, hFont);
	wglUseFontOutlines(hdc, 0, 255, base, 0.0f, depth, WGL_FONT_POLYGONS, gmf);

	return base;
}

void COpenGLControl::OLTextPrintString(unsigned int base, char *str)
{
	float length = 0;
	int idx;
	
	if((str==NULL)) return;

	// �ؽ�Ʈ�� �߾ӿ� ��ġ��Ų��. 
	for(idx = 0; idx<strlen(str); idx++)
	{
		length += gmf[str[idx]].gmfCellIncX;;	// �� ������ �ʺ� ���Ѵ�. 
	}
	glTranslatef(-length/2.0f, 0.0f, 0.0f);	// �ؽ�Ʈ�� �߾ӿ� ������ ��ǥ�踦 �̵�

	// �ؽ�Ʈ�� �׸���. 
	glPushAttrib(GL_LIST_BIT);
		glListBase(base);
		glCallLists(strlen(str), GL_UNSIGNED_BYTE, str);
	glPopAttrib();
}

void COpenGLControl::OLTextClearFont(unsigned int base)
{
	glDeleteLists(base, 256);
}




void COpenGLControl::TestFunc()
{
	// Orientation ���
	Vector3d eAngle;
	eAngle = theApp.T_SW->m_robot[0].GetOrientation("Ezyz");
	cout << "orientation ----------- \n" << eAngle << endl;

	
	for(int i=0; i<theApp.GetNumRobot(); i++)
	{
		VectorXd iq = VectorXd::Zero(theApp.T_SW->m_robot[i].GetDOF());
		theApp.T_SW->m_robot[i].SetQ(iq);
		theApp.T_SW->m_robot[i].CalcFK();

		Vector3d pos = theApp.T_SW->m_robot[i].GetPosition();
		Vector3d ori = theApp.T_SW->m_robot[i].GetOrientation("Ezyz");

		theApp.T_SW->m_robot[i].MakeRobotTransform(pos, ori, goal_T[i]);
	}

	// TODO
	MatrixXd P = MatrixXd::Zero(theApp.T_SW->m_robot[0].GetDOF(), 6);

	P = theApp.T_SW->m_robot[0].GetPointsforDraw();
	cout << P << endl;
}