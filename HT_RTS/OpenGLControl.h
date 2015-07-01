#pragma once
#include "afxwin.h"

#include "resource.h"


#include "HT_RTS.h"
//#include "TeachingSW_HM.h"

#define DtoR	M_PI/180	// degree to radian
#define RtoD	180/M_PI	// radian to degree


struct STCoord
{
	VectorXd des_WX;	// Desired World Coordinate
	VectorXd des_q;		// Desired Joint Coordinate
	VectorXd des_TX;	// Desired Tool Coordinate
};


using namespace Eigen;


#define FRONT_VIEW	1
#define LEFT_VIEW	2
#define RIGHT_VIEW	3
#define TOP_VIEW	4
#define ISO_VIEW	5


class COpenGLControl : public CWnd
{
	public:
		/******************/
		/* PUBLIC MEMBERS */
		/******************/
		// Timer
		UINT_PTR	m_unpTimer;
		// View information variables
		float	 m_fLastX;
		float	 m_fLastY;
		float	 m_fPosX;
		float	 m_fPosY;
		float	 m_fZoom;
		float	 m_fRotX;
		float	 m_fRotY;
		bool	 m_bIsMaximized;

		BOOL	m_AltOn;
		BOOL	m_ShiftOn;
		BOOL	m_viewFlag;

		
		STCoord		m_RobotPos;
		BOOL		onIKstart;		// IK on/off flag 변수

		// 로봇 개수에 영향 받음
		BOOL		*onProcessing;		// IK가 실행 중인지를 나타내는 변수
		Matrix4d	*goal_T;			// desired position

	private:
		float	 m_desfPosX;
		float	 m_desfPosY;
		float	 m_desfZoom;
		float	 m_desfRotX;
		float	 m_desfRotY;
		float	 m_gain;
		float	 m_thres;




	private:
		// Window information
		CWnd	*hWnd;
		HDC		hdc;
		HGLRC	hrc;
		int		m_nPixelFormat;
		CRect	m_rect;
		CRect	m_oldWindow;
		CRect	m_originalRect;
		BOOL	m_viaShow;

		// for vT
		BOOL	m_viaFlag;
		VectorXd worldPos;
		Vector3d worldPos_grid;
		VectorXd Teaching_data_grid;
		int		m_gap;
		int		Teachstep;
		


		//// Thread 함수
		//CWinThread* pThread;
		//static UINT ThreadUpdateView(LPVOID pParam);

		
		// Text 출력 변수
		unsigned int m_listBase;
		
	public:
		// Via Point 변수
		// CViaPoint vpList;  // 그리기 위해서..
		int *selectedVP;

	public:
		COpenGLControl(void);
		virtual ~COpenGLControl(void);

		void oglCreate(CRect rect, CWnd *parent);
		void oglInitialize(void);
		void oglDrawScene(void);
		void setInitialView(void);
		void setIsometricView();
		void oglDrawRobot(T_Robot &_m_Robot);	// Robot 그리기
		void oglDrawBasis(void);		// Basis coordinate 그리기
		void oglDrawViaPoint(void);		// via-point 그리기
		void oglDrawViaTeaching(void);	// via-point teaching drawing
		


		// for vT
		void oglDrawGrid(void);
		void setTopView(void);
		void setFrontView(void);
		void setLeftView(void);

		void setViewUP(void);
		void setViewDOWN(void);
		void setViewLEFT(void);
		void setViewRIGHT(void);
		
		// ** A function for drawing circle in specified plnae _ 0: XY, 1: YZ, 2: ZX */
		void oglDrawCircle(float radius, float delta_theta=0.02, unsigned int plane_num=0);	
		
		
		
		BOOL GetonProcess(int nRobot);	// IK process가 실행중인지를 나타내는 변수 얻기


		// 시뮬레이터 로봇의 Cartesian coordinate Inverse Kinematics계산
		void SetinitGoal();


		// Text 출력을 위한 함수들 정의
		unsigned int TextCreateBitmapFont(char* fontName, int fontSize);
		void TextPrintString(unsigned int base, char*str);
		void TextClearFont(unsigned int base);
		void TextIntialize();

		// Outline Text 출력을 위한 함수들 정의
		unsigned int OLTextCreateFont(char *fontName, int fontSize, float depth);
		void OLTextPrintString(unsigned int base, char *str);
		void OLTextClearFont(unsigned int base);
		void OLTextInitialize();
		float angle;
		GLYPHMETRICSFLOAT gmf[256];

		// Cartesian Movement관련 함수
		void SetDesPosition(int coordinate, float increment, int nRobot);

		// Joint-Task sync
		void SetGoalPos(int nRobot, Vector3d pos, Vector3d ori);	// Goal position&orientation setting
		void SetGoalPos(int nRobot, Transform<double, 3, 2> *_HT_target);
		void SetDesPositionByJoint(int jNum, float increment);
		void SetDesPositionByJoint();
		void RobotSetup(int robot, T_Robot &_m_Robot);
		VectorXd InverseKinematics(Matrix4d &_goal_T, T_Robot &_m_Robot, int nRobot);
		//int GetNumRobot();
		void TestFunc();

		// Visualization 관련 함수들
		void viaShow();
		void viaHide();


		afx_msg void OnPaint();
		afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
		afx_msg	void OnDraw(CDC *pDC);
		afx_msg void OnTimer(UINT_PTR nIDEvent);
		afx_msg void OnSize(UINT nType, int cx, int cy);
		afx_msg void OnMouseMove(UINT nFlags, CPoint point);

		DECLARE_MESSAGE_MAP()
		afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
		afx_msg void OnLButtonDown(UINT nFlags, CPoint point);


		/** grid에 click을 통해 rough teaching 하기 위한.. 애매한 곳을 click해도 grid point로 잡아주기 위해서 */
		void setGridPointbyClick(Vector3d _input, int _scale, int _grid_coord[]);

};

