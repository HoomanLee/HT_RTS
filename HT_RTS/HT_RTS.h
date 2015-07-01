
// HT_RTS.h : PROJECT_NAME 응용 프로그램에 대한 주 헤더 파일입니다.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH에 대해 이 파일을 포함하기 전에 'stdafx.h'를 포함합니다."
#endif

#include "resource.h"		// 주 기호입니다.

#include "stdafx.h"
//#include "TeachingSW_HM.h"


// CHT_RTSApp:
// 이 클래스의 구현에 대해서는 HT_RTS.cpp을 참조하십시오.
//
class CHT_RTSDlg;	// 클래스 암시
class CHT_RTSApp : public CWinApp
{
public:
	CHT_RTSApp();
	~CHT_RTSApp();

// 재정의입니다.
public:
	virtual BOOL InitInstance();

// 구현입니다.
	
	CHT_RTSDlg *pDlg;


	// TeachingSW_HM instance
	TeachingSW_HM *T_SW;
	unsigned int numRobot;

	int *rModel;

	int GetNumRobot();


	DECLARE_MESSAGE_MAP()
	virtual BOOL PreTranslateMessage(MSG* pMsg);
};

extern CHT_RTSApp theApp;