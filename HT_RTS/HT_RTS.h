
// HT_RTS.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.

#include "stdafx.h"
//#include "TeachingSW_HM.h"


// CHT_RTSApp:
// �� Ŭ������ ������ ���ؼ��� HT_RTS.cpp�� �����Ͻʽÿ�.
//
class CHT_RTSDlg;	// Ŭ���� �Ͻ�
class CHT_RTSApp : public CWinApp
{
public:
	CHT_RTSApp();
	~CHT_RTSApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.
	
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