
// stdafx.h : ���� ��������� ���� ��������� �ʴ�
// ǥ�� �ý��� ���� ���� �� ������Ʈ ���� ���� ������ 
// ��� �ִ� ���� �����Դϴ�.

#pragma once

#ifndef _SECURE_ATL
#define _SECURE_ATL 1
#endif

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // ���� ������ �ʴ� ������ Windows ������� �����մϴ�.
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // �Ϻ� CString �����ڴ� ��������� ����˴ϴ�.

// MFC�� ���� �κа� ���� ������ ��� �޽����� ���� ����⸦ �����մϴ�.
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC �ٽ� �� ǥ�� ���� ����Դϴ�.
#include <afxext.h>         // MFC Ȯ���Դϴ�.


#include <afxdisp.h>        // MFC �ڵ�ȭ Ŭ�����Դϴ�.



#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // Internet Explorer 4 ���� ��Ʈ�ѿ� ���� MFC �����Դϴ�.
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // Windows ���� ��Ʈ�ѿ� ���� MFC �����Դϴ�.
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // MFC�� ���� �� ��Ʈ�� ���� ����



// Control style -> XP style..
// #ifdef -> #ifndef�� �ٲٸ� ��Ƽ����Ʈ ���� ������ ����ϸ鼭 ���ÿ� ��Ʈ���� xp��Ÿ�Ϸ� �ٲ� �� �ִ�. 
#ifndef _UNICODE	// ���� �ڵ�: #ifdef
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif




//////////////////////////////////////////
// ����� ��� �� ��� �߰�..

BOOL WINAPI GetModulePath(CString& strPath);
BOOL TestDLL();
#pragma comment(lib, "KeyHook")	// dll, lib �̸� ����

// Console ���
#pragma comment(linker, "/entry:WinMainCRTStartup /subsystem:console")


// socket ��� �߰�
#include <afxsock.h>
#include <Dense>

#include "LargeText.h"
#include "afxcmn.h"

#include "TeachingSW_HM.h"

#define PC		0
#define OMNI	1

#define CART	0
#define JOINT	1
#define TOOL	2


// �κ� ��ġ ����
typedef struct _Coordi {

	// **************************************************************
	// inputmode�� PC�� ���� thread���� ��� ������ ���� �ƴ϶�
	// ��ư �̺�Ʈ�� �߻��� ��쿡�� �κ��� ����� ����. 
	// 
	// inputmode�� OMNI�� ���� thread���� �κ��� continous�ϰ� 
	// �̵� ����� ����
	// **************************************************************

	int inputmode;	
	int coordflag;
	BOOL BtnEvent;

	// Cartesian
	double x;
	double y;
	double z;
	double rx;
	double ry;
	double rz;
	
	// Joint
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;
	double j6;

	// Tool
	double xt;
	double yt;
	double zt;
	double rxt;
	double ryt;
	double rzt;


	// Constructor
	_Coordi()
	{
		inputmode = PC;
		coordflag = CART;
		BtnEvent = FALSE;

		x = 0.0;
		y = 0.0;
		z = 0.0;
		rx = 0.0;
		ry = 0.0;
		rz = 0.0;

		j1 = 0.0;
		j2 = 0.0;
		j3 = 0.0;
		j4 = 0.0;
		j5 = 0.0;
		j6 = 0.0;

		xt = 0.0;
		yt = 0.0;
		zt = 0.0;
		rxt = 0.0;
		ryt = 0.0;
		rzt = 0.0;
	}

}Coordi;