
// stdafx.h : 자주 사용하지만 자주 변경되지는 않는
// 표준 시스템 포함 파일 및 프로젝트 관련 포함 파일이 
// 들어 있는 포함 파일입니다.

#pragma once

#ifndef _SECURE_ATL
#define _SECURE_ATL 1
#endif

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 거의 사용되지 않는 내용은 Windows 헤더에서 제외합니다.
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 일부 CString 생성자는 명시적으로 선언됩니다.

// MFC의 공통 부분과 무시 가능한 경고 메시지에 대한 숨기기를 해제합니다.
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC 핵심 및 표준 구성 요소입니다.
#include <afxext.h>         // MFC 확장입니다.


#include <afxdisp.h>        // MFC 자동화 클래스입니다.



#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // Internet Explorer 4 공용 컨트롤에 대한 MFC 지원입니다.
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // Windows 공용 컨트롤에 대한 MFC 지원입니다.
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // MFC의 리본 및 컨트롤 막대 지원



// Control style -> XP style..
// #ifdef -> #ifndef로 바꾸면 멀티바이트 문자 집합을 사용하면서 동시에 컨트롤을 xp스타일로 바꿀 수 있다. 
#ifndef _UNICODE	// 원래 코드: #ifdef
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif




//////////////////////////////////////////
// 사용자 헤더 및 기능 추가..

BOOL WINAPI GetModulePath(CString& strPath);
BOOL TestDLL();
#pragma comment(lib, "KeyHook")	// dll, lib 이름 지정

// Console 출력
#pragma comment(linker, "/entry:WinMainCRTStartup /subsystem:console")


// socket 헤더 추가
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


// 로봇 위치 정보
typedef struct _Coordi {

	// **************************************************************
	// inputmode가 PC일 때는 thread에서 계속 보내는 것이 아니라
	// 버튼 이벤트가 발생할 경우에만 로봇에 명령을 전달. 
	// 
	// inputmode가 OMNI일 떄는 thread에서 로봇에 continous하게 
	// 이동 명령을 내림
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