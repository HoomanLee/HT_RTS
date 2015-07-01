// ProgramTab.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "ProgramTab.h"
#include "afxdialogex.h"


// CProgramTab 대화 상자입니다.

IMPLEMENT_DYNAMIC(CProgramTab, CDialogEx)

CProgramTab::CProgramTab(CWnd* pParent /*=NULL*/)
	: CDialogEx(CProgramTab::IDD, pParent)
{

}

CProgramTab::~CProgramTab()
{
}

void CProgramTab::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CProgramTab, CDialogEx)
END_MESSAGE_MAP()


// CProgramTab 메시지 처리기입니다.
