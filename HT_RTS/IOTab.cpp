// IOTab.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "IOTab.h"
#include "afxdialogex.h"


// CIOTab 대화 상자입니다.

IMPLEMENT_DYNAMIC(CIOTab, CDialogEx)

CIOTab::CIOTab(CWnd* pParent /*=NULL*/)
	: CDialogEx(CIOTab::IDD, pParent)
{

}

CIOTab::~CIOTab()
{
}

void CIOTab::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CIOTab, CDialogEx)
END_MESSAGE_MAP()


// CIOTab 메시지 처리기입니다.
