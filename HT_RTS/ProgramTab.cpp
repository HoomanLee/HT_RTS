// ProgramTab.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "ProgramTab.h"
#include "afxdialogex.h"


// CProgramTab ��ȭ �����Դϴ�.

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


// CProgramTab �޽��� ó�����Դϴ�.
