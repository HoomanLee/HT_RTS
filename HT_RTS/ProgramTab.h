#pragma once


// CProgramTab ��ȭ �����Դϴ�.

class CProgramTab : public CDialogEx
{
	DECLARE_DYNAMIC(CProgramTab)

public:
	CProgramTab(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CProgramTab();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_PROGRAM };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};
