#pragma once


// CIOTab ��ȭ �����Դϴ�.

class CIOTab : public CDialogEx
{
	DECLARE_DYNAMIC(CIOTab)

public:
	CIOTab(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~CIOTab();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_INPUT_OUTPUT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};
