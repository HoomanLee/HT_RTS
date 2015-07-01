#pragma once


// CProgramTab 대화 상자입니다.

class CProgramTab : public CDialogEx
{
	DECLARE_DYNAMIC(CProgramTab)

public:
	CProgramTab(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CProgramTab();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_PROGRAM };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()
};
