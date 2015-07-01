#pragma once


// CIOTab 대화 상자입니다.

class CIOTab : public CDialogEx
{
	DECLARE_DYNAMIC(CIOTab)

public:
	CIOTab(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~CIOTab();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_INPUT_OUTPUT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()
};
