#if !defined(AFX_LARGETEXT_H__A4407FFF_F7FB_4CDC_93CB_873CD7F22EF1__INCLUDED_)
#define AFX_LARGETEXT_H__A4407FFF_F7FB_4CDC_93CB_873CD7F22EF1__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// LargeText.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CLargeText window

class CLargeText : public CStatic
{
// Construction
public:
	CLargeText();

// Attributes
public:
	CString m_Caption;
    COLORREF m_TextColor;
    COLORREF m_BkColor;
    int m_FontSize;
    BOOL m_bHorizontalCenter;
    BOOL m_bVerticalCenter;

// Operations
public:
	inline CString GetCaption();
	inline void SetCaption(const char *pCaption);
    inline void SetBackgroundColor(COLORREF color);
    inline COLORREF GetBackgroundColor();
    inline void SetTextColor(COLORREF color);
    inline COLORREF GetTextColor();
    inline void SetFontSize(int size);
    inline int GetFontSize();
    inline void SetHorizontalCenter(BOOL bEnable);
    inline BOOL GetHorizontalCenter();
    inline void SetVerticalCenter(BOOL bEnable);
    inline BOOL GetVerticalCenter();


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CLargeText)
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CLargeText();

	// Generated message map functions
protected:
	//{{AFX_MSG(CLargeText)
	afx_msg void OnPaint();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
};

inline void CLargeText::SetBackgroundColor(COLORREF color)
{
    m_BkColor = color;
    InvalidateRect(NULL, TRUE);
}

inline COLORREF CLargeText::GetBackgroundColor()
{
    return m_BkColor;
}

inline void CLargeText::SetTextColor(COLORREF color)
{
    m_TextColor = color;
    InvalidateRect(NULL, TRUE);
}

inline COLORREF CLargeText::GetTextColor()
{
    return m_TextColor;
}

inline void CLargeText::SetFontSize(int size)
{
    m_FontSize = size;
    InvalidateRect(NULL, TRUE);
}

inline int CLargeText::GetFontSize()
{
    return m_FontSize;
}

inline CString CLargeText::GetCaption()
{
	return m_Caption;
}

inline void CLargeText::SetCaption(const char *pCaption)
{
	m_Caption = pCaption;
	InvalidateRect(NULL, TRUE);
}

inline void CLargeText::SetHorizontalCenter(BOOL bEnable)
{
    m_bHorizontalCenter = bEnable;
}

inline BOOL CLargeText::GetHorizontalCenter()
{
    return m_bHorizontalCenter;
}

inline void CLargeText::SetVerticalCenter(BOOL bEnable)
{
    m_bVerticalCenter = bEnable;
}

inline BOOL CLargeText::GetVerticalCenter()
{
    return m_bVerticalCenter;
}

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_LARGETEXT_H__A4407FFF_F7FB_4CDC_93CB_873CD7F22EF1__INCLUDED_)

