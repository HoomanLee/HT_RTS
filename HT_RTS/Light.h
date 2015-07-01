#if !defined(AFX_LIGHT_H__63089781_0E2D_460F_8AC6_BD12BBE3076D__INCLUDED_)
#define AFX_LIGHT_H__63089781_0E2D_460F_8AC6_BD12BBE3076D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Light.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CLight window
#define LIGHT_RED               0
#define LIGHT_YELLOW            1
#define LIGHT_GREEN             2
#define LIGHT_USERDEFINED       3
class CLight : public CStatic
{
// Construction
public:
	CLight();

// Attributes
public:
    static BOOL m_bBrushCreated;
    static CBrush m_Brush[3];
    int m_Mode;
    COLORREF m_Color;

// Operations
public:
    inline int GetMode();
    inline void SetMode(int mode);
    inline void SetColor(COLORREF color);
    COLORREF GetColor();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CLight)
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CLight();

	// Generated message map functions
protected:
	//{{AFX_MSG(CLight)
	afx_msg void OnPaint();
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
};

inline void CLight::SetColor(COLORREF color)
{
    m_Mode = LIGHT_USERDEFINED;
    m_Color = color;
    InvalidateRect(NULL, TRUE);
}

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_LIGHT_H__63089781_0E2D_460F_8AC6_BD12BBE3076D__INCLUDED_)

inline int CLight::GetMode()
{
    return m_Mode;
}

inline void CLight::SetMode(int mode)
{
    m_Mode = mode;
    InvalidateRect(NULL, FALSE);
}
