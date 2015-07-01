// Light.cpp : implementation file
//

#include "stdafx.h"
#include "Light.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

BOOL CLight::m_bBrushCreated = FALSE;
CBrush CLight::m_Brush[3];

/////////////////////////////////////////////////////////////////////////////
// CLight

CLight::CLight()
{
    if (!m_bBrushCreated) {
      m_Brush[0].CreateSolidBrush(RGB(255,0,0));
      m_Brush[1].CreateSolidBrush(RGB(255,255,0));
      m_Brush[2].CreateSolidBrush(RGB(0,255,0));
      m_bBrushCreated = TRUE;
    }
    m_Mode = 0;
    m_Color = RGB(255,255,255);
}

CLight::~CLight()
{
}


BEGIN_MESSAGE_MAP(CLight, CStatic)
	//{{AFX_MSG_MAP(CLight)
	ON_WM_PAINT()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// Operations
COLORREF CLight::GetColor()
{
    switch (m_Mode) {
      case LIGHT_RED:
        return RGB(255,0,0);
      case LIGHT_YELLOW:
        return RGB(255,255,0);
      case LIGHT_GREEN:
        return RGB(0,255,0);
      case LIGHT_USERDEFINED:
      default:
        return m_Color;
    }
}

/////////////////////////////////////////////////////////////////////////////
// CLight message handlers

void CLight::OnPaint() 
{
	CPaintDC dc(this); // device context for painting
	
	// TODO: Add your message handler code here
	CRect client; GetClientRect(&client);
    CBrush *pOld;
    CBrush brush;
    if (m_Mode<=2) pOld = (CBrush *) dc.SelectObject(&m_Brush[m_Mode]);
    else {
      brush.CreateSolidBrush(m_Color);
      pOld = (CBrush *) dc.SelectObject(&brush);
    }

    dc.Ellipse(&client);
    dc.SelectObject(pOld);
	// Do not call CStatic::OnPaint() for painting messages
}
