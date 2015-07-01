// LargeText.cpp : implementation file
//

#include "stdafx.h"
#include "LargeText.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CLargeText

CLargeText::CLargeText()
{
    m_TextColor = RGB(0,0,0);
    m_BkColor = RGB(255,255,255);
    m_FontSize = 500;
    m_bHorizontalCenter = TRUE;
    m_bVerticalCenter = TRUE;
}

CLargeText::~CLargeText()
{
}


BEGIN_MESSAGE_MAP(CLargeText, CStatic)
	//{{AFX_MSG_MAP(CLargeText)
	ON_WM_PAINT()
	ON_WM_ERASEBKGND()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CLargeText message handlers

void CLargeText::OnPaint() 
{
	CPaintDC dc(this); // device context for painting
	
	// TODO: Add your message handler code here
	//if (m_Caption.GetLength()==0) return;

    CFont font;
	font.CreatePointFont(m_FontSize, "±¼¸²");
	CFont *pOldFont = dc.SelectObject(&font);
    dc.SetTextColor(m_TextColor);
    
    
	CRect rect; GetClientRect(&rect);
    /*
    CBrush brush(m_BkColor);
    CBrush *pOldBrush = (CBrush *) dc.SelectObject(&brush);
	dc.Rectangle(&rect);
    dc.SelectObject(pOldBrush);
    */
    dc.SetBkMode(TRANSPARENT);

	rect.InflateRect(-2, -7);
    int format=0;
    if (m_bHorizontalCenter) format |= DT_CENTER;
    if (m_bVerticalCenter) format |= (DT_SINGLELINE | DT_VCENTER);
	dc.DrawText(m_Caption, &rect, format);

	dc.SelectObject(pOldFont);
	
	// Do not call CStatic::OnPaint() for painting messages
}

BOOL CLargeText::OnEraseBkgnd(CDC* pDC) 
{
	// TODO: Add your message handler code here and/or call default
	CBrush brush(m_BkColor);
    CBrush *pOldBrush = (CBrush *) pDC->SelectObject(&brush);

    CRect client; GetClientRect(&client);

    pDC->Rectangle(&client);

    pDC->SelectObject(pOldBrush);

	return CStatic::OnEraseBkgnd(pDC);
}
