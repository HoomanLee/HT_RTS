#ifndef __IMAGEDISP
#define __IMAGEDISP

// ImageDisp.h : header file
//

#pragma once
#include "CDibN.h"
#include <math.h>

/////////////////////////////////////////////////////////////////////////////
// CImageDisp window

#define ZM_NONE             0   // No Zooming
#define ZM_SHRINKONLY       1   // Only Shrinking (No Enlargement)
#define ZM_FITTOWINDOW      2   // Fit to the Window
#define ZM_FITTOWINDOWNRA   3   // Fit to the Window (not reserving aspect ratio)

#define RF_NO           0   // disable refresh background
#define RF_YES          1   // enable refresh background
#define RF_AUTO         2   // automatic selection (새로 load 되는 영상의 크기가 바뀌는 경우에만 TRUE)
class CImageDisp : public CStatic
{
// Construction
public:
	CImageDisp();

// Attributes
public:
    CDib *m_pDib;
    int m_ZoomMode;
    BOOL m_bCentering;
    BOOL m_RefreshMode;


private:
    double m_ZoomX;
    double m_ZoomY;

	double m_iZoom;	// initial zoom constant

    CRect m_Disp;

    CSize m_OriginalSize;

	// for drawing rect by draging
	bool m_drawRect;
	CPoint m_clipStart;
	CPoint m_clipEnd;
	CPoint m_PointDepth;	// depth value at specific point
	CPoint m_PointImgProc;	// imgProc에서의 clicked point

	int m_currWnd;	// 현재 마우스가 위치한 커서가 어떤 것인지를 나타내줌
	

// Operations
public:
    void SetDib(CDib *pDib);
    inline CDib *GetDib();
    void DisplayWithoutZoom(CDC *pDC);
    void DisplayShrinkOnly(CDC *pDC);
    void DisplayFitToWindow(CDC *pDC);
    void DisplayFitToWindowNRA(CDC *pDC);
    void ClientToImageCoordinate(CPoint *pPoint);
    void ClientToImageCoordinate(CRect *pRect);
    void ImageToClientCoordinate(CPoint *pPoint);
    void ImageToClientCoordinate(CRect *pRect);
	void calcInitZoom();
	void ImageToinitClientCoordinate(CRect *pRect);
	void SetCurrWindow();
	void DrawCross(CPoint point);

	/////////////////////////////////////////////////
	// defined by T.W.Kim
	CPoint GetGlobalCoordinate(CPoint pt);

    inline void SetZoomMode(int mode);
    inline int GetZoomMode();
    inline void SetCentering(BOOL bEnable);
    inline BOOL GetCentering();
    inline void SetRefresh(int refresh);
    inline int GetRefresh();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CImageDisp)
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CImageDisp();

	// Generated message map functions
protected:
	//{{AFX_MSG(CImageDisp)
	afx_msg void OnPaint();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
};

inline CDib *CImageDisp::GetDib()
{
    return m_pDib;
}

inline void CImageDisp::SetZoomMode(int mode)
{
    m_ZoomMode = mode;
    if (m_RefreshMode) InvalidateRect(NULL, TRUE);
    else InvalidateRect(NULL, FALSE);
}

inline int CImageDisp::GetZoomMode()
{
    return m_ZoomMode;
}

inline void CImageDisp::SetCentering(BOOL bEnable)
{
    m_bCentering = bEnable;
    if (m_RefreshMode) InvalidateRect(NULL, TRUE);
    else InvalidateRect(NULL, FALSE);
}

inline BOOL CImageDisp::GetCentering()
{
    return m_bCentering;
}

inline void CImageDisp::SetRefresh(int refresh)
{
    m_RefreshMode = refresh;
    //if (m_RefreshMode) InvalidateRect(NULL, TRUE);
    //else InvalidateRect(NULL, FALSE);
}

inline int CImageDisp::GetRefresh()
{
    return m_RefreshMode;
}

///////////////////////////////////////////////////////////////////////////////
//// CImageDispButton window
//#define TRACKMODE_TRACK			1
//#define TRACKMODE_MOVE			2
//#define TRACKMODE_CLICK			3
//#define TRACKMODE_SAMPLE		4
//#define TRACKMODE_UBOXTRACK		5	// TRACKMODE_TRACK과 마찬가지로 사각형 영역을 지정하는 거지만, Parent CUnitBoxPage인 경우
//class CImageDispButton : public CButton
//{
//// Construction
//public:
//	CImageDispButton();
//
//// Attributes
//public:
//    CDib *m_pDib;
//    int m_ZoomMode;
//    BOOL m_bCentering;
//    int m_RefreshMode;
//
//    double m_ZoomX;
//    double m_ZoomY;
//    CRect m_Disp;
//
//    CSize m_OriginalSize;
//
//	BOOL m_bClipSet;
//	int m_TrackMode;
//	CRect m_ClipRect;
//	BOOL m_bMoving;
//	CPoint m_PrevPoint;
//
//// Operations
//public:
//    void SetDib(CDib *pDib);
//    inline CDib *GetDib();
//    void DisplayWithoutZoom(CDC *pDC);
//    void DisplayShrinkOnly(CDC *pDC);
//    void DisplayFitToWindow(CDC *pDC);
//    void DisplayFitToWindowNRA(CDC *pDC);
//    CPoint GetImageCoordinate(CPoint point);
//    void ClientToImageCoordinate(CPoint *pPoint);
//    void ClientToImageCoordinate(CRect *pRect);
//    void ImageToClientCoordinate(CPoint *pPoint);
//    void ImageToClientCoordinate(CRect *pRect);
//
//    inline void SetZoomMode(int mode);
//    inline int GetZoomMode();
//    inline void SetCentering(BOOL bEnable);
//    inline BOOL GetCentering();
//    inline void SetRefresh(int refresh);
//    inline int GetRefresh();
//
//	inline void SetClipRect(CRect rect);
//	inline CRect GetClipRect();
//	inline void SetTrackMode(int mode);
//	inline int GetTrackMode();
//
//// Overrides
//	// ClassWizard generated virtual function overrides
//	//{{AFX_VIRTUAL(CImageDispButton)
//	public:
//	virtual void DrawItem(LPDRAWITEMSTRUCT lpDrawItemStruct);
//	//}}AFX_VIRTUAL
//
//// Implementation
//public:
//	virtual ~CImageDispButton();
//
//	// Generated message map functions
//protected:
//	//{{AFX_MSG(CImageDispButton)
//	afx_msg void OnPaint();
//	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
//	//}}AFX_MSG
//
//	DECLARE_MESSAGE_MAP()
//public:
//	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
//	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
//	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
//};
//
//inline CDib *CImageDispButton::GetDib()
//{
//    return m_pDib;
//}
//
//inline void CImageDispButton::SetZoomMode(int mode)
//{
//    m_ZoomMode = mode;
//    if (m_RefreshMode) InvalidateRect(NULL, TRUE);
//    else InvalidateRect(NULL, FALSE);
//}
//
//inline int CImageDispButton::GetZoomMode()
//{
//    return m_ZoomMode;
//}
//
//inline void CImageDispButton::SetCentering(BOOL bEnable)
//{
//    m_bCentering = bEnable;
//    if (m_RefreshMode) InvalidateRect(NULL, TRUE);
//    else InvalidateRect(NULL, FALSE);
//}
//
//inline BOOL CImageDispButton::GetCentering()
//{
//    return m_bCentering;
//}
//
//inline void CImageDispButton::SetRefresh(int refresh)
//{
//    m_RefreshMode = refresh;
//    if (m_RefreshMode) InvalidateRect(NULL, TRUE);
//    else InvalidateRect(NULL, FALSE);
//}
//
//inline int CImageDispButton::GetRefresh()
//{
//    return m_RefreshMode;
//}
//
//inline void CImageDispButton::SetClipRect(CRect rect)
//{
//	m_ClipRect = rect;
//	m_bClipSet = TRUE;
//}
//
//inline CRect CImageDispButton::GetClipRect()
//{
//	return m_ClipRect;
//}
//
//inline void CImageDispButton::SetTrackMode(int mode)
//{
//	m_TrackMode = mode;
//}

#endif
