// ImageDisp.cpp : implementation file
//
#pragma once

#include "stdafx.h"
#include "ImageDisp.h"
//#include "resource.h"

#include "HT_RTS.h"
//#include "HT_RTSDlg.h"
#include "VisionTab.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


enum CURR_WND
{
	NULL_WND,
	RGB_WND,
	DEPTH_WND,
	IMGPROC_WND,
};




/////////////////////////////////////////////////////////////////////////////
// CImageDisp

CImageDisp::CImageDisp()
{
    m_pDib = NULL;

    //m_ZoomMode = ZM_SHRINKONLY;
	m_ZoomMode = ZM_FITTOWINDOW;
    m_bCentering = FALSE;
    m_RefreshMode = RF_AUTO;

	m_drawRect = FALSE;	// �簢���� �׸��� ����

	// picture control�� clip �̹��� size �ʱ�ȭ
	m_clipStart.x = 0;
	m_clipStart.y = 0;
	m_iZoom = 0.0;

	m_currWnd = NULL_WND;
	m_PointDepth.x = -1;
	m_PointDepth.y = -1;
}

CImageDisp::~CImageDisp()
{
    if (m_pDib) {
      delete m_pDib;
      m_pDib = NULL;
    }
}

BEGIN_MESSAGE_MAP(CImageDisp, CStatic)
	//{{AFX_MSG_MAP(CImageDisp)
	ON_WM_PAINT()
	ON_WM_ERASEBKGND()
	//}}AFX_MSG_MAP
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// Operations
void CImageDisp::SetDib(CDib *pDib)
{
	if (m_pDib) {delete m_pDib; m_pDib=NULL;}
    m_pDib = pDib;

    if (!m_pDib) return;

	calcInitZoom();	// ó�� �� ���� �����


    CSize nsize(m_pDib->Width(), m_pDib->Height());

    switch (m_RefreshMode) {
      case RF_NO: 
        InvalidateRect(NULL, FALSE);
        break;
      case RF_YES:
        InvalidateRect(NULL, TRUE);
        break;
      case RF_AUTO:
        if (m_OriginalSize==nsize) InvalidateRect(NULL, FALSE);
        else InvalidateRect(NULL, TRUE);
        break;
      default:
        break;
    }

    m_OriginalSize = nsize;

}

void CImageDisp::DisplayWithoutZoom(CDC *pDC)
{
    CRect client; GetClientRect(&client);

    m_ZoomX = m_ZoomY = 1.;
    m_Disp.SetRect(0, 0, min(m_pDib->Width(), client.Width()), min(m_pDib->Height(), client.Height()));

    if ((m_pDib->Width()>client.Width()) || (m_pDib->Height()>client.Height())) {
      CRect target;
      target.SetRect(0, 0, min(m_pDib->Width(), client.Width()), min(m_pDib->Height(), client.Height()));
      if (m_bCentering) {
        CPoint Scroll(0,0);
        if (client.Width()>m_pDib->Width()) Scroll.x = (client.Width()-m_pDib->Width())/2;
        if (client.Height()>m_pDib->Height()) Scroll.y = (client.Height()-m_pDib->Height())/2;
        target += Scroll;
      }
      m_Disp = target;

      CRect source;
      if (m_pDib->Height()>client.Height()) source.SetRect(0, m_pDib->Height()-client.Height(), min(m_pDib->Width(), client.Width()), m_pDib->Height());
      else source = target;
      m_pDib->LoadPaletteImage(pDC, &target, &source);
    }
    else {
      if (m_bCentering) {
        CPoint Scroll(0,0);
        Scroll.x = (client.Width()-m_pDib->Width())/2;
        Scroll.y = (client.Height()-m_pDib->Height())/2;
        m_Disp += Scroll;
        m_pDib->LoadPaletteImage(pDC, &m_Disp);
      }
      else m_pDib->LoadPaletteImage(pDC);
    }
}

void CImageDisp::DisplayShrinkOnly(CDC *pDC)
{
    CRect client; GetClientRect(&client);

    if ((client.Width()<m_pDib->Width()) || (client.Height()<m_pDib->Height())) {
      double xz = (double) client.Width() / (double) m_pDib->Width();
      double yz = (double) client.Height() / (double) m_pDib->Height();
      xz = min(xz, yz);
      yz = xz;
      m_ZoomX = xz;
      m_ZoomY = yz;
      int width = (int) (((double) m_pDib->Width()) * xz);
      int height = (int) (((double) m_pDib->Height()) * yz);
      CRect disp;
      if (m_bCentering) {
        disp.left = (client.Width() - width)/2;
        disp.top = (client.Height() - height)/2;
        disp.right = disp.left + width;
        disp.bottom = disp.top + height;
      }
      else {
        disp.left = 0; 
        disp.top = 0; 
        disp.right = width; 
        disp.bottom = height; 
      }
      m_Disp = disp;

      pDC->SetStretchBltMode(COLORONCOLOR);
      m_pDib->LoadPaletteImage(pDC, &disp);	
    }
    else DisplayWithoutZoom(pDC);


	// --------------------------------------------------------
	// ------------------------twkim---------------------------
	// --------------------------------------------------------
	// Clip ���� �ǽð����� �׸���
	//CVisionTab *pVTDlg = (CVisionTab*)::AfxGetMainWnd();
	CVisionTab *pVTDlg = (CVisionTab*)GetParent();
	if(!pVTDlg->clip) {
		CClientDC dc(this);
		dc.SetROP2(R2_BLACK);
		dc.MoveTo(m_clipStart); dc.LineTo(m_clipStart.x, m_clipEnd.y);
		dc.MoveTo(m_clipStart); dc.LineTo(m_clipEnd.x, m_clipStart.y);
		dc.MoveTo(m_clipStart.x, m_clipEnd.y); dc.LineTo(m_clipEnd);
		dc.MoveTo(m_clipEnd.x, m_clipStart.y); dc.LineTo(m_clipEnd);
	}

	// �ش� ������ Depth value �ǽð� ���
	if(m_PointDepth.x >= 0 && m_PointDepth.y >= 0) {
		// drawing cross at the position
		DrawCross(m_PointDepth);

		// Controlâ�� �ش� ��ǥ�� depth �� ���
		// ���� �̹��� ũ�⿡ �°� ��ǥ ����
		CPoint rpt;
		rpt.x = (int)m_PointDepth.x/m_ZoomX;
		rpt.y = (int)m_PointDepth.y/m_ZoomY;
		
		CString str;
		str.Format("X: %d, Y: %d, Dist: %d ", rpt.x, rpt.y, pVTDlg->kinect.getDepthValue(rpt.x, rpt.y));
		pVTDlg->SetDlgItemTextA(IDC_DISPDEPTHVAL, str);
	}

}

void CImageDisp::DisplayFitToWindow(CDC *pDC)
{
    CRect client; GetClientRect(&client);

    double xz = (double) client.Width() / (double) m_pDib->Width();
    double yz = (double) client.Height() / (double) m_pDib->Height();
	// TODO
    xz = min(xz, yz);
    yz = xz;
    m_ZoomX = xz;
    m_ZoomY = yz;
    int width = (int) (((double) m_pDib->Width()) * xz);
    int height = (int) (((double) m_pDib->Height()) * yz);
    CRect disp;
    if (m_bCentering) {
      disp.left = (client.Width() - width)/2;
      disp.top = (client.Height() - height)/2;
      disp.right = disp.left + width;
      disp.bottom = disp.top + height;
    }
    else {
      disp.left = 0; 
      disp.top = 0; 
      disp.right = width; 
      disp.bottom = height; 
    }
    m_Disp = disp;


    pDC->SetStretchBltMode(COLORONCOLOR);
    m_pDib->LoadPaletteImage(pDC, &disp);

	
	// --------------------------------------------------------
	// ------------------------twkim---------------------------
	// --------------------------------------------------------
	// Clip ���� �ǽð����� �׸���
	//CVisionTab *pVTDlg = (CVisionTab*)::AfxGetMainWnd();
	CVisionTab *pVTDlg = (CVisionTab*)GetParent();
	// clip���� �� �簢�� ������� �ּ� �ݴ��...
	if(true/*!pRTV4Dlg->clip*/) {
		CClientDC dc(this);
		dc.SetROP2(R2_BLACK);
		dc.MoveTo(m_clipStart); dc.LineTo(m_clipStart.x, m_clipEnd.y);
		dc.MoveTo(m_clipStart); dc.LineTo(m_clipEnd.x, m_clipStart.y);
		dc.MoveTo(m_clipStart.x, m_clipEnd.y); dc.LineTo(m_clipEnd);
		dc.MoveTo(m_clipEnd.x, m_clipStart.y); dc.LineTo(m_clipEnd);
	}

	// �ش� ������ Depth value �ǽð� ���
	if(m_PointDepth.x >= 0 && m_PointDepth.y >= 0) {
		
		DrawCross(m_PointDepth);

		// Controlâ�� �ش� ��ǥ�� depth �� ���
		// ���� �̹��� ũ�⿡ �°� ��ǥ ����
		// Image�� clip���� �� ������ ��ǥ ��ȯ ó���� �ʿ���...
		CPoint rpt = GetGlobalCoordinate(m_PointDepth);

		CString str;
		str.Format("X: %d, Y: %d, Dist: %d ", rpt.x, rpt.y, pVTDlg->kinect.getDepthValue(rpt.x, rpt.y));
		pVTDlg->SetDlgItemTextA(IDC_DISPDEPTHVAL, str);
	}
	
	// Calib points �׸���
	if(pVTDlg->m_calibPoint.GetNum()) {
		int num = pVTDlg->m_calibPoint.GetNum();
		CPoint pt;
		for(int j=0; j<num; j++) {
			pt.x = pVTDlg->m_calibPoint.GetVal(j,0);
			pt.y = pVTDlg->m_calibPoint.GetVal(j,1);
			DrawCross(pt);
		}
	}
		
}

void CImageDisp::DisplayFitToWindowNRA(CDC *pDC)
{
    CRect client; GetClientRect(&client);

    double xz = (double) client.Width() / (double) m_pDib->Width();
    double yz = (double) client.Height() / (double) m_pDib->Height();

    // not preserving aspect ratio (different from FitToWindow)
    // xz = min(xz, yz);
    // yz = xz;

    m_ZoomX = xz;
    m_ZoomY = yz;
    int width = (int) (((double) m_pDib->Width()) * xz);
    int height = (int) (((double) m_pDib->Height()) * yz);
    CRect disp;
    if (m_bCentering) {
      disp.left = (client.Width() - width)/2;
      disp.top = (client.Height() - height)/2;
      disp.right = disp.left + width;
      disp.bottom = disp.top + height;
    }
    else {
      disp.left = 0; 
      disp.top = 0; 
      disp.right = width; 
      disp.bottom = height; 
    }
    m_Disp = disp;

    pDC->SetStretchBltMode(COLORONCOLOR);
    m_pDib->LoadPaletteImage(pDC, &disp);	
}

void CImageDisp::ClientToImageCoordinate(CPoint *pPoint)
{
    CPoint res(-1, -1);
    if (!m_Disp.PtInRect(*pPoint)) {
      *pPoint = res;
      return;
    }

    *pPoint -= m_Disp.TopLeft();
    pPoint->x = (int) (pPoint->x/m_ZoomX);
    pPoint->y = (int) (pPoint->y/m_ZoomY);
}

void CImageDisp::ClientToImageCoordinate(CRect *pRect)
{
    CPoint pt1 = pRect->TopLeft();
    CPoint pt2 = pRect->BottomRight();

    ClientToImageCoordinate(&pt1);
    ClientToImageCoordinate(&pt2);

    pRect->left = pt1.x;
    pRect->top = pt1.y;
    pRect->right = pt2.x;
    pRect->bottom = pt2.y;
}

void CImageDisp::ImageToClientCoordinate(CPoint *pPoint)
{
    pPoint->x = (int) (pPoint->x*m_ZoomX);
    pPoint->y = (int) (pPoint->y*m_ZoomY);
    
    *pPoint += m_Disp.TopLeft();
}

void CImageDisp::ImageToClientCoordinate(CRect *pRect)
{
    CPoint pt1 = pRect->TopLeft();
    CPoint pt2 = pRect->BottomRight();

    ImageToClientCoordinate(&pt1);
    ImageToClientCoordinate(&pt2);

    pRect->left = pt1.x;
    pRect->top = pt1.y;
    pRect->right = pt2.x;
    pRect->bottom = pt2.y;
}

// ���� �̹��� ũ�⿡ ���� picture control�� zoom ���� ó�� �� ���� ���
void CImageDisp::calcInitZoom()
{
	//CRTV4Dlg *pRTV4Dlg = (CRTV4Dlg*)::AfxGetMainWnd();
	//if(!pRTV4Dlg->clip && !m_iZoom) {
	if(!m_iZoom) {
		CRect client; GetClientRect(&client);
		double wz = (double)client.Width()/m_pDib->Width();
		double hz = (double)client.Height()/m_pDib->Height();
		m_iZoom = min(wz, hz);
	}
}


void CImageDisp::ImageToinitClientCoordinate(CRect *pRect)
{
	pRect->left = (int)((double)m_clipStart.x/m_iZoom);
	pRect->top = (int)((double)m_clipStart.y/m_iZoom);
	pRect->right = (int)((double)m_clipEnd.x/m_iZoom);
	pRect->bottom = (int)((double)m_clipEnd.y/m_iZoom);
}



CPoint CImageDisp::GetGlobalCoordinate(CPoint pt)
{
	CPoint g_pt;

	CVisionTab *pVTDlg = (CVisionTab*)GetParent();
	if(pVTDlg->clip) {
		int left = pVTDlg->m_ClipRect.left;	// clipped image start point x
		int top = pVTDlg->m_ClipRect.top;	// clipped image start point y
			
		g_pt.x = pt.x/m_ZoomX + left;
		g_pt.y = pt.y/m_ZoomY + top;
	} else {
		g_pt.x = pt.x/m_ZoomX;
		g_pt.y = pt.y/m_ZoomY;
	}

	return g_pt;
}







/////////////////////////////////////////////////////////////////////////////
// CImageDisp message handlers

void CImageDisp::OnPaint() 
{
	CPaintDC dc(this); // device context for painting
	// TODO: Add your message handler code here
	if (!m_pDib) return;
    
    switch (m_ZoomMode) {
      case ZM_NONE:
        DisplayWithoutZoom(&dc);        
        break;
      case ZM_SHRINKONLY:
        DisplayShrinkOnly(&dc);
        break;
      case ZM_FITTOWINDOW:
        DisplayFitToWindow(&dc);
        break;
      case ZM_FITTOWINDOWNRA:
      default:
        DisplayFitToWindowNRA(&dc);
        break;
    }

	// Do not call CStatic::OnPaint() for painting messages
}

BOOL CImageDisp::OnEraseBkgnd(CDC* pDC) 
{
	// TODO: Add your message handler code here and/or call default
	CBrush *pOldBrush = (CBrush *) pDC->SelectStockObject(WHITE_BRUSH);
    CRect rect; GetClientRect(&rect);
    pDC->Rectangle(&rect);
    pDC->SelectObject(pOldBrush);

	return CStatic::OnEraseBkgnd(pDC);
}


void CImageDisp::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	SetCurrWindow();
	CVisionTab *pVTDlg = (CVisionTab*)GetParent();
	
	//printf("m_currWnd: %d \n", m_currWnd);

	if(!m_drawRect && !pVTDlg->clip && m_currWnd == RGB_WND) {
		//printf("RGB \n");
		m_drawRect=TRUE;
		m_clipEnd = m_clipStart = point;
	} else if(m_currWnd == DEPTH_WND)	// ���콺�� depthâ���� �������� depth value ���
	{
		//printf("Depth \n");
		// ���� depth �̹����� 640x480�ε� â�� ũ��� �̿� ���� �����Ƿ� 
		// ������ ��ǥ ��ȯ�� �ʿ��ϴ�.... 
		m_PointDepth.x = point.x;
		m_PointDepth.y = point.y;
		//printf("m_ZoomX: %lf, Y: %lf, ox: %d, oy: %d \n", m_ZoomX, m_ZoomY, point.x, point.y);
	}else if(m_currWnd == IMGPROC_WND && pVTDlg->clip) {
		
		// vision mode�� calibration�� ���� �۵�
		if(pVTDlg->GetVisionMode() == Calibration) {
			//printf("imgProc \n");
			// ���� setting
			static int y=0; 
			m_PointImgProc.x = (int)((double)(point.x)/m_ZoomX + (double)pVTDlg->m_ClipRect.left);
			m_PointImgProc.y = (int)((double)(point.y)/m_ZoomY + (double)pVTDlg->m_ClipRect.top);
			m_PointImgProc = GetGlobalCoordinate(point);
			
			printf("pointx: %d, pointy: %d trx: %.2f, try: %.2f \n", point.x, point.y, (double)(point.x)/m_ZoomX, (double)(point.y)/m_ZoomY);
			printf("m_ZoomX: %lf, Y: %lf, ox: %d, oy: %d \n", m_ZoomX, m_ZoomY, m_PointImgProc.x, m_PointImgProc.y);
			//ImageToClientCoordinate(&m_PointImgProc);
			
			pVTDlg->m_calibPoint.SetVal(y, 0, m_PointImgProc.x);
			pVTDlg->m_calibPoint.SetVal(y, 1, m_PointImgProc.y);
			pVTDlg->m_calibPoint.SetVal(y, 2, 0.0);
			//printf("m_x: %.2f, m_y: %.2f \n", pVTDlg->m_calibPoint.GetVal(y,0),pVTDlg->m_calibPoint.GetVal(y,1));

			// List control�� ���� ����
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];

			item.iItem = y;
			item.iSubItem = 0;
			item.pszText = itoa(y+1, nullBuffer, 10);
			pVTDlg->m_CalibList.InsertItem(&item);
			for(int i=0; i<3; i++)	// column ���� �߰�
			{
				item.iSubItem = i+1;
				item.pszText = gcvt(pVTDlg->m_calibPoint.GetVal(y,i), 7, nullBuffer);
				pVTDlg->m_CalibList.SetItem(&item);
			}
			

			y = pVTDlg->m_calibPoint.GetNum();
		}
	}
	
	CStatic::OnLButtonDown(nFlags, point);
}


void CImageDisp::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	SetCurrWindow();
	CVisionTab *pVTDlg = (CVisionTab*)GetParent();

	if(m_drawRect && !pVTDlg->clip && m_currWnd == RGB_WND) {
		m_drawRect=FALSE;
		m_clipEnd = point;

		// image clipping process
		CPoint vertex[4];
		vertex[0] = m_clipStart;
		vertex[1] = m_clipEnd;
		vertex[2] = CPoint(m_clipStart.x, m_clipEnd.y);
		vertex[3] = CPoint(m_clipEnd.x, m_clipStart.y);


		double dist = 0;
		double minV = sqrt((double)(vertex[0].x*vertex[0].x + vertex[0].y*vertex[0].y));
		double maxV = sqrt((double)(vertex[1].x*vertex[1].x + vertex[1].y*vertex[1].y));

		int imin = 0;
		int imax = 0;

		for(int i=0; i<sizeof(vertex)/sizeof(CPoint); i++)
		{
			dist = sqrt((double)(vertex[i].x*vertex[i].x + vertex[i].y*vertex[i].y));
			
			if(dist <= minV) {
				minV = dist;
				imin = i;
			}

			if(dist >= maxV) {
				maxV = dist;
				imax = i;
			}
		}
		
		m_clipStart = vertex[imin];
		m_clipEnd = vertex[imax];
		

		// Controlâ�� Clip image ��ǥ ���
		// ���� �� ��ü�� ���� picture control ��ǥ�� ������ �ְ�
		// ����� ���� �̹��� ������(640x480)�� ��ȯ�Ͽ� ����Ѵ�. 
		CString str;
		str.Format("sx: %d, sy: %d, ex: %d, ey: %d", 
			(int)(m_clipStart.x/m_iZoom), (int)(m_clipStart.y/m_iZoom), (int)(m_clipEnd.x/m_iZoom), (int)(m_clipEnd.y/m_iZoom));
		pVTDlg->SetDlgItemTextA(IDC_CLIPCOORDINATE, str);
		
		// Console ���
		/*printf("sx: %d, sy: %d, ex: %d, ey: %d\n", 
		m_clipStart.x, m_clipStart.y,
		m_clipEnd.x, m_clipEnd.y);*/
	} 
	

	CStatic::OnLButtonUp(nFlags, point);
}


void CImageDisp::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	SetCurrWindow();

	//CVisionTab *pVTDlg = (CVisionTab*)::AfxGetMainWnd();
	CVisionTab *pVTDlg = (CVisionTab*)GetParent();
	if(m_drawRect && !pVTDlg->clip && m_currWnd == RGB_WND) {
		printf("x: %d, y: %d\n", point.x, point.y);
		m_clipEnd = point;
	}

	CStatic::OnMouseMove(nFlags, point);
}

//extern CVisionTab m_VisionTab;	// TODO

// ���콺�� ��� �����쿡�� ���ȴ����� �Ǵ�
void CImageDisp::SetCurrWindow()
{
	// �������� �θ� �����츦 ������ ���� AfxGetMainWnd() �Լ��� ����ߴµ�, ���߿� ���α׷��� �ٿ� �Ǵ� �� 
	// �� �۵����� �ʾҴ�. GetParent()�� ȣ���ϴ� �����ϰ� �ذ��
	// �� ���� ������ �̶�� AfxGetMainWnd()�� MFC�� ���� �����忡�� MainFrame ������ ���� ���̾�α��� �����͸� ��ȯ�ϱ⸸
	// �ϴ� �Լ��̴�. �ݸ� GetParent()�� Win32 API�� GetParent()�� ȣ���Ͽ� CWnd ��ü�� ã�� �Ѱ��ִ� ����
	// TabControl�� ������ ���� ���� -> �� ��Ʈ�� -> ���ϵ� ���̾�α� �� ������ ������ �Ǿ� �ִµ�, 
	// ���ݱ��� ����ߴ� AfxMainWnd()�� ȣ���ϸ� ���� ���� �����͸� ��� �Ǵ� ���̾���. �׷��� ��� ���� ����� �۵����� �ʰ�
	// �ý����� �ٿ� �Ǵ� ������ �߻��ߴµ�, GetParent()�� ȣ�������ν� �ٷ� �� �θ��� �� ��Ʈ���� �����͸� ȣ���ϴ� ���̴�. 
	CVisionTab *pVTDlg = (CVisionTab*)GetParent();
	
	CPoint mPos;	// mouse position
	CRect rectRGB;		
	CRect rectDepth;		
	CRect rectProc;		// Process Area

	GetCursorPos(&mPos);
	pVTDlg->ScreenToClient(&mPos);
	
	pVTDlg->m_DispRGB.GetWindowRect(&rectRGB);
	pVTDlg->ScreenToClient(&rectRGB);

	pVTDlg->m_DispDepth.GetWindowRect(&rectDepth);
	pVTDlg->ScreenToClient(&rectDepth);

	pVTDlg->m_imgProcess.GetWindowRect(&rectProc);
	pVTDlg->ScreenToClient(&rectProc);

	//printf("x: %d, y: %d \n", mPos.x, mPos.y);
	//printf("top: %d, left: %d, bottom: %d, right: %d \n", rectRGB.top, rectRGB.left, rectRGB.bottom, rectRGB.right);


	if(mPos.y > rectRGB.top && mPos.x > rectRGB.left && mPos.y < rectRGB.bottom && mPos.x < rectRGB.right)
	{
		m_currWnd = RGB_WND;
		//printf("RGB\n");
	} 
	else if(mPos.y > rectDepth.top && mPos.x > rectDepth.left && mPos.y < rectDepth.bottom && mPos.x < rectDepth.right)
	{
		m_currWnd = DEPTH_WND;
		//printf("Depth\n");
	}
	else if(mPos.y > rectProc.top && mPos.x > rectProc.left && mPos.y < rectProc.bottom && mPos.x < rectProc.right)
	{
		m_currWnd = IMGPROC_WND;
		//printf("imgProc\n");
	}

}


// Cross �׸���
void CImageDisp::DrawCross(CPoint point)
{
	// drawing cross at the position
	CClientDC dc(this);
			
	CPen pen;
	pen.CreatePen(PS_SOLID, 3, RGB(255,0,0));	// ������ �� ����
	CPen* oldPen = dc.SelectObject(&pen);

	CPoint cp;
	dc.SetROP2(R2_COPYPEN);
	cp.x = point.x;
	cp.y = point.y-5;
	dc.MoveTo(cp); dc.LineTo(cp.x, cp.y+10);
	cp.x = point.x-5;
	cp.y = point.y;
	dc.MoveTo(cp); dc.LineTo(cp.x+10, cp.y);
	dc.SelectObject(oldPen);
}