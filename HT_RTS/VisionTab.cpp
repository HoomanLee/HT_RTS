// VisionTab.cpp : ���� �����Դϴ�.
//
#pragma once

#include "stdafx.h"
#include "HT_RTS.h"
#include "VisionTab.h"
#include "Kde.h"
#include "afxdialogex.h"
#include <iostream>



//#include "ImageDisp.h"
#include "KinectConnection.h"

using namespace std;

// CVisionTab ��ȭ �����Դϴ�.


IMPLEMENT_DYNAMIC(CVisionTab, CDialogEx)

CVisionTab::CVisionTab(CWnd* pParent /*=NULL*/)
	: CDialogEx(CVisionTab::IDD, pParent)
	, clip(FALSE)
	, pColor(NULL)
	, pDepth(NULL)
	, pBufColor(NULL)
	, pBufDepth(NULL)
	, pColorClip(NULL)
	, pDepthClip(NULL)
	, m_visionMode(0)
	, m_flag(FALSE)	// TODO
	, pTestImage(NULL)	// TODO
	, m_binThres(100)
{
	
}

CVisionTab::~CVisionTab()
{
}

void CVisionTab::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_IMAGERGB, m_DispRGB);
	DDX_Control(pDX, IDC_IMAGEDEPTH, m_DispDepth);
	DDX_Control(pDX, IDC_CLIPIMAGE, m_ImgClipBtn);
	DDX_Control(pDX, IDC_IMGPROCESS, m_imgProcess);
	DDX_Control(pDX, IDC_VISIONCLASSCOMBO, m_Combo_VisionClass);
	DDX_Control(pDX, IDC_IMGTEST, m_testView);
	DDX_Control(pDX, IDC_CALIB_LIST, m_CalibList);
	DDX_Control(pDX, IDC_SPIN_BINARIZATION, m_SpinBin);
}


BEGIN_MESSAGE_MAP(CVisionTab, CDialogEx)
//	ON_WM_PAINT()
//	ON_STN_CLICKED(IDC_IMAGERGB, &CVisionTab::OnStnClickedImagergb)
	ON_BN_CLICKED(IDC_CLIPIMAGE, &CVisionTab::OnBnClickedClipimage)
	ON_CBN_SELCHANGE(IDC_VISIONCLASSCOMBO, &CVisionTab::OnCbnSelchangeVisionclasscombo)
	ON_BN_CLICKED(IDC_DO_CALIB, &CVisionTab::OnBnClickedDoCalib)
	ON_BN_CLICKED(IDC_CLEAR_CALIB, &CVisionTab::OnBnClickedClearCalib)
//	ON_EN_UPDATE(IDC_EDIT_BINARIZATION, &CVisionTab::OnEnUpdateEditBinarization)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_BINARIZATION, &CVisionTab::OnDeltaposSpinBinarization)
END_MESSAGE_MAP()


// CVisionTab �޽��� ó�����Դϴ�.


// ���� ��� ����
HANDLE CVisionTab::m_updateEvent = FALSE;
HANDLE CVisionTab::m_GetCimgEvent = FALSE;
HANDLE CVisionTab::m_GetDimgEvent = FALSE;
HANDLE CVisionTab::m_Mutex = FALSE;



//----------------------------
//	�����ӷ���Ʈ ���
//----------------------------
void CVisionTab::CalcTime()
{
	static UINT FrameCount = 0;
	static UINT FPS = 0;
	static UINT LastFrameCount = 0;
	static DWORD LastTick = 0;


	FrameCount++;

    DWORD tickCount = GetTickCount();
    DWORD span      = tickCount - LastTick;
    if (span >= 1000)
    {
        FPS            = (UINT)((double)(FrameCount - LastFrameCount) * 1000.0 / (double)span + 0.5);
        LastTick       = tickCount;
        LastFrameCount = FrameCount;

		CString fps;
		fps.Format("%d", FPS > 30 ? 30 : FPS);	// 30 limit
		m_FrameRate.SetCaption(fps);
    }
}


//---------------------------------------
//	Image Processing �����ӷ���Ʈ ���
//---------------------------------------

void CVisionTab::CalcTimeIP()
{
	static UINT FrameCount = 0;
	static UINT FPS = 0;
	static UINT LastFrameCount = 0;
	static DWORD LastTick = 0;

	FrameCount++;

    DWORD tickCount = GetTickCount();
    DWORD span      = tickCount - LastTick;
    if (span >= 1000)
    {
        FPS            = (UINT)((double)(FrameCount - LastFrameCount) * 1000.0 / (double)span + 0.5);
        LastTick       = tickCount;
        LastFrameCount = FrameCount;

		CString fps;
		fps.Format("%d", FPS > 30 ? 30 : FPS);	// 30 limit
		m_FrameRateIP.SetCaption(fps);
    }

}




BOOL CVisionTab::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.

	/////// Kinect �ʱ�ȭ
	if(kinect.initKinect())
	{
		// TODO !!!
		m_updateEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		InitializeCriticalSection(&cs);

		m_Mutex = CreateMutex(NULL, FALSE, NULL);

		m_GetCimgEvent = CreateEventW(NULL, TRUE, FALSE, NULL);	// �ʱ� �̺�Ʈ ����, ���� ����, �ʱ� ����, �̺�Ʈ �̸�,..., ó���� �ʱ�ȭ ������ �� ��!
		m_GetDimgEvent = CreateEventW(NULL, TRUE, FALSE, NULL);	
		
		SetEvent(m_updateEvent);
		//SetEvent(m_GetCimgEvent);
		//SetEvent(m_GetDimgEvent);

		pThread = AfxBeginThread((AFX_THREADPROC)CVisionTab::ThreadUpdateKinectImage, (LPVOID)this);
		pThreadIP = AfxBeginThread((AFX_THREADPROC)CVisionTab::ThreadImageProcessing, (LPVOID)this);	// image processing thread
		
		
		if(pThread == NULL) {
			AfxMessageBox("Failed to begin thread!");
		} else if(pThreadIP == NULL) {
			AfxMessageBox("Failed to begin thread I.P!");
		}
		else
			theApp.T_SW->bool_vision = true;

	}else {
		CString str;
		str.Format("Failed to connect to Kinect!");
		AfxMessageBox(str);
	}



	/////// FPS text �ʱ�ȭ
	m_FrameRate.SubclassDlgItem(IDC_FPS, this);
	m_FrameRate.SetFontSize(300);

	/////// FPS Image Process text �ʱ�ȭ
	m_FrameRateIP.SubclassDlgItem(IDC_FPS_IP, this);
	m_FrameRateIP.SetFontSize(300);


	/////// �޺� �ڽ� �ʱ�ȭ
	// Vision Class
	for(int i=0; i<sizeof(VisionClass)/sizeof(CString); i++)
	{
		m_Combo_VisionClass.AddString(VisionClass[i]);
	}
	m_Combo_VisionClass.SetCurSel(0);

	//////// Binarization�� ���� Spin ��Ʈ�� �ʱ�ȭ
	m_SpinBin.SetRange(0,255);
	m_SpinBin.SetPos(100);


	// Calib-point List initialization
	char *szText[7] = {"Num", "VX", "VY", "VZ", "RX", "RY", "RZ"};
	int nWid[7] = {50, 65, 65, 65, 65, 65, 65};
	LV_COLUMN lCol;
	lCol.mask = LVCF_FMT|LVCF_SUBITEM|LVCF_TEXT|LVCF_WIDTH;	// ����ü�� ����� Ȯ���� �÷��� ����
	lCol.fmt = LVCFMT_LEFT;

	for(int i=0; i<7; i++)
	{
		lCol.pszText = szText[i];			// �÷��� ������ ����
		lCol.iSubItem = i;					// ���� �������� �ε����� ����
		lCol.cx = nWid[i];					// �÷��� ���̸� ����
		m_CalibList.InsertColumn(i, &lCol);	// LVCOLUMN����ü�� ������� ���� ���� ����Ʈ ��Ʈ�ѿ� ����
	}
	m_CalibList.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);	// List control style ����
	
	



	K = MatrixXd::Zero(3,3);
	K(0,0) = 532.45;	// fx
	K(1,1) = 531.82;	// fy
	K(0,2) = 335.82;	// cx
	K(1,2) = 263.95;	// cy
	K(2,2) = 1;			// dummy



	// Transformation Matrix initialization
	// ���� �Ķ���� ���
	//double rad = -PI/2;
	//M_st(0,0) = cos(rad);		M_st(0,1) = -sin(rad);		M_st(0,2) = 0.0;	M_st(0,3) = -862.0;
	//M_st(1,0) = sin(rad);		M_st(1,1) =  cos(rad);		M_st(1,2) = 0.0;	M_st(1,3) = 155.0;
	//M_st(2,0) = 0.0;			M_st(2,1) =  0.0;			M_st(2,2) = 1.0;	M_st(2,3) = -801.0 + 391;	// ������ 391 // distance between camera and floor
	//M_st(3,0) = 0.0;			M_st(3,1) =  0.0;			M_st(3,2) = 0.0;	M_st(3,3) = 1.0;

	M_st = MatrixXd::Zero(4,4);
	double rad = PI;
	M_st(0,0) = cos(rad);		M_st(0,1) =  sin(rad);		M_st(0,2) = 0.0;	M_st(0,3) = 838;
	M_st(1,0) = -sin(rad);		M_st(1,1) =  cos(rad);		M_st(1,2) = 0.0;	M_st(1,3) = -52.5;
	M_st(2,0) = 0.0;			M_st(2,1) =  0.0;			M_st(2,2) = 1.0;	M_st(2,3) = 379;	//// TODO 
	M_st(3,0) = 0.0;			M_st(3,1) =  0.0;			M_st(3,2) = 0.0;	M_st(3,3) = 1.0;

	return TRUE;  // return TRUE unless you set the focus to a control
	// ����: OCX �Ӽ� �������� FALSE�� ��ȯ�ؾ� �մϴ�.
}








// -----------------------------------------------------------------
// -------------------- Thread 1 -----------------------------------
// -----------------------------------------------------------------
UINT CVisionTab::ThreadUpdateKinectImage(LPVOID pParam)
{
	CVisionTab *dlg = (CVisionTab *)pParam;

	HANDLE events[] = {dlg->kinect.m_cFrameReadyEvent,
					   dlg->kinect.m_dFrameReadyEvent};
	
	while(TRUE)
	{

		// �ش� �̺�Ʈ�� �߻��Ҷ����� thread�� ����ٰ� �̺�Ʈ�� �߻��ϸ� 
		// thread�� �۵���Ű�� �ڵ�
		if(dlg->kinect.getSpeckleTrigger()) {
			dlg->kinect.reOpenColorMode();
			Sleep(500);
		}
		DWORD ret = WaitForMultipleObjects(ARRAYSIZE(events), events, FALSE, INFINITE);
		
        if(WAIT_OBJECT_0 + 1 >= ret)	// 0�� or 1�� �̺�Ʈ �϶�!
        {
			//EnterCriticalSection(&dlg->cs);
			if(WAIT_OBJECT_0 == WaitForSingleObject(dlg->m_updateEvent, INFINITE)) 
			{
				//ResetEvent(m_updateEvent);
				dlg->updateColorStream();
				dlg->updateDepthStream();
			}
			SetEvent(dlg->m_updateEvent);
			//LeaveCriticalSection(&dlg->cs);
        }
	}
	return 0;
}


// -----------------------------------------------------------------
// -------------------- Thread 2 -----------------------------------
// -----------------------------------------------------------------
UINT CVisionTab::ThreadImageProcessing(LPVOID pParam)
{
	CVisionTab *dlg = (CVisionTab *)pParam;

	while(1)
	{
		//EnterCriticalSection(&dlg->cs);
		if(WAIT_OBJECT_0 == WaitForSingleObject(dlg->m_updateEvent, INFINITE)) 
		{
			//ResetEvent(m_updateEvent);
			dlg->doingImageProcess();
		}
		SetEvent(dlg->m_updateEvent);
		//LeaveCriticalSection(&dlg->cs);
	}

	return 0;
}



// ------------------------------------------------------------------
// ------------------ Image processing ------------------------------
// ------------------------------------------------------------------
void CVisionTab::doingImageProcess()
{
	//cout << "image processing..." << endl;
	// Calculate the Frame Rate

	CalcTimeIP();

	if(clip) {
		//if(pGColorClip && pGDepthClip)	// Clipped image�� ���ؼ��� ���� ó�� ����
		{
			
			switch(m_visionMode) {
				case None:
					{
					CDib* pDib = GetColorClipImage(m_ClipRect);
					if(pDib) m_imgProcess.SetDib(pDib);
					break;
					}
				case Color_box:
					{
					CDib* pRes = NULL;
					CDib* pDib = NULL;

					pDib = GetColorClipImage(m_ClipRect);
					if(pDib) {
						pRes = ColorBoxRecog(pDib);	// DLL�� ���� �Լ�..
						m_imgProcess.SetDib(pRes);
					}
					delete pDib;
					

					// Display the coordinate of each color box
					CString str;	// RGBY
					str.Format("Red      x: %.1lf y: %.1lf z: %.1lf rz: %.1lf \nGreen   x: %.1lf y: %.1lf z: %.1lf rz: %.1lf \nBlue     x: %.1lf y: %.1lf z: %.1lf rz: %.1lf \nYellow  x: %.1lf y: %.1lf z: %.1lf rz: %.1lf", 
					m_cbWorldPoint[RED].x, m_cbWorldPoint[RED].y, m_cbWorldPoint[RED].z, m_cbWorldPoint[RED].rz*(180/PI),
					m_cbWorldPoint[GREEN].x, m_cbWorldPoint[GREEN].y, m_cbWorldPoint[GREEN].z, m_cbWorldPoint[GREEN].rz*(180/PI),
					m_cbWorldPoint[BLUE].x, m_cbWorldPoint[BLUE].y, m_cbWorldPoint[BLUE].z, m_cbWorldPoint[BLUE].rz*(180/PI),
					m_cbWorldPoint[YELLOW].x, m_cbWorldPoint[YELLOW].y, m_cbWorldPoint[YELLOW].z, m_cbWorldPoint[YELLOW].rz*(180/PI));

					/*********************************************************/
					/*T_SW�� Vision data ���� */
					for(int i=0; i<NUMBOX; i++){
						theApp.T_SW->T_ND.recv_vision[4*i + 0] = m_cbWorldPoint[i].x;
						theApp.T_SW->T_ND.recv_vision[4*i + 1] = m_cbWorldPoint[i].y;
						theApp.T_SW->T_ND.recv_vision[4*i + 2] = m_cbWorldPoint[i].z;
						theApp.T_SW->T_ND.recv_vision[4*i + 3] = m_cbWorldPoint[i].rz;
					}
					/*********************************************************/

					SetDlgItemTextA(IDC_COORDTARGETOBJ, str);


					// TODO, Monitor
					if(pTestImage) {
						m_testView.SetDib(pTestImage->CopyCDib());
						delete pTestImage; pTestImage=NULL;
					}
					

					break;
					}
				case Calibration:
					{
						CDib* pDib = NULL;
						pDib = GetColorClipImage(m_ClipRect);
						if(pDib) { 
							m_imgProcess.SetDib(pDib->CopyCDib());
						}
						//m_testView.SetDib(pDib->GaussianSmoothing(2));
						delete pDib;
					}
					break;
				case Test:
					{
						CDib* pDib = NULL;
						CDib* pTmp = NULL;
						pDib = GetColorClipImage(m_ClipRect);
						if(pDib) { 
							pTmp = pDib->GlobalBinFixed(m_binThres)->ReverseG()->Opening(3); 	// clip image�� ���ؼ�..

							// KDE, image point
							TargetObject obj = CellPhonePartRecog(pTmp);
							
							if(obj.density > 0.6) {

								// Orientation
								double rad = pTmp->GetSobelOrientation();
							
								//center = pTmp->GetMeanCoordinateB(0.0);

								MatrixXd StaubliPoint = MatrixXd::Zero(4,1);
								MatrixXd CamPoint	  = MatrixXd::Zero(4,1);

								// Image -> Camera point ��ȯ
								obj.wz = kinect.getDepthValue(obj.ix, obj.iy);
								CamPoint(0,0) = ((obj.ix - (double)m_GlobalCenter.x)*obj.wz)/K(0,0) + OFFSETX;
								CamPoint(1,0) = -((obj.iy - (double)m_GlobalCenter.y)*obj.wz)/K(1,1) + OFFSETY;
								//CamPoint(2,0) = 940 - obj.wz;	// 940 mm: Kinect�� World Frame ������ �Ÿ�
								CamPoint(3,0) = 1.0;

								// Camera -> Staubli point ��ȯ
								StaubliPoint = M_st * CamPoint;	// Transformation Matrix ������

			
								obj.rox = StaubliPoint(0,0);
								obj.roy = StaubliPoint(1,0);
								obj.roz = -305;	// ���� �ε����� �ʵ��� limit �ɾ���
								obj.Rz = rad;

								
								// Print & Visualization
								CString str;
								int len = 30;
								//char* CellPhoneParts[] = {"ũ����", "�÷���", "USBĿ����", "�̾���"};
								const CString CellPhoneParts[] = {"ũ����", "�÷���", "USBĿ����", "�̾���"};
								str.Format("rox: %.2f, roy: %.2f, roz: %.2f, ori: %.2lf, cnt: %d, Obj Class: %s \n", obj.rox, obj.roy, obj.roz, obj.Rz*RtoD, obj.cnt, CellPhoneParts[obj.ID]);
								SetDlgItemTextA(IDC_COORDTARGETOBJ, str);
								unsigned char val=255;
							
								pDib->DrawCross(obj.cix, obj.ciy, val);
								pDib->DrawLine(obj.cix-len*cos(obj.Rz), obj.ciy-len*sin(obj.Rz), obj.cix+len*cos(obj.Rz), obj.ciy+len*sin(obj.Rz), RGB(255,100,128));
							}
							

							m_imgProcess.SetDib(pDib->CopyCDib());
							m_testView.SetDib(pTmp->CopyCDib());
						}
						delete pDib;
						delete pTmp;
					}
					break;
			}
			
		}
	} else {	// ��ü �̹��� ����϶� �׳� ��¸�...
		if(pColor)
		{
			CDib* pDib = GetColorImage();
			if(pDib != NULL) m_imgProcess.SetDib(pDib);


			switch(m_visionMode) 
			{
				case Test:{
					//CDib* pDib = GetColorImage();
					//m_testView.SetDib(pDib->GaussianSmoothing(3));
					//delete pDib;
					break;
						  }
				default: {
					
						 }

			}
		}
	}
}



void CVisionTab::updateColorStream()
{
	if(WAIT_OBJECT_0 == WaitForSingleObject(kinect.m_cFrameReadyEvent, 0))
	{
	//ResetEvent(m_GetCimgEvent);
	if(pBufColor == NULL && pColor != NULL ) {
		pBufColor = pColor->CopyCDib();
		SetEvent(m_GetCimgEvent);
	}
	
	if(pColor != NULL) {delete pColor; pColor=NULL;}
	

	CDib* temp = kinect.getCDibColorImage();
	if(!temp) return;

	CDib *pDibRGB = temp->CopyCDib();
	//pColor = pDibRGB->CopyCDib();
	pColor = pDibRGB->GetDistortionCorrectedImage(532.45, 531.82, 0, 331.7, 264.18, 0.16657, -0.29178);	// ī�޶� ���� �ְ� ���� �̹���
	
	

	static bool flag = FALSE;
	
	// ó�� �� ���� Global image�� �߽� ��ǥ ���
	if(!flag && pDibRGB) {
		// Image�� �߽� ��ǥ
		m_GlobalCenter.x = pDibRGB->Width()/2;
		m_GlobalCenter.y = pDibRGB->Height()/2;

		// Calibration�� ���� ���� Principal point ����
		//m_GlobalCenter.x = 331;
		//m_GlobalCenter.y = 264;
		flag = TRUE;
	}

	if(pDibRGB) {
		// Display update
		if(clip) {
			// ���� �̹����� ���� clip �Ϸ��� �ּ� �ݴ��..
			/*CDib *pRGB = pDibRGB->ClipCDib(m_ClipRect);
			m_DispRGB.SetDib(pRGB);*/

			m_DispRGB.SetDib(pDibRGB->CopyCDib());
			CalcTime();	// Calc FPS
			
		} else {
			m_DispRGB.SetDib(pDibRGB->CopyCDib());
			CalcTime();	// Calc FPS	
		}


		delete pDibRGB;
	}
	
	//SetEvent(m_GetCimgEvent);

	}
}


void CVisionTab::updateDepthStream()
{
	if(WAIT_OBJECT_0 == WaitForSingleObject(kinect.m_dFrameReadyEvent, 0))
	{

	if(pBufDepth == NULL && pDepth != NULL) {
		pBufDepth = pDepth->CopyCDib();
		SetEvent(m_GetDimgEvent);
	}

	if(pDepth != NULL) {delete pDepth; pDepth=NULL;}

	CDib *pDibDepth = kinect.getCDibDepthImage()->CopyCDib();
	pDepth = pDibDepth->CopyCDib();
	
	
	if(pDibDepth) {
		// Display update
		if(clip) {
			CDib *pDepth = pDibDepth->ClipCDib(m_ClipRect);	// memory leak when using median filter... 
			m_DispDepth.SetDib(pDepth);

		} else {
			m_DispDepth.SetDib(pDibDepth->CopyCDib());
		}

		delete pDibDepth;
	}

	}
}



CDib* CVisionTab::GetColorImage()
{
	//if(WAIT_OBJECT_0 == WaitForSingleObject(m_GetCimgEvent, 0)) {
		//ResetEvent(m_GetCimgEvent);
		
		CDib* pDib = NULL;
		//EnterCriticalSection(&cs);
		if(pBufColor != NULL) 
		{
			pDib = pBufColor->CopyCDib();
			delete pBufColor; pBufColor=NULL;
			//SetEvent(m_GetCimgEvent);
		} 
		//LeaveCriticalSection(&cs);
		
		return pDib;
	//}
}

CDib* CVisionTab::GetDepthImage()
{
	//DWORD ret = WaitForSingleObject(m_GetDimgEvent, INFINITE);

	//if(ret == WAIT_OBJECT_0 && pDepth) {
		//ResetEvent(m_GetDimgEvent);
		
		//g_CS.Lock();
		CDib* pDib = NULL;
		if(pBufDepth != NULL)
		{
			pDib = pBufDepth->CopyCDib();
			delete pBufDepth; pBufDepth=NULL;
			//SetEvent(m_GetDimgEvent);
		}
		//g_CS.Unlock();
		
		
		//SetEvent(m_GetDimgEvent);
		return pDib;		
	//}
}


CDib* CVisionTab::GetColorClipImage(RECT *pRect)
{
	CDib* pTmp = NULL;
	CDib* pDib = NULL;

	if(WAIT_OBJECT_0 == WaitForSingleObject(m_GetCimgEvent, INFINITE))
		pTmp = GetColorImage();
	if(pTmp) pDib = pTmp->ClipCDib(pRect);
	delete pTmp; pTmp=NULL;

	return pDib;
}


CDib* CVisionTab::GetDepthClipImage(RECT *pRect)
{
	CDib* pTmp;
	CDib* pDib;
	
	if(WAIT_OBJECT_0 == WaitForSingleObject(m_GetDimgEvent, INFINITE))
		pTmp = GetDepthImage();
	if(pTmp) pDib = pTmp->ClipCDib(pRect);
	delete pTmp; pTmp=NULL;

	return pDib;
}




// Image Clip button�� ������ ��..
void CVisionTab::OnBnClickedClipimage()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(clip == FALSE) {
		m_ImgClipBtn.SetWindowTextA("Use whole img");
		clip = TRUE;
		
		// ���� �̹��� ��ǥ �������� ������Ʈ
		m_DispRGB.ImageToinitClientCoordinate(&m_ClipRect);
		
		// Center Point ����
		m_ClipCenter.x = (int)m_ClipRect.Width()/2 + m_ClipRect.left;
		m_ClipCenter.y = (int)m_ClipRect.Height()/2 + m_ClipRect.top;
		
		printf("cx: %d, cy: %d \n", m_ClipCenter.x, m_ClipCenter.y);
	} else {
		m_ImgClipBtn.SetWindowTextA("Image clipping");
		clip = FALSE;
	}
}


void CVisionTab::OnCbnSelchangeVisionclasscombo()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int index = m_Combo_VisionClass.GetCurSel();
	m_visionMode = index;
}







///////////// Vision Class Implementation //////////////
CDib* CVisionTab::ColorBoxRecog(CDib* pImg)
{
	
	// ------------------------------------------------------
	// ------------------- Color Slicing &-------------------
	// ------------- Kernel density estimation --------------
	// ------------------------------------------------------

	//CalcTime();
	
	// input image ���� ó�� �� ����
	CDib* src = pImg->GetSingleScaleRetinexImage(1.5);
	//CDib* src = pImg->CopyCDib();

	int width = src->Width();
	int height = src->Height();
	//int bitcount = src->BitCount();
	int bitcount = 8;
	
	
	
	CDib* pRed = new CDib;
	pRed->Allocate(width, height, bitcount);
	pRed->SetGrayPalette();

	CDib* pGreen = new CDib;
	pGreen->Allocate(width, height, bitcount);
	pGreen->SetGrayPalette();

	CDib* pBlue = new CDib;
	pBlue->Allocate(width, height, bitcount);
	pBlue->SetGrayPalette();

	CDib* pYellow = new CDib;
	pYellow->Allocate(width, height, bitcount);
	pYellow->SetGrayPalette();

	
	register int i, j;
	double h, s, v;
	int step = (int)src->BitCount()/8;
	RGBQUAD quad;


	CKde* kdemat[NUMBOX];
	for(i=0; i<NUMBOX; i++) {
		kdemat[i] = new CKde;
		kdemat[i]->initKDEmask(width, height, 4);	// initialize KDE function
	}

	

	for(j=0; j<src->Height(); j++)
	{
		unsigned char *ptr = src->GetPointer(0,j);
		for(i=0; i<src->Width(); i++, ptr+=step)
		{
			quad.rgbBlue = *(ptr+0);	quad.rgbGreen = *(ptr+1);	quad.rgbRed = *(ptr+2);
			src->RGBtoHSV(quad, &h, &s, &v);

			// normalize Hue
			h /= 360;
			
			
			// Red Box: 0.93, 0.97
			if(h > 0.91 && h < 0.99 && s > TS && v > Ti) {
				unsigned char *pr = pRed->GetPointer(i,j);
				*(pr+0) = 255;
				kdemat[RED]->setSumKDEmask(i,j);
			}

			// Green Box: 0.25, 0.37
			if(h > 0.24 && h < 0.38 && s > TS && v > Ti) {	// default: 0.27, 0.36
				unsigned char *pg = pGreen->GetPointer(i,j);
				*(pg+0) = 255;
				kdemat[GREEN]->setSumKDEmask(i,j);
			}

			// Blue: 0.59, 0.65
			if(h > 0.59 && h < 0.65 && s > TS  && v > Ti) {
				unsigned char *pb = pBlue->GetPointer(i,j);
				*(pb+0) = 255;
				kdemat[BLUE]->setSumKDEmask(i,j);
			}

			// Yellow: 0.12, 0.14
			if(h > 0.11 && h < 0.15 && s > TS  && v > Ti) {
				unsigned char *py = pYellow->GetPointer(i,j);
				*(py+0) = 255;
				kdemat[YELLOW]->setSumKDEmask(i,j);
			}
			
		}
	}
	

	// *****************************************************************
	//
	// �� ���α׷��� ��ǥ�� ũ�� �Ʒ� 3������ ������. 
	// Clip ��ǥ(ci_pt):			clip image�� �������� �� ��ǥ
	// Global image ��ǥ(mpt):	640x480�� ��ü �̹����� �������� �� ��ǥ
	// World ��ǥ:			Camera calibration�� ���� ���� ��ǥ��
	//
	// *****************************************************************
	
	static Coordi prePoint[NUMBOX];	// Low pass filter������ ���� ���� ��ǥ ���� ����
	static Coordi preGiPt[NUMBOX];	
	double alpha = 0.3;

	Coordi gi_pt[NUMBOX];		// global image point
	unsigned char val = 255;

	CDib* bDib[NUMBOX];

	// RGBY in order
	for(i=0; i<NUMBOX; i++) {
		// Color Box Position ------------------
		CPoint cp = kdemat[i]->GetMaxCoordi();	// Clip image������ KDE max ��ǥ
		CPoint ci_pt = kdemat[i]->GetThresMeanCoordiClip(0.2, cp, 60);				// depth ������ ���� Area ���� ������ ��!
		//bDib[i] = kdemat[i]->GetThresBinaryClip(0.1, cp, 60);						// Binary image ���


		// Color Box Orientation ----------------
		CDib* pTmp[10] = {NULL,};	// Image processing �� ����� ���� �迭
		int sub=20;
		pTmp[0] = src->ClipCDib(ci_pt.x-sub, ci_pt.y-sub, ci_pt.x+sub, ci_pt.y+sub);		// Clipped Grayscale �̹��� ���
		pTmp[1] = pTmp[0]->GetGrayCDib();
		//pTmp[2] = pTmp[1]->BrightnessG(25);
		pTmp[3] = pTmp[1]->ContrastG(3);
		pTmp[10] = pTmp[3]->GaussianSmoothingG(2);

		bDib[i] = pTmp[10];
		
		for(int k=0; k<10; k++){
			if(pTmp[k]) { delete pTmp[k]; pTmp[k]=NULL;}
		}
		
		double maxkde = kdemat[i]->GetMaxValue();
		
		double currx = (ci_pt.x + m_ClipRect.left);
		double curry = (ci_pt.y + m_ClipRect.top);
		double currz = (double)kinect.getDepthValue(currx, curry);
		double currRz = bDib[i]->GetSobelOrientation();	// ���⼭ ���� orientation ���
		

		// Get global x, y, z-axis coordinate
		// Global image ��ǥ, Low pass filter
		gi_pt[i].x = currx*alpha + (1-alpha)*preGiPt[i].x;
		gi_pt[i].y = curry*alpha + (1-alpha)*preGiPt[i].y;
		gi_pt[i].z = currz*alpha + (1-alpha)*preGiPt[i].z;
		gi_pt[i].rz = currRz*alpha + (1-alpha)*preGiPt[i].rz;

		// depth noise ó��
		if(currz == 0.0) {
			gi_pt[i].z = preGiPt[i].z;	// ���� depth ���� 0�� ��� ���� ���� ���
		}
		
		
		// �� Color Box�� pixel ������ ���� �̻��̾�� World ��ǥ ������Ʈ
		// Insert Kalman or any other filter code
		if(maxkde > 0.6) {

			
			MatrixXd StaubliPoint = MatrixXd::Zero(4,1);
			MatrixXd CamPoint = MatrixXd::Zero(4,1);

			// Camera World��ǥ��� ��ȯ
			CamPoint(0,0) = ((gi_pt[i].x - (double)m_GlobalCenter.x)*gi_pt[i].z)/K(0,0) + OFFSETX;
			CamPoint(1,0) = -((gi_pt[i].y - (double)m_GlobalCenter.y)*gi_pt[i].z)/K(1,1) + OFFSETY;
			CamPoint(2,0) = 940 - gi_pt[i].z;	// 940 mm: Kinect�� World Frame ������ �Ÿ�
			CamPoint(3,0) = 1.0;

			//Staubli ���� ��ǥ�� ��ȯ
			StaubliPoint = M_st * CamPoint;	// Transformation Matrix ������


			//m_cbWorldPoint[i].x = CamPoint(0,0);
			//m_cbWorldPoint[i].y = CamPoint(1,0);
			//m_cbWorldPoint[i].z = (CamPoint(2,0) < -370 ? -370 : CamPoint(2,0));	// ���� �ε����� �ʵ��� limit �ɾ���
			//m_cbWorldPoint[i].rx = 0.0;
			//m_cbWorldPoint[i].ry = 0.0;
			//m_cbWorldPoint[i].rz = gi_pt[i].rz;

			
			m_cbWorldPoint[i].x = StaubliPoint(0,0);
			m_cbWorldPoint[i].y = StaubliPoint(1,0);
			//m_cbWorldPoint[i].z = (StaubliPoint(2,0) < -305 ? -305 : StaubliPoint(2,0));	// ���� �ε����� �ʵ��� limit �ɾ���
			m_cbWorldPoint[i].z = -305;	// ���� �ε����� �ʵ��� limit �ɾ���
			m_cbWorldPoint[i].rx = 0.0;
			m_cbWorldPoint[i].ry = 0.0;
			m_cbWorldPoint[i].rz = gi_pt[i].rz;
			
			
			// ���� �����͸� ���� �����ͷ� ����
			preGiPt[i].x = gi_pt[i].x;
			preGiPt[i].y = gi_pt[i].y;
			preGiPt[i].z = gi_pt[i].z;
			preGiPt[i].rx = gi_pt[i].rx;
			preGiPt[i].ry = gi_pt[i].ry;
			preGiPt[i].rz = gi_pt[i].rz;

			double px = gi_pt[i].x-m_ClipRect.left;
			double py = gi_pt[i].y-m_ClipRect.top;

			//pRed->DrawCross(mpt[i].x, mpt[i].y, val);
			src->DrawCross(px, py, val);	// using local(clip) coordinate


			double rad = gi_pt[i].rz;
			double len = 15;
			src->DrawLine(px-len*cos(rad), py-len*sin(rad), px+len*cos(rad), py+len*sin(rad), RGB(255,100,128));
		}
	}

	


	// Region growing�� ������...
	//CDib* pDib = pRed->RegionGrowing(mpt[BLUE].x, mpt[BLUE].y);
	//CDib* pDib = src->RegionGrowing(mpt[RED].x, mpt[RED].y, 80);
	
	if(m_flag) {
		int width = kdemat[RED]->Width();
		int height = kdemat[RED]->Height();

		// ���Ϸ� ����
		FILE *f;
		f = fopen("c:\\kde.dat","wb");
		if(f!=NULL) {
					
			for(int j=0; j<height; j++)
			{
				for(int i=0; i<width; i++)
				{
					fprintf(f, "%lf ", kdemat[RED]->GetValue(i,j));
				}
				fprintf(f, "\n");
			}
	
			fclose(f);
		}
	
		m_flag=0;
		AfxMessageBox("kde map is saved");
	}



	////////////////////////////////////////////
	// Copy an image for monitoring
	pTestImage = bDib[GREEN]->CopyCDib();
	//pTestImage = src->GetGrayCDib();



	////////////////////////////////////////////
	//// delete images.... 
	delete pRed;
	delete pGreen;
	delete pBlue;
	delete pYellow;

	for(i=0; i<NUMBOX; i++) {
		delete bDib[i];
		kdemat[i]->releaseKDEMask();
	}

	return src;
}




/////// -----------------------------------------------------------------------
/////// �޴��� ��ǰ �ν� ���
TargetObject CVisionTab::CellPhonePartRecog(CDib* pImg)
{
	TargetObject obj;

	if(pImg->BitCount() > 8) {
		cout << "Input image should be gray scale! " << endl;
		return obj;
	}
	

	CDib* src = pImg->CopyCDib();
	int width = src->Width();
	int height = src->Height();
	int bitcount = src->BitCount();

	register int i, j;
	
	CKde* kdemat;
	kdemat = new CKde;
	kdemat->initKDEmask(width, height, 4);	// initialize KDE function

	

	for(j=0; j<src->Height(); j++)
	{
		unsigned char *ptr = src->GetPointer(0,j);
		for(i=0; i<src->Width(); i++, ptr++)
		{
			if(*ptr == 255) {
				kdemat->setSumKDEmask(i,j);
			}
		}
	}


	CPoint cp	 = kdemat->GetMaxCoordi();							// Clip image������ KDE max ��ǥ
	CPoint center = kdemat->GetThresMeanCoordiClip(0.2, cp, 60);		// depth ������ ���� Area ���� ������ ��!

	obj.density = kdemat->GetMaxValue();
	if(obj.density > 0.6) {
		//cout << "den" << kdemat->GetMaxValue() << endl;
		// Clip image ��ǥ
		obj.cix = center.x;
		obj.ciy = center.y;
	}

	//// ��ü image ��ǥ
	obj.ix = m_ClipRect.left + obj.cix;
	obj.iy = m_ClipRect.top + obj.ciy;
	
	//// Object ID �з�
	obj.cnt = src->GetTruePixelNumB();
	int range = 100;
	if(obj.cnt > 2050-range && obj.cnt < 2050+range) // ũ����
	{
		obj.ID = CRADLE;
	}
	else if(obj.cnt > 760-range && obj.cnt < 760+range) // �÷���
	{
		obj.ID = PLUG;
	}
	else if(obj.cnt > 920-range && obj.cnt < 920+range) // USB Ŀ����
	{
		obj.ID = USBCON;
	}
	else if(obj.cnt > 500-range && obj.cnt < 500+range)	// �̾���
	{
		obj.ID = EARPHONE;
	}
	else
	{
		obj.ID = -1;
	}

	
	
	
	// ���⿡ orientation ����? 


	// �޸� ����
	kdemat->releaseKDEMask();
	delete src;

	return obj;
}





UINT CVisionTab::GetVisionMode()
{
	return m_visionMode;
}


void CVisionTab::OnBnClickedDoCalib()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	cout << "Do Calib!!" << endl;
}


void CVisionTab::OnBnClickedClearCalib()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	cout << "Clear Calib points" << endl;
	m_calibPoint.SetClear();
	m_CalibList.DeleteAllItems();
}



void CVisionTab::OnDeltaposSpinBinarization(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_EDIT_BINARIZATION, get_value);
	UpdateData(TRUE);	// control -> variable(��Ʈ�� ���� ������)
	m_binThres = atoi(get_value);	// 10�� �������� double�� �Ǽ���..

	if(pNMUpDown->iDelta>0)	// Click up button
	{
		m_binThres++;
	}
	else	// click down button
	{
		m_binThres--;
	}

	set_value.Format("%d", m_binThres);
	SetDlgItemTextA(IDC_EDIT_BINARIZATION, set_value);
	UpdateData(FALSE);	// variable -> control


	*pResult = 0;
}
