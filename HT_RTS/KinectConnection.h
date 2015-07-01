#pragma once

#include "CDibN.h"


#include <NuiApi.h>
#include <NuiSensor.h>
#include "NuiImageBuffer.h"


#define MAX_PLAYER_INDEX    0

class CKinectConnection
{
public:
	CKinectConnection(void);
	~CKinectConnection(void);

public:
	int		initKinect();
	void	openColorStream();
	void	openInfraredStream();
	void	openDepthStream();
	void	closeKinect();
	CDib*	getCDibColorImage();
	CDib*	getCDibDepthImage();
	void	setDepthCalib(BOOL bDepthCalib);
	USHORT	getDepthValue(int x, int y);

	void	setSpeckleImage(BOOL bSetSpeckle);
	BOOL	getSpeckleTrigger();
	void	reOpenColorMode();


	void	ColorMemAlloc();
	void	DepthMemAlloc();

	// for initialization of depth color table
	BYTE*	ResetBuffer(UINT size);
	void	InitDepthColorTable();
	BYTE	GetIntensity(int depth);
	inline void SetColor(UINT* pColor, BYTE red, BYTE green, BYTE blue, BYTE alpha = 255);
	
public:
	HANDLE	videoStreamHandle;
	HANDLE	depthStreamHandle;

	HANDLE	m_cFrameReadyEvent;
	HANDLE	m_dFrameReadyEvent;
	HANDLE	m_cOpenFrameReadyEvent;
	HANDLE	m_dOpenFrameReadyEvent;

	bool	m_pause;
private:
	CDib	*m_pColor;
	CDib	*m_pDepth;

	USHORT**			m_depthMap;		// depth map(actual value)
	DWORD               m_nSizeInBytes;
    BYTE*               m_pBuffer;

	BYTE    m_intensityShiftR[MAX_PLAYER_INDEX + 1];
    BYTE    m_intensityShiftG[MAX_PLAYER_INDEX + 1];
    BYTE    m_intensityShiftB[MAX_PLAYER_INDEX + 1];
	UINT    m_depthColorTable[MAX_PLAYER_INDEX + 1][USHRT_MAX + 1];
	

	HANDLE	m_UpdateEvent;



private:
	BOOL	m_bSpeckle;
	BOOL	m_bSpeckleTrigger;
	bool    m_nearMode;
	NuiImageBuffer m_imageBuffer;



protected:
	INuiSensor*		m_pNuiSensor;

};