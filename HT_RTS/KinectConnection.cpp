#include "StdAfx.h"
#include "KinectConnection.h"
#include "Utility.h"

//#include <opencv\cv.h>
//#include <opencv\highgui.h>
//#include <opencv\cxcore.h>

#include <cmath>

#define max(a,b)            (((a) > (b)) ? (a) : (b))
#define min(a,b)            (((a) < (b)) ? (a) : (b))

#define BYTES_PER_PIXEL_RGB         4
#define BYTES_PER_PIXEL_DEPTH       sizeof(NUI_DEPTH_IMAGE_PIXEL)

#define COLOR_INDEX_BLUE            0
#define COLOR_INDEX_GREEN           1
#define COLOR_INDEX_RED             2
#define COLOR_INDEX_ALPHA           3

#define MIN_DEPTH	400
#define MAX_DEPTH	16383
#define UNKNOWN_DEPTH               0
#define UNKNOWN_DEPTH_COLOR         0x003F3F07
#define TOO_NEAR_COLOR              0x001F7FFF
#define TOO_FAR_COLOR               0x007F0F3F
#define NEAREST_COLOR               0x00FFFFFF

#define SUCCESS		1
#define FAIL		0

#define WIDTH	640
#define HEIGHT	480

CKinectConnection::CKinectConnection(void)
	: m_nearMode(false)
	, m_pause(false)
	, m_pColor(NULL)
	, m_pDepth(NULL)
	, m_bSpeckle(FALSE)
	, m_bSpeckleTrigger(FALSE)
	, m_pBuffer(nullptr)
{
	InitDepthColorTable();
}


CKinectConnection::~CKinectConnection(void)
{
}

RGBQUAD m_rgbWk[640*480];

RGBQUAD Nui_ShortToQuad_Depth(USHORT s)
{	
		// USHORT max: 65535
		// 0xfff8: 65528, 1111 1111 1111 1000
		// 0x0fff: 4095,  0000 1111 1111 1111
		
        USHORT realDepth = (s&0xfff8) >> 3;
        //플레이어 비트 정보가 필요할 경우 : USHORT Player = s & 7;
        BYTE l = 255-(BYTE)(256*realDepth /(0x0fff));	// L
 
        RGBQUAD q;
        q.rgbRed = q.rgbBlue = q.rgbGreen = ~l;
        return q;
}


BYTE CKinectConnection::GetIntensity(int depth)
{
    // Validate arguments
    if (depth < MIN_DEPTH || depth > MAX_DEPTH)
    {
        return UCHAR_MAX;
    }

    // Use a logarithmic scale that shows more detail for nearer depths.
    // The constants in this formula were chosen such that values between
    // MIN_DEPTH and MAX_DEPTH will map to the full range of possible
    // byte values.
    const float depthRangeScale = 500.0f;
    const int intensityRangeScale = 74;
    return (BYTE)(~(BYTE)min(
        UCHAR_MAX,
        log((double)(depth - MIN_DEPTH) / depthRangeScale + 1) * intensityRangeScale));
}


void CKinectConnection::SetColor(UINT* pColor, BYTE red, BYTE green, BYTE blue, BYTE alpha)
{
    if (!pColor)
        return;

    BYTE* c = (BYTE*)pColor;
    c[COLOR_INDEX_RED]   = red;
    c[COLOR_INDEX_GREEN] = green;
    c[COLOR_INDEX_BLUE]  = blue;
    c[COLOR_INDEX_ALPHA] = alpha;
}


BYTE* CKinectConnection::ResetBuffer(UINT size)
{
    if (!m_pBuffer || m_nSizeInBytes != size)
    {
        SafeDeleteArray(m_pBuffer);

        if (0 != size)
        {
            m_pBuffer = new BYTE[size];
        }
        m_nSizeInBytes = size;
    }

    return m_pBuffer;
}

void CKinectConnection::InitDepthColorTable()
{
	// Get the min and max reliable depth
    USHORT minReliableDepth = (m_nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
    USHORT maxReliableDepth = (m_nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

    ZeroMemory(m_depthColorTable, sizeof(m_depthColorTable));


	// ------------------------------------------------
	// CLAMP_UNRELIABLE_DEPTHS ------------------------
	// ------------------------------------------------
    // Set color for unknown depth
    m_depthColorTable[0][UNKNOWN_DEPTH] = UNKNOWN_DEPTH_COLOR;

	// Fill in the "near" portion of the table with solid color
    for (int depth = UNKNOWN_DEPTH + 1; depth < minReliableDepth; depth++)
    {
        m_depthColorTable[0][depth] = TOO_NEAR_COLOR;
    }

    // Fill in the "far" portion of the table with solid color
    for (int depth = maxReliableDepth + 1; depth <= USHRT_MAX; depth++)
    {
        m_depthColorTable[0][depth] = TOO_FAR_COLOR;
    }
    


	for (USHORT depth = minReliableDepth; depth <= maxReliableDepth; depth++)
    {
        BYTE intensity = GetIntensity(depth);

        for (int index = 0; index <= MAX_PLAYER_INDEX; index++)
        {
            BYTE r = intensity >> m_intensityShiftR[index];
            BYTE g = intensity >> m_intensityShiftG[index];
            BYTE b = intensity >> m_intensityShiftB[index];
            SetColor(&m_depthColorTable[index][depth], r, g, b);
        }
    }

}




int CKinectConnection::initKinect()
{
	// Get a working kinect sensor
	int numSensors;
	if(NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return FAIL;
	if(NuiCreateSensorByIndex(0, &m_pNuiSensor) < 0) return FAIL;
	m_bSpeckle = FALSE;
	
	HRESULT hr = m_pNuiSensor->NuiInitialize(
		  NUI_INITIALIZE_FLAG_USES_COLOR 
		| NUI_INITIALIZE_FLAG_USES_DEPTH
		);
	
	if(FAILED(hr)) {
		printf("Failed to connect to kinect... \n");
		return FAIL;
	} else {

		// Ensure infrared emitter enabled
		if(SUCCEEDED(hr)) {
			m_pNuiSensor->NuiSetForceInfraredEmitterOff(FALSE);
		}
		m_cFrameReadyEvent = CreateEventW(NULL, TRUE, FALSE, NULL);	// 초기 이벤트 설정, 수동 꺼짐, 초기 설정, 이벤트 이름
		m_dFrameReadyEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

		openColorStream();
		openDepthStream();
		
		return SUCCESS;
	}
}


// Return할 Color image 메모리 할당
void CKinectConnection::ColorMemAlloc()
{
	if(m_pColor != NULL) {
		delete m_pColor; m_pColor=NULL;
	}

	m_pColor = new CDib;
	m_pColor->Allocate(640, 480, 32);
	
}


void CKinectConnection::DepthMemAlloc()
{
	if(m_pDepth != NULL) {
		delete m_pDepth; m_pDepth=NULL;
	}

	m_pDepth = new CDib;
	m_pDepth->Allocate(640, 480, 32);
	
}


void CKinectConnection::openColorStream()
{
	HRESULT hr = m_pNuiSensor->NuiImageStreamOpen(
										NUI_IMAGE_TYPE_COLOR, 
										NUI_IMAGE_RESOLUTION_640x480, 
										0, 
										2, 
										m_cFrameReadyEvent, 
										&videoStreamHandle);
	
	if(SUCCEEDED(hr)) ColorMemAlloc();
}

void CKinectConnection::openInfraredStream()
{
	HRESULT hr = m_pNuiSensor->NuiImageStreamOpen(
										NUI_IMAGE_TYPE_COLOR_INFRARED, 
										NUI_IMAGE_RESOLUTION_640x480, 
										0, 
										2, 
										m_cFrameReadyEvent, 
										&videoStreamHandle);
	
	

	if(SUCCEEDED(hr)) ColorMemAlloc();
}

void CKinectConnection::openDepthStream()
{
	HRESULT hr = m_pNuiSensor->NuiImageStreamOpen(
										NUI_IMAGE_TYPE_DEPTH, 
										NUI_IMAGE_RESOLUTION_640x480,
										0,
										2, 
										m_dFrameReadyEvent,
										&depthStreamHandle);
	

	if(SUCCEEDED(hr)) {
		m_pNuiSensor->NuiImageStreamSetImageFrameFlags(depthStreamHandle, m_nearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);
		DepthMemAlloc();
	}
	
}


void CKinectConnection::closeKinect()
{
	NuiShutdown();
}


#define BYTES_PER_PIXEL_INFRARED	2
#define BYTES_PER_PIXEL_RGB			4

CDib* CKinectConnection::getCDibColorImage()
{
	
	if(WAIT_OBJECT_0 == WaitForSingleObject(m_cFrameReadyEvent, 0))
	{
		HRESULT hr;
		NUI_IMAGE_FRAME colorFrame;
		hr = m_pNuiSensor->NuiImageStreamGetNextFrame(videoStreamHandle, 0, &colorFrame);

		if(FAILED(hr))
		{
			printf("Failed to get a Kinect Image\n");
			return NULL;
		}

		if(m_pause)
		{
			goto ReleaseColorFrame;
		}

		INuiFrameTexture *pTexture = colorFrame.pFrameTexture;

		// Lock the frame data so the Kinect knows not to modify it while we are reading it
		NUI_LOCKED_RECT LockedRectColor;
		pTexture->LockRect(0, &LockedRectColor, NULL, 0);
		

		// Make sure we've received valid data
		if(LockedRectColor.Pitch != 0)
		{
			ColorMemAlloc();	// CDib 컬러 이미지 변수 메모리 할당
			
			
			if(m_bSpeckle==FALSE) {
				BYTE * pBuffer = (BYTE*)LockedRectColor.pBits;

				int width = LockedRectColor.Pitch/sizeof(RGBQUAD);
				int height = LockedRectColor.size/sizeof(RGBQUAD)/width;

				// CDib image pattern point
				unsigned char* pPattern = m_pColor->GetPattern();
		
				int cnt=0;
				pPattern += sizeof(RGBQUAD)*width*(height-1);
				for(int y=0; y<height; y++)
				{
					// ------ 좌우 변경
					for(int x=4*width-4, i=0; i<4*width; x++, i++)
					{
						// LockedRect: BGRA, BGRA... 
						// 이런 순으로 되어 있는데, 1픽셀을 4개의 byte로 나타냄
						// 이 코드는 추가 Flip작업 없이 한 번에 좌우를 맞추기 위함(속도 향상)
						pPattern[i] = pBuffer[x];
						cnt++;
						if(cnt > 3) {
							cnt = 0;
							x = x-2*sizeof(RGBQUAD);
						}
					}
					// ------

					// 좌우 변경 시 위 코드는 주석, 아래 코드를 살림
					//memcpy(pPattern, pBuffer, LockedRect.Pitch);

					pPattern -= sizeof(RGBQUAD)*width;
					pBuffer += LockedRectColor.Pitch;
				}

			} else {
				int width = LockedRectColor.Pitch/BYTES_PER_PIXEL_INFRARED;
				int height = LockedRectColor.size/BYTES_PER_PIXEL_INFRARED/width;

				// CDib image pattern point
				unsigned char* ptr = m_pColor->GetPattern();

				UINT size = LockedRectColor.size;

				// Initialize pixel pointers
				USHORT* pPixelRun = (USHORT*)LockedRectColor.pBits;
				USHORT* pPixelEnd = pPixelRun + size / BYTES_PER_PIXEL_INFRARED;

				
				ptr += width*height*BYTES_PER_PIXEL_RGB - BYTES_PER_PIXEL_RGB;
				// Run through pixels
				while (pPixelRun < pPixelEnd)
				{
					// Convert pixel from 16-bit to 8-bit intensity
					BYTE intensity = (*pPixelRun) >> 8;

					// TODO... Color와 전환 시 가끔씩 튕긴다....
					ptr[0] = intensity;	// B
					ptr[1] = intensity;	// G
					ptr[2] = intensity;	// R
					ptr[3] = 0;

					// Move to next pixel
					++pPixelRun;
					ptr -= BYTES_PER_PIXEL_RGB;
				}

			}

			
		}


		// Unlock frame data
		pTexture->UnlockRect(0);

	ReleaseColorFrame:
		hr = m_pNuiSensor->NuiImageStreamReleaseFrame(videoStreamHandle, &colorFrame);


		return m_pColor;
	}
	
}



CDib* CKinectConnection::getCDibDepthImage()
{
	if (WAIT_OBJECT_0 == WaitForSingleObject(m_dFrameReadyEvent, 0))
	{
		HRESULT hr;
		NUI_IMAGE_FRAME depthFrame, colorFrame;

		// Attemp to get the depth frame
		hr = m_pNuiSensor->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame);	// Get depth frame
		if(FAILED(hr)){ printf("Failed to get a depth frame \n"); return NULL; }
		//hr = m_pNuiSensor->NuiImageStreamGetNextFrame(videoStreamHandle, 20, &colorFrame);	// Get color frame
		//if(FAILED(hr)){ printf("Failed to get a color frame \n"); return NULL; }

		if(m_pause)
		{
			goto ReleaseDepthFrame;
		}
	
		BOOL nearMode;
		INuiFrameTexture *pDepthTex;// = depthFrame.pFrameTexture;
		//INuiFrameTexture* pColorTex = colorFrame.pFrameTexture;

		hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(depthStreamHandle, &depthFrame, &nearMode, &pDepthTex);
		if(FAILED(hr)) return NULL;
		//hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(videoStreamHandle, &colorFrame, &nearMode, &pColorTex);
		//if(FAILED(hr)) return NULL;

		NUI_LOCKED_RECT lockedRectDepth;
		NUI_LOCKED_RECT lockedRectColor;

		

		// Lock the frame data so the Kinect knows not to modify it while we're reading it
		pDepthTex->LockRect(0, &lockedRectDepth, NULL, 0);
		//pColorTex->LockRect(0, &lockedRectColor, NULL, 0);
		// 여기 까지 했음..

		//printf("pBits: %d, Pitch: %d, size: %d \n", LockedRectColorBuffer.pBits, LockedRectColorBuffer.Pitch, LockedRectColorBuffer.size);

		static LONG* pMappedCoords = NULL;

		if(!pMappedCoords) {
			pMappedCoords = new LONG[640*480*2];
		}


		// Make sure we've received valid data
		if(lockedRectDepth.Pitch != 0)
		{
			//// RGB - Depth Calibration code
			//// x1, y1, x2, y2, .... 이런 식으로 calibrated coordinate가 저장됨
			//hr = m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
   //         NUI_IMAGE_RESOLUTION_640x480,
   //         NUI_IMAGE_RESOLUTION_640x480,
   //         640 * 480,							// The number of depth values
			//(unsigned short*) lockedRectDepth.pBits,	// pointer to depth values array
   //         (640 * 480) * 2,					// number of color coordinates
			//pMappedCoords						// calibrated coordination
			//);

			// 여기 손 볼것... TODO
			int width = lockedRectDepth.Pitch/sizeof(RGBQUAD);
			int height = lockedRectDepth.size/sizeof(RGBQUAD)/width;

			//printf("widht: %d height: %d pitch: %d \n", width, height, lockedRectDepth.Pitch);

			// near mode까지 커버 되는 코드..
			// CDib Pointer
			RGBQUAD* pRGB = m_pDepth->GetRgbQuadPointer();
			pRGB = pRGB + width*height;		// move pointer to end
			RGBQUAD* startAddr = pRGB;
			RGBQUAD* endAddr = pRGB+width*height;

			//// Allocate buffer for color image. If required buffer size hasn't changed, the previously allocated buffer is returned
			//UINT* rgb = (UINT*)ResetBuffer(width * height * BYTES_PER_PIXEL_RGB);

			//NUI_DEPTH_IMAGE_PIXEL* pPixelRun = (NUI_DEPTH_IMAGE_PIXEL*)lockedRectDepth.pBits;
			//NUI_DEPTH_IMAGE_PIXEL* pPixelEnd = pPixelRun + width * height;

			//int count = 0;
			//int i=WIDTH;
			//int j=0;

			//while(pPixelRun < pPixelEnd)
			//{
			//	// Get pixel depth and player index
			//	USHORT depth = pPixelRun->depth;
			//	USHORT index = pPixelRun->playerIndex;

			//
			//	// Get mapped color from depth-color table
			//	*rgb = m_depthColorTable[0][depth];
			//	BYTE* pRaw = (BYTE *)rgb;
			//	pRGB->rgbBlue = pRaw[COLOR_INDEX_BLUE];
			//	pRGB->rgbGreen = pRaw[COLOR_INDEX_GREEN];
			//	pRGB->rgbRed = pRaw[COLOR_INDEX_RED];
			//	pRGB->rgbReserved = pRaw[COLOR_INDEX_ALPHA];


			//	// Building a depth map(actual value)
			//	if(j<HEIGHT) {
			//		if(i>0) {
			//			i--;
			//			//m_depthMap[j][i] = depth;
			//		} else {
			//			i=WIDTH-1;
			//			j++;
			//			//m_depthMap[j][i] = depth;
			//		}
			//	}


			//	// Move pointers to the next pixel
			//	pPixelRun++;
			//	pRGB--;

			//	count++;
			//}

			
			
			//printf("width: %d, height: %d \n", width, height);

			// rgbrun에 depth 이미지 복사
			m_imageBuffer.CopyDepth(lockedRectDepth.pBits, 
									lockedRectDepth.size, 
									nearMode, 
									DISPLAY_ALL_DEPTHS, 
									width, 
									height, 
									pRGB, 
									pMappedCoords,
									endAddr);
					
		}

		// Done with the texture. Unlock and release it
		pDepthTex->UnlockRect(0);
		pDepthTex->Release();
		

		delete pMappedCoords;
		pMappedCoords = NULL;
		
	ReleaseDepthFrame:
		hr = m_pNuiSensor->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame);


		return m_pDepth;
	}

}


USHORT CKinectConnection::getDepthValue(int x, int y)
{
	return m_imageBuffer.getDepthValue(x, y);
}


void CKinectConnection::setDepthCalib(BOOL bDepthCalib)
{
	m_imageBuffer.m_calib = bDepthCalib;
}


void CKinectConnection::setSpeckleImage(BOOL bSetSpeckle)
{
	m_bSpeckle = bSetSpeckle;
	m_bSpeckleTrigger = TRUE;
}


BOOL CKinectConnection::getSpeckleTrigger()
{
	return m_bSpeckleTrigger;
}


void CKinectConnection::reOpenColorMode()
{
	if(m_bSpeckleTrigger) {
		if(m_bSpeckle) openInfraredStream();
		else openColorStream();
		m_bSpeckleTrigger=FALSE;
	}
}




// Depth Get code 백업

//DepthMemAlloc();	// CDib Depth 이미지 변수 메모리 할당

//// near mode 커버 안되는 코드..
//// Conver depth data to color image and copy to image buffer
//// Pitch: The number of bytes of data in a row
//// size: The size of pBits, in bytes
//// pBits: A pointer to the upper-left corner of the rectangle

/*printf("Pitch: %d, size: %d, pBits: %d \n", 
	lockedRect.Pitch, lockedRect.size, lockedRect.pBits);*/

			
// lockedRect.size	:	1228800	(Loop: )
// pMappedCoords	:	614400	(Loop: 307200)
// rgbrun			:	307200	(Loop: 307200)


//const NUI_DEPTH_IMAGE_PIXEL* curr = (const NUI_DEPTH_IMAGE_PIXEL*) lockedRect.pBits;
//const NUI_DEPTH_IMAGE_PIXEL* dataEnd = curr + (width*height);
////NUI_DEPTH_IMAGE_PIXEL a;
////printf("a: %d\n", sizeof(a));
////Sleep(3000);

////printf("Pitch: %d, size: %d, pBits: %d \n", 
//	//lockedRect.Pitch, lockedRect.size, lockedRect.pBits);

////printf("%d %d \n", curr, dataEnd);

//int k = 0;
////rgbrun = rgbrun + width*height;

//// Loop : 307200
//while(curr < dataEnd) {
//	// get real depth in milimeters
//	USHORT depth = curr->depth >> 3;
//	curr++;
//	//USHORT depth = NuiDepthPixelToDepth(*curr++);
//	
//	//printf("curr: %d \n", curr);
//	//Sleep(500);
//	UINT
//		x = pMappedCoords[k], // tried with *(pMappedBits + (i * 2)),
//		y = pMappedCoords[k+1]; // tried with *(pMappedBits + (i * 2) + 1);

//	k += 2;

//	if (x >= 0 && x < width && y >= 0 && y < height) {

//		UINT *ptr = (UINT *)rgbrun;

//		
//		rgbrun = endAddr - (x + y*width);
//		//printf("staddr: %d, n1: %d, rgbrun: %d, endAddr: %d \n", startAddr, x+y*width, rgbrun, endAddr);
//		//Sleep(1000);
//		
//		rgbrun->rgbBlue = (BYTE) depth%256;
//		rgbrun->rgbGreen = (BYTE) depth%256;
//		rgbrun->rgbRed = (BYTE) depth%256;
//		rgbrun->rgbReserved = 0xff;

//	}
//	rgbrun--;
//
//}