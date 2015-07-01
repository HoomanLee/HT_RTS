/////////////////////////////////////////////////////////////////////////
// CDib class : bitmap image manipulation
//.......................................................................
// 2004/05/15, Intelligent Robot Research Division, ETRI
/////////////////////////////////////////////////////////////////////////

//#include "ip_struct.h"
//#include "WDib.h"
#ifndef _CDIB_H_
#define _CDIB_H_

#define __WINDOWS               // enables Windows specific functions
#define __VERSION_2013          // enables CDib extension
#include <afxtempl.h>

/*==============================================================*
*																*
*       Related Structures  									*
*                                                               *
*===============================================================*/
#define PI		3.1415927
#define MIRROR(a, size) (a < size ) ? (a) : ( size - 2*(a - (size - 1) ))
#define ABSMAX(pel1, pel2) (max ((int) abs(pel1), (int) abs(pel2)))

template <class T>
inline T square(const T &x) { return x*x; };

typedef CArray<CPoint, CPoint> CPointArray;
typedef CArray<RECT, RECT> RectArray;
typedef CArray<int, int> CIntArray;
typedef CArray<CIntArray *, CIntArray *> CIntArrayArray;
typedef CArray<double, double> CDoubleArray;

typedef struct tag_FPOINT {
	double x;
	double y;
	} FPoint;
typedef CArray<FPoint, FPoint> FPointArray;

/*==============================================================*
*																*
*       Hough Line Structure									*
*                                                               *
*===============================================================*/
typedef struct tag_HLine {
		float rho;
		float theta;
		} HLine;
typedef CArray<HLine, HLine> HLineArray;

/*==============================================================*
*                                                               *
*       class CIPHistogram                                        *
*                                                               *
*===============================================================*/
class CIPHistogram : public CObject
{
// protected data members
protected :
        int m_Size;
        int *m_pData;
        
// constructors and destructors
public:
        CIPHistogram(int size=256);
		CIPHistogram(int* Histogram,int size=256);
        ~CIPHistogram();
        
// public operations
public:
        inline int GetSize();
        inline int GetValue(int index);
        inline int operator[](int index);
        inline void Increment(int index);
        inline void Decrement(int index);
        inline void SetValue(int index, int data);
        inline void ResetAll();
        int SumOfItems(int start=0, int count=0);       // count가 0인 경우, array의 크기(m_Size)와 같은 것으로 취급.
};

inline int CIPHistogram::GetSize()
{
        return m_Size;
}

inline int CIPHistogram::operator[](int index)
{
        return GetValue(index);
}

inline int CIPHistogram::GetValue(int index)
{
#ifdef __DEBUG
        if ((index<0) || (index>=m_Size)) {
          AfxMessageBox("??? CIPHistogram Index Exceeded the Range");
          return 0;
        }
#endif
        return *(m_pData+index);
}

inline void CIPHistogram::Increment(int index)
{
#ifdef __DEBUG
        if ((index<0) || (index>=m_Size)) {
          AfxMessageBox("??? CIPHistogram Index Exceeded the Range");
          return 0;
        }
#endif
        (*(m_pData+index))++;
}

inline void CIPHistogram::Decrement(int index)
{
        (*(m_pData+index))--;
}

inline void CIPHistogram::SetValue(int index, int data)
{
        (*(m_pData+index)) = data;
}

inline void CIPHistogram::ResetAll()
{
        memset(m_pData, 0, sizeof(int) * m_Size);
}

/////////////////////////////////////////////////////////////////////////
// CObBlob
class CObBlob
{
// public data members
public:
    int m_Label;
	int m_Count;
    CRect m_Region;

// constructors and destructors
public:
    CObBlob(int label=-1, int x=-1, int y=-1);
    ~CObBlob();

	void AddPoint(int x, int y);
	CPoint GetCenter();
};

class CObBlobArray : public CArray<CObBlob *, CObBlob *>
{
// public data members
public:

// constructors and destructors
public:
    CObBlobArray();
    ~CObBlobArray();

// protected operations
protected:
	static BOOL CheckExistence(CIntArray& array, int value);
	void DestroyContents();
	static BOOL Intersect(CIntArray *pA1, CIntArray *pA2);
	static void MergeIntArray(CIntArray *pA1, CIntArray *pA2);

// public operations
public:
	void Update(int label, int x, int y);
	CObBlob *Find(int label);
	static BOOL CheckRectProximity(CRect rect1, CRect rect2, int tx, int ty);
	int MergeBlobs(int tx, int ty);
	RectArray *GetRectArray();
	CObBlob *GetLargestBlob();
};

/*==============================================================*
*																*
*       Constants and Data Structures related to the Labeling	*
*                                                               *
*===============================================================*/
/* MIL image file specifications
 */
#define Max                     30000
#define BACKG 			0

extern int num_of_label, max_label;     

#ifndef __LABELINFO
#define __LABELINFO

struct label_info {
       int  count;
       int  gray;
       int  x_start;
       int  x_end;
       int  y_start;
       int  y_end;
       int  hough;
       int  compact;
       int  flag;
       };

//struct label_info lab_info[Max], lab_out[Max];
#endif


/*==============================================================*
*																*
*       CWDib                                               	*
*                                                               *
*===============================================================*/
#define BLOCK_SIZE              16
class CDib;
class CWDib : public CObject
{
// protected data members
protected:
        short *m_pPattern;
        int m_Width;
        int m_Height;
        
// constructors and destructors
public:
        CWDib();
        CWDib(int width, int height);
        CWDib(const char *pFileName);
        ~CWDib();

// Inline Functions
public:
        inline BOOL IsInitialized();
        inline int Width();
        inline int Height();
        inline int GetSizeByte();
        inline int GetSizeWord();
        inline void ResetContents();
        inline short *GetPointer(int x, int y);
        inline short *GetPattern();
        inline short GetValue(int x, int y);
        inline void SetValue(int x, int y, short value);

// Operations
public:
        void Serialize(CArchive& ar);
        BOOL LoadFile(const char *pFileName);
        BOOL SaveFile(const char *pFileName);
        void Allocate(int width, int height);
        void DestroyContent();

        void GetMinMax(short *pMinValue, short *pMaxValue);   
        CDib *GetMappedCDib(double gain=0., double offset=0.);

        CDib *BinarizationLocalMean();
		CDib *ExtractLabeledPattern(int label);
		CWDib *ClipWDib(RECT *pRect);
		CWDib *ClipWDib(int left, int top, int right, int bottom);
		BOOL CopyRect(RECT *pTargetRect, CWDib *pSrWDib, RECT *pSrcRect);
};

inline BOOL CWDib::IsInitialized()
{
        if (m_pPattern) return TRUE;
        else return FALSE;
}

inline int CWDib::Width()
{
        return m_Width;
}

inline int CWDib::Height()
{
        return m_Height;
}

inline int CWDib::GetSizeByte()
{
        return m_Width*m_Height*2;
}

inline int CWDib::GetSizeWord()
{
        return m_Width*m_Height;
}

inline void CWDib::ResetContents()
{
        if (m_pPattern) memset(m_pPattern, 0, GetSizeByte());
}

inline short *CWDib::GetPointer(int x, int y)
{
        if (!m_pPattern) return NULL;
        return m_pPattern + m_Width*y + x;
}

inline short *CWDib::GetPattern()
{
        return m_pPattern;
}

inline short CWDib::GetValue(int x, int y)
{
        short *ptr=GetPointer(x, y);
        if (ptr) return *ptr;
        else return 0;
}

inline void CWDib::SetValue(int x, int y, short value)
{
        short *ptr = GetPointer(x, y);
        if (ptr) *ptr = value;
}

/*==============================================================*
*																*
*       CIDib                                               	*
*                                                               *
*===============================================================*/
class CIDib : public CObject
{
// protected data members
protected:
        int *m_pPattern;
        int m_Width;
        int m_Height;
        
// constructors and destructors
public:
        CIDib();
        CIDib(int width, int height);
        CIDib(const char *pFileName);
        ~CIDib();

// Inline Functions
public:
        inline BOOL IsInitialized();
        inline int Width();
        inline int Height();
        inline int GetSizeByte();
        inline int GetSizeWord();
        inline void ResetContents();
        inline int *GetPointer(int x, int y);
        inline int *GetPattern();
        inline int GetValue(int x, int y);
        inline void SetValue(int x, int y, int value);

// Operations
public:
        void Serialize(CArchive& ar);
        BOOL LoadFile(const char *pFileName);
        BOOL SaveFile(const char *pFileName);
        void Allocate(int width, int height);
        void DestroyContent();

        void GetMinMax(int *pMinValue, int *pMaxValue);   
        CDib *GetMappedCDib(double gain=0., double offset=0.);

        CDib *BinarizationLocalMean();
};

inline BOOL CIDib::IsInitialized()
{
        if (m_pPattern) return TRUE;
        else return FALSE;
}

inline int CIDib::Width()
{
        return m_Width;
}

inline int CIDib::Height()
{
        return m_Height;
}

inline int CIDib::GetSizeByte()
{
        return m_Width*m_Height*sizeof(int);
}

inline int CIDib::GetSizeWord()
{
        return m_Width*m_Height;
}

inline void CIDib::ResetContents()
{
        if (m_pPattern) memset(m_pPattern, 0, GetSizeByte());
}

inline int *CIDib::GetPointer(int x, int y)
{
        if (!m_pPattern) return NULL;
        return m_pPattern + m_Width*y + x;
}

inline int *CIDib::GetPattern()
{
        return m_pPattern;
}

inline int CIDib::GetValue(int x, int y)
{
        int *ptr=GetPointer(x, y);
        if (ptr) return *ptr;
        else return 0;
}

inline void CIDib::SetValue(int x, int y, int value)
{
        int *ptr = GetPointer(x, y);
        if (ptr) *ptr = value;
}

/*==============================================================*
*																*
*       CFDib: float type Dib                                   *
*                                                               *
*===============================================================*/
#define BLOCK_SIZE              16
#define INF		 1E20f
class CFDib : public CObject
{
// protected data members
protected:
        float *m_pPattern;
        int m_Width;
        int m_Height;
        
// constructors and destructors
public:
        CFDib();
        CFDib(int width, int height);
        CFDib(const char *pFileName);
        ~CFDib();

// Inline Functions
public:
        inline BOOL IsInitialized();
        inline int Width();
        inline int Height();
        inline int GetSizeByte();
        inline int GetSizeFloat();
        inline void ResetContents();
        inline float *GetPointer(int x, int y);
        inline float *GetPattern();
        inline float GetValue(int x, int y);
        inline void SetValue(int x, int y, float value);

// Operations
public:
        void Serialize(CArchive& ar);
        BOOL LoadFile(const char *pFileName);
        BOOL SaveFile(const char *pFileName);
        void Allocate(int width, int height);
        void DestroyContent();

        void GetMinMax(float *pMinValue, float *pMaxValue);   
        CDib *GetMappedCDib(double gain=0., double offset=0.);

        CDib *BinarizationLocalMean();

		float *DistanceTransformLine(float *pBuf, int length);
		void DistanceTransform();
		static CDib *DistanceTransform(CDib *pDib);
};

inline BOOL CFDib::IsInitialized()
{
        if (m_pPattern) return TRUE;
        else return FALSE;
}

inline int CFDib::Width()
{
        return m_Width;
}

inline int CFDib::Height()
{
        return m_Height;
}

inline int CFDib::GetSizeByte()
{
        return m_Width*m_Height*sizeof(float);
}

inline int CFDib::GetSizeFloat()
{
        return m_Width*m_Height;
}

inline void CFDib::ResetContents()
{
        if (m_pPattern) {
          float *ptr = m_pPattern;
          for (int i=0; i<(m_Width*m_Height) ;i++) *ptr++ = 0.;
        }
}

inline float *CFDib::GetPointer(int x, int y)
{
        if (!m_pPattern) return NULL;
        return m_pPattern + m_Width*y + x;
}

inline float *CFDib::GetPattern()
{
        return m_pPattern;
}

inline float CFDib::GetValue(int x, int y)
{
        float *ptr=GetPointer(x, y);
        if (ptr) return *ptr;
        else return 0;
}

inline void CFDib::SetValue(int x, int y, float value)
{
        float *ptr = GetPointer(x, y);
        if (ptr) *ptr = value;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//  CDib starts here
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define WIDTHBYTES(bits)        (((bits) + 31) / 32 * 4)
#define TYPE_BITMAP             ((WORD) ('M' << 8) | 'B')

// constants related to Canny Edge Detection Codes
#define BOOSTBLURFACTOR 90.0
#define NOEDGE 255
#define POSSIBLE_EDGE 128
#define EDGE 0
#define M_PI        3.1415927


// Harris Corner related constants
#define HARRISCORNER_HARRIS     1
#define HARRISCORNER_NOBLE      2

class CDib
{
// protected data members
protected:
        unsigned char           *m_pPackedDib;

        // references
        BITMAPINFO              *m_pInfo;
        BITMAPINFOHEADER        *m_pInfoHeader;
        RGBQUAD                 *m_pRgbQuad;
        unsigned char           *m_pPattern;

// static data member
public:
        static unsigned char m_BitMask[8];
		static BOOL m_bUseGrayFast;

        // temporary variable used for the Labeling process
        int num_of_label, max_label;     

// constructor and destructors
public:
        CDib();
        CDib(const char *pFileName);
        CDib(int width, int height, int bitcount, unsigned char *pPattern,
				RGBQUAD *pRgbQuad=NULL);
        ~CDib();

// Protected Operations
protected:
        void ResetVariables();
        void DestroyContents();
        void AssignReferenceVariables();
        BITMAPINFO *GetCopyOfBitmapInfo();
        
        // sub functions for Gaussian smoothing
        void make_gaussian_kernel(float sigma, float **kernel, int *windowsize);
        void gaussian_smooth(unsigned char *image, int rows, int cols, float sigma, short int *smoothedim);

        // sub functions for canny edge detection
        void follow_edges(unsigned char *edgemapptr, short *edgemagptr, short lowval, int cols);
        void apply_hysteresis(short int *mag, unsigned char *nms, int rows, int cols, float tlow, float thigh, unsigned char *edge);
        void non_max_supp(short *mag, short *gradx, short *grady, int nrows, int ncols, unsigned char *result);
        double angle_radians(double x, double y);
        void magnitude_x_y(short int *delta_x, short int *delta_y, int rows, int cols, short int **magnitude);
        void radian_direction(short int *delta_x, short int *delta_y, int rows, int cols, float **dir_radians, int xdirtag, int ydirtag);
        void derivative_x_y(short int *smoothedim, int rows, int cols, short int **delta_x, short int **delta_y);
        void canny(unsigned char *image, int rows, int cols, float sigma, float tlow, float thigh, unsigned char *edge, char *fname);

        // sub functions for HarrisCorner
        void convolve(float *pImage, int width, int height, float *pTemp, float *pKernel, int wsize);
        void ZeroFloat(float *pData, int count);

// File Operations
public:
        BOOL ReadImage(const char *pFileName);
        BOOL SaveImage(const char *pFileName);

// Data Allocation
public:
        // raw pattern을 가지고 있는 경우, CDib를 초기화
        void Initialize(int width, int height, int bitcount, unsigned char *pPattern,
					    RGBQUAD *pRgbQuad=NULL);
        // pattern은 지정하지 않고, memory만 allocate
        void Allocate(int width, int height, int bitcount);
        // Windows의 Packed DIB format에 의한 CDib의 초기화 
        void LoadPackedDib(void *pPackedDib);

// Information Functions
public:
        inline LONG Width();
        inline LONG Height();
        inline WORD BitCount();         // Windows BMP의 경우, bitcount는 1, 4, 8, 16, 24, 32임.
        inline BOOL HavePalette();      // 16 bit 이하의 Dib는 Palette를 가짐.
        inline BITMAPINFO *GetBitmapInfo();
        inline BITMAPINFOHEADER *GetBitmapInfoHeader();
        inline WORD ByteWidth();        // Width in Bytes (aligned to 4 bytes)
        static inline WORD ByteWidth(LONG nbits);
        inline BOOL IsInitialized();

        inline unsigned char *GetPackedDibPointer();
        inline unsigned char *GetPattern();             // pattern부에 대한 pointer
        unsigned char *GetPointer(int x, int y);        // 좌표 (x, y)에 대한 pointer
        unsigned char Intensity(int x, int y);          // 좌표 (x, y)의 pixel의 gray값 (color image의 경우, gray로 conversion 됨)
        unsigned char GetIndex(int x, int y);           // Palette Image의 index, full color의 경우는 0이 return됨
        RGBQUAD GetColor(int x, int y);                 // 좌표 위치 pixel의 color 값
		inline RGBQUAD *GetRgbQuadPointer();
        DWORD PaletteSize();     // size of palette entry data in bytes
        DWORD NumberOfColors();  // number of palette colors
        static DWORD CalcPackedDibSize(BITMAPINFOHEADER *pBi);
        static void RGBtoYIQ(RGBQUAD rgb, unsigned char *y, unsigned char *i, unsigned char *q);
        static unsigned char GetBrightness(RGBQUAD rgb);

// Create New CDib Functions
public:
        // Color CDib로부터 흑백 CDib를 작성
        CDib *GetGrayCDib();
		CDib *GetGrayCDibFast();
        // 똑같은 CDib를 작성
        CDib *CopyCDib();
        // Size를 변경한 CDib를 작성
        CDib *GetSizedCDib(int width, int height=0);
        // Edge를 추출한 CDib를 작성 (8 bit gray image)
        CDib *GetEdgeCDib(double threshold, BOOL bIsGray=false);
		// Clipping of the Image
		CDib *ClipCDib(RECT *pRect);
		CDib *ClipCDib(int left, int top, int right, int bottom);

// Operations
public:
        void SetPaletteEntries(UINT nStartIndex, UINT nNumEntries, PALETTEENTRY *pEntries);     // set color table
        void GetPaletteEntries(UINT nStartIndex, UINT nNumEntries, PALETTEENTRY *pEntries);     // get color table
        void SetGrayPalette();          // set current palette to gray palette
        void Inverse();
        void UpsideDown();              // 패턴의 상하 바꿈
        void ResetContents();           // pattern을 무조건 0으로
        static RGBQUAD Decode16(WORD data);
        static WORD Encode16(unsigned char red, unsigned char green, unsigned char blue);

        // 패턴의 일부를 copy 할 경우
        BOOL CopyRect(RECT *pTargetRect, CDib *pSrCDib, RECT *pSrcRect, int BackColor);

        // Get Image Dimension Without Loading it
        static BOOL GetImageInfoBmp(const char *pFileName, UINT *pWidth, UINT *pHeight, UINT *pBitCount);


// Additional Functions
        BOOL IsGrayImage();
        void SplitChannel(CDib **pDibR, CDib **pDibG, CDib **pDibB);
        static CDib *MergeChannel(CDib *pDibR, CDib *pDibG, CDib *pDibB, int bitcount=24);

// Windows Specific Functions
#ifdef __WINDOWS
public:
        BOOL GetClipboard();
        BOOL SetClipboard();
        void Serialize(CArchive& ar);

// GDI Operations
        //CPalette *GetPalette();
        //CPalette *GetPaletteNoCollapse();
        CPalette *GetPaletteNoFlag();
        CBitmap *GetCBitmap(CDC *pDC=NULL);

        static void PaletteEntryToRgbQuad(PALETTEENTRY *pEntries, RGBQUAD *pRgbQuad, int count); 
        static void RgbQuadToPaletteEntry(RGBQUAD *pRgbQuad, PALETTEENTRY *pEntries, int count);

        // CDib Object를 화면 (pDC)에 그림 
        void LoadImage(CDC *pDC, CRect *pRectTarget=NULL, 
                       CRect *pRectSource=NULL, DWORD dwRop=SRCCOPY);
        // LoadImage와 같은 기능이나, Palette Operation까지를 function내에서 처리
        void LoadPaletteImage(CDC *pDC, CRect *pRectTarget=NULL,
                       CRect *pRectSource=NULL, DWORD dwRop=SRCCOPY);
#endif      // end of __WINDOWS

#ifdef __VERSION_2013
// public operations
public:
    CDib *GetTextImage(const char *pCaption, CRect region, const char *pFontName, int size);

    // Drawing Functions
	void AddPoints(FPointArray *pPoints, COLORREF color);
	void AddPoints(CPointArray *pPoints, COLORREF color);
	void AddRects(RectArray *pRects, COLORREF color);
    void DrawDot(int x, int y, COLORREF color);
	void DrawCross(int x, int y, COLORREF color);
    void DrawLine(int x1, int y1, int x2, int y2, COLORREF color);
    void DrawRect(RECT rect, BOOL bFillIn, COLORREF color);
	void DrawCircle(int xc, int yc, int radius, COLORREF color);
	void DrawEllipse(int xc, int yc, int rx, int ry, COLORREF color);
    void DrawEllipse(double xc, double yc, double rx, double ry, double angle, COLORREF color);
    void DrawText(const char *pCaption, CRect region, const char *pFontName, int size, COLORREF color); 

    void DrawHLines(HLineArray *pLines, COLORREF color);

    // Drawing Functions of Gray Version
	void AddPoints(FPointArray *pPoints, unsigned char value);
	void AddPoints(CPointArray *pPoints, unsigned char value);
	void AddRects(RectArray *pRects, unsigned char value);
    void DrawDot(int x, int y, unsigned char value);
	void DrawCross(int x, int y, unsigned char value);
    void DrawLine(int x1, int y1, int x2, int y2, unsigned char value);
    void DrawRect(RECT rect, BOOL bFillIn, unsigned char value);
	void DrawCircle(int xc, int yc, int radius, unsigned char value);
	void DrawEllipse(int xc, int yc, int rx, int ry, unsigned char value);
    void DrawEllipse(double xc, double yc, double rx, double ry, double angle, unsigned char value);
    void DrawText(const char *pCaption, CRect region, const char *pFontName, int size, unsigned char value);

    void DrawHLines(HLineArray *pLines, unsigned char value);
    
	// Color Conversion Functions
    CDib *GetFullcolorCDib();

	// Other Functions
	unsigned char GetIntensity(COLORREF color);
	unsigned char GetIntensity(unsigned char r, unsigned char g, unsigned char b);
	unsigned char GetIntensityFast(COLORREF color);
	unsigned char GetIntensityFast(unsigned char r, unsigned char g, unsigned char b);
	unsigned char ClipByte(int value);
	CRect ClipRect(CRect rect, CDib *pDib=NULL);
	void SetCircularPalette();

	// Color Format Conversions
	RGBQUAD HSVtoRGB(double h, double s, double v);
	void RGBtoYUV(RGBQUAD rgb, unsigned char *y, unsigned char *u, unsigned char *v);
	RGBQUAD YUVtoRGB(unsigned char y, unsigned char u, unsigned char v);
    void RGBtoHSV(RGBQUAD &rgb, double *h, double *s, double *v);
	BOOL RGBtoYUV(CDib **ppY, CDib **ppU, CDib **ppV);
    CDib *RGBtoYUV();
	void RGBtoYUV_IP();
	CDib *YUVtoRGB(CDib *pY, CDib *pU, CDib *pV);
    CDib *YUVtoRGB(CDib *pYUV);
	void YUVtoRGB_IP();

    void CalculateHistogramMoments(int *pHistogram, int bin, int SumofItem, double pFeature[3]);
    void GetColorMoments(double pFeature[9]);
    void GetMaskedColorMoments(CDib *pBin, double pFeature[9]);
    void GetHSVMoments(double pFeature[9]);
	void GetMaskedHSVMoments(CDib *pMask, double pFeature[9]);
    void GetYUVMoments(double pFeature[9]);
    void GetMaskedYUVMoments(CDib *pBin, double pFeature[9]);

	// simple conversion
	CDib *BlendImage(CDib *pDib1, CDib *pDib2, int weight1, int weight2);
	CDib *Flip_Vertical();
	void Flip_Vertical_IP();
	CDib *Flip_Horizontal();
	void Flip_Horizontal_IP();
	CDib *_Rotate(int degree);
	CDib *RotateLeft();
	CDib *RotateRight();
	CDib *Rotate180();

    //static void MaskedCopy(CDib *pTarget, CDib *pSrc, CDib *pMask);
	double interpolation(double v0, double v1, double v2, double v3, double x);
	CDib* GetResizedDib(float fRatio, int nType);
	CDib* GetResizedDib(int outW, int outH, int nType);

	// arithmetic operators
    CDib *AbsDiff(CDib *pDib1, CDib *pDib2);
	CDib *AbsDiffBin(CDib *pDib1, CDib *pDib2, int threshold);
    CDib *AND(CDib *pDib1, CDib *pDib2);
    CDib *OR(CDib *pDib1, CDib *pDib2);
    CDib *XOR(CDib *pDib1, CDib *pDib2);

	// wrapper for gray image operations
	CIPHistogram *GetHistogram();
	
	CDib *HistogramEqualizationG();
	CDib *HistogramEqualization();
	CDib *HistogramExpansionG();
	CDib *HistogramExpansion();
	CDib *BrightnessG(int brightness);
	CDib *Brightness(int brightness);
	CDib *ContrastG(double contrast);
	CDib *Contrast(double contrast);
	CDib *GammaCorrectionG(double gamma);
	CDib *GammaCorrection(double gamma);
    CDib *GaussianSmoothingG(float sigma);
    CDib *GaussianSmoothing(float sigma);
    CDib *DoG(CDib *pDib1, CDib *pDib2);
    CDib *AbsDoG(CDib *pDib1, CDib *pDib2);
    CDib *DoGFilterG(double sigma1, double sigma2, BOOL bAbs=FALSE, CDib **pSmooth1=NULL, CDib **pSmooth2=NULL);
    CDib *DoGFilterGray(double sigma1, double sigma2, BOOL bAbs=FALSE, CDib **pSmooth1=NULL, CDib **pSmooth2=NULL);
    CDib *DoGFilter(double sigma1, double sigma2, BOOL bAbs=FALSE, CDib **pSmooth1=NULL, CDib **pSmooth2=NULL);

	// Edge Operations
	CWDib *Edge_MagKirsch();
	CWDib *Edge_MagLaplacian();
	CWDib *Edge_MagPrewitt();
	CWDib *Edge_MagRobert();
	CWDib *Edge_MagSobel();
	CDib *Edge_AdaptiveSobel(int divide=150);
	CDib *EdgeB_MagKirsch();
	CDib *EdgeB_MagLaplacian();
	CDib *EdgeB_MagPrewitt();
	CDib *EdgeB_MagRobert();
	CDib *EdgeB_MagSobel();

    CDib *EdgeB_Canny(float sigma=1.0, float tlow=0.2, float thigh=0.9);
    int HarrisCorner(CPointArray& corners, int alg=HARRISCORNER_HARRIS, float K=0.04, float threshold=20000.f, int suppress_range=3);
    int HarrisCornerG(CPointArray& corners, int alg=HARRISCORNER_HARRIS, float K=0.04, float threshold=20000.f, int suppress_range=3);

	// Gray Version of Edge Operations
	CWDib *Edge_MagKirschG();
	CWDib *Edge_MagLaplacianG();
	CWDib *Edge_MagPrewittG();
	CWDib *Edge_MagRobertG();
	CWDib *Edge_MagSobelG();
	CDib *Edge_AdaptiveSobelG(int divide=150);
	CDib *EdgeB_MagKirschG();
	CDib *EdgeB_MagLaplacianG();
	CDib *EdgeB_MagPrewittG();
	CDib *EdgeB_MagRobertG();
	CDib *EdgeB_MagSobelG();

	// Binarization
	void comp_between_group_var(double* Gray_prob,double* BG_var);
	int otsu(int* Gray_hist,int size,int Npix, double *pMaxVariance=NULL);
	CDib *GlobalBinFixed(int threshold);
	CDib *GlobalBinOtsu(int shift=0, int *pThreshold=NULL);
	CDib *LocalBinOtsu();
	
	// Gray Version of Binarization
	CDib *GlobalBinFixedG(int threshold);
	CDib *GlobalBinOtsuG(int shift=0, int *pThreshold=NULL);
	CDib *LocalBinOtsuG();

	// Filters
	static int compare_int(const void *p1, const void *p2);
	CDib *Median3x3();
	CDib *Median5x5();
	CDib *Lowpass3x3();
	CDib *Lowpass5x5();

	CDib *Median3x3G();
	CDib *Median5x5G();
	CDib *Lowpass3x3G();
	CDib *Lowpass5x5G();

	// Morphology Operations
	// SE_size는 반드시 홀수여야 하며, 3-15 사이의 값을 갖는다.
	CDib *Dilation(int SE_size);
	CDib *Erosion(int SE_size);
	CDib *Opening(int SE_size, int repeat=1);
	CDib *Closing(int SE_size, int repeat=1);

	CDib *DilationG(int SE_size);
	CDib *ErosionG(int SE_size);
	CDib *OpeningG(int SE_size, int repeat=1);
	CDib *ClosingG(int SE_size, int repeat=1);

	// Morphology Operations for Binary Images
	// MaskShape: 0(원), 1(사각형)
	// MaskLength: 보통 3에서 5 사이의 값
	BYTE *CreateMorphologyMask(int MaskShape, int MaskLength);
	CDib *DilationB(unsigned char *mask, int MaskLength);
	CDib *DilationB(int MaskShape, int MaskLength);
	CDib *ErosionB(unsigned char *mask, int MaskLength);
	CDib *ErosionB(int MaskShape, int MaskLength);
	CDib *OpeningB(int MaskShape, int MaskLength, int repeat=1);
	CDib *ClosingB(int MaskShape, int MaskLength, int repeat=1);

	// Binary Operations
    void ImageSubtraction(CDib* A, CDib* B);
    int SubThinning(CDib* img, int i, int j, int* a, int* b);
    CDib* Thinning();
	inline CDib *DistanceTransform();

	void cvJacobiEigens_32f (float* A, float* V, float* E, int n, float eps);
	void cvJacobiEigens_64d(double* A, double* V, double* E, int n, double eps);
	void GetCovarianceMatrix(CPoint center, double cov[2][2]);
	double GetOrientationPCA(CPoint center);

	double LogForm(double value);
	void GetInvariantMoments(double value[7]);
	void GetLogScaleInvariantMoments(double value[7]);

	CDib *GetOuterBoundary();

	// Connected Component Analysis
	void enter_equ_table(int a, int b, int equ_table[]);
	void make_mapping_table(CWDib *pWDib, int equ_table[], int mapping_table[], struct label_info lab_info[], struct label_info lab_out[]);
	CObBlobArray *ConnectedComponentAnalysis(CWDib *pLabel=NULL);

	void GetBackground(CDib *pDib, CObBlobArray *pBlobs, char flag[]);
	void FillHole();
	int CountPixel();
	void RemoveNoisyBlobs(int threshold);

	//static CDib *Outline(CDib *pDib, int MaskShape, int MaskLength);



	//-------------------------------------------------------------------------------------------------------------------------------


	////////////////////////////////////////////////////////////////
	//
	//
	//	CDib twLibrary !!! 
	//	written by t.w.kim ~ !!
	//
	////////////////////////////////////////////////////////////////
	
	CDib*	GetTrueMaskedImage(CDib* mask);	// 0/255인 이진 mask CDib 이미지와 and한 이미지를 리턴
	CDib*	RegionGrowing(int seedx, int seedy, double thres = 50.0);
	CDib*	UnsharpMask();
	double	GetSobelOrientation();
	double	GetSobel5x5Orientation();
	CDib*	GetSobelImage();
	CDib*	GetSingleScaleRetinexImage(float sigma);
	CDib*	GetDistortionCorrectedImage(float fx, float fy, float skew, float cx, float cy, float k1, float k2);
	
	

	UCHAR limit(const UCHAR& value);





#endif      // end of __VERSION_2013

};


inline UCHAR CDib::limit(const UCHAR& value)
{
	return ( (value>255) ? 255 : ((value<0) ? 0 : value) );
}



inline LONG CDib::Width()
{
        return m_pInfoHeader->biWidth;
}

inline LONG CDib::Height()
{
        return m_pInfoHeader->biHeight;
}

inline WORD CDib::BitCount()
{
        return m_pInfoHeader->biBitCount;
}

inline BOOL CDib::HavePalette()
{
        return (BitCount()<=8);
}

inline BITMAPINFO * CDib::GetBitmapInfo()
{
        return m_pInfo;
}

inline BITMAPINFOHEADER * CDib::GetBitmapInfoHeader()
{
        return m_pInfoHeader;
}

inline WORD CDib::ByteWidth()
{
        return (WORD) ByteWidth(m_pInfoHeader->biWidth * m_pInfoHeader->biBitCount);
}

inline WORD CDib::ByteWidth(LONG nbits)
{
        return (WORD) WIDTHBYTES(nbits);
}

inline BOOL CDib::IsInitialized()
{
        return (m_pPackedDib!=NULL);
}

inline unsigned char * CDib::GetPattern()
{
        return m_pPattern;
}

inline unsigned char * CDib::GetPackedDibPointer()
{
        return m_pPackedDib;
}

inline RGBQUAD *CDib::GetRgbQuadPointer()
{
    return m_pRgbQuad;
}

inline CDib *CDib::DistanceTransform()
{
	return CFDib::DistanceTransform(this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// CDib Array type
class CDibArray : public CArray<CDib *, CDib *>
{
// constructors and destructors
public:
	CDibArray();
	~CDibArray();

// public operations
public:
	void RemoveTail(int nFrom);
	void ResetContents();
};


#endif //CDib