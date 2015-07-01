/****************************************************************
*                                                               *
*       CDib Class (Bitmap Manipulation Class)               *
*                                                               *
*...............................................................*
*       jLee    revised 97/01/24                                *
*****************************************************************/
//#ifdef _WINDOWS
#include "stdafx.h"
#include "CDibN.h"
//#endif

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <list>

using namespace std;

#ifdef _DEBUG
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

/************************************************************************
*                                                                       *
*       Histogram Manipulation Class                                    *
*                                                                       *
*.......................................................................*
*       /98/02/13       J. Lee                                          *
*************************************************************************/
CIPHistogram::CIPHistogram(int size)
{
        m_Size = size;
        m_pData = new int [m_Size];
        ASSERT(m_pData);
}

CIPHistogram::CIPHistogram(int* Histogram, int size)
{
    	m_Size = size;
		m_pData = new int [m_Size];
		ASSERT(m_pData);
		for(int i=0;i<size;i++)  m_pData[i] =  Histogram[i];
       
}

CIPHistogram::~CIPHistogram()
{
        if (m_pData) delete [] m_pData;
}

int CIPHistogram::SumOfItems(int startIndex, int count)
{
        if (count==0) count = m_Size;

        int sum=0, *ptr=m_pData;
        for (int idx=startIndex, i=0; i<count ;i++, idx++) 
          sum += GetValue(idx);

        return sum;
}

/////////////////////////////////////////////////////////////////////////
// CObBlob
CObBlob::CObBlob(int label, int x, int y)
{
    m_Label = label;
	m_Count = 0;
	m_Region.SetRect(x, y, x, y);
}

CObBlob::~CObBlob()
{
}

void CObBlob::AddPoint(int x, int y)
{
	m_Region.left = min(x, (int) m_Region.left);
	m_Region.top = min(y, (int) m_Region.top);
	m_Region.right = max(x, (int) m_Region.right);
	m_Region.bottom = max(y, (int) m_Region.bottom);
	m_Count++;
}

CPoint CObBlob::GetCenter()
{
	CPoint pt;
	pt.x = (m_Region.left+m_Region.right)/2;
	pt.y = (m_Region.top+m_Region.bottom)/2;

	return pt;
}

///////////////////////////////////////////////////////////////////////////
// CObBlobArray

CObBlobArray::CObBlobArray()
{
}

CObBlobArray::~CObBlobArray()
{
	DestroyContents();
}

void CObBlobArray::DestroyContents()
{
	for (int i=0; i<GetSize() ;i++)
	  delete GetAt(i);
	RemoveAll();
}

CObBlob *CObBlobArray::Find(int label)
{
	for (int i=0; i<GetSize() ;i++)
	  if (label==GetAt(i)->m_Label) return GetAt(i);

	return NULL;
}

BOOL CObBlobArray::CheckRectProximity(CRect rect1, CRect rect2, int tx, int ty)
{
	CRect rect = rect1 & rect2;
	if (!rect.IsRectEmpty()) return TRUE;
	
	int dx, dy;
	if (rect1.left<rect2.left) dx = rect2.left - rect1.right;
	else dx = rect1.left - rect2.right;
	if (dx>tx) return FALSE;
	if (rect1.top<rect2.top) dy = rect2.top - rect1.bottom;
	else dy = rect1.top - rect2.bottom;
	if (dy>ty) return FALSE;

	return TRUE;
}

BOOL CObBlobArray::CheckExistence(CIntArray& array, int id)
{
	for (int i=0; i<array.GetSize() ;i++)
	  if (array[i]==id) return TRUE;

	return FALSE;
}

BOOL CObBlobArray::Intersect(CIntArray *pA1, CIntArray *pA2)
{
	for (int i=0; i<pA2->GetSize() ;i++)
	  if (CheckExistence(*pA1, pA2->GetAt(i))) return TRUE;

	return FALSE;
}

void CObBlobArray::MergeIntArray(CIntArray *pA1, CIntArray *pA2)
{
	for (int i=0; i<pA2->GetSize() ;i++) 
	  if (!CheckExistence(*pA1, pA2->GetAt(i))) pA1->Add(pA2->GetAt(i));
}

int CObBlobArray::MergeBlobs(int tx, int ty)
{
	if (GetSize()<2) return GetSize();

/*
{
TRACE("BEFORE: ");
for (int i=0; i<GetSize() ;i++)
  TRACE("(%d %d %d %d)  ", GetAt(i)->m_Region.left, GetAt(i)->m_Region.top, GetAt(i)->m_Region.right, GetAt(i)->m_Region.bottom);
TRACE("\n");
}
*/
    // 우선은 item 개수만큼 CIntArray를 생성한다.
    CIntArrayArray map;
	int i;
    for (i=0; i<GetSize() ;i++) {
      CIntArray *pArray = new CIntArray;
      map.Add(pArray);
    }

    // 각 item i에 대하여 자신과 가까이 있는 item을 map[i]에 추가한다.
    for (i=0; i<GetSize() ;i++) {
      map[i]->Add(i);
      for (int j=(i+1); j<GetSize() ;j++) 
        if (CheckRectProximity(GetAt(i)->m_Region, GetAt(j)->m_Region, tx, ty)) {
          map[i]->Add(j);
          map[j]->Add(i);
        }
    }

    // merging arrays (동일한 item이 들어 있는 array는 서로 결합한다.
    BOOL bMerge = TRUE;
    while (bMerge) {
      bMerge = FALSE;
	  int index=0;
	  while (index<map.GetSize()) {
	    bMerge = FALSE;
        CIntArray *pProbe = map[index];
        for (int i=(index+1); i<map.GetSize() ;i++)
          if (Intersect(pProbe, map[i])) {
            MergeIntArray(pProbe, map[i]);
		    delete map[i]; map.RemoveAt(i);
            bMerge = TRUE;
            break;
          }
		if (bMerge) index = 0;
		else index++;
	  }
    }

    CObBlobArray newblob;
    for (i=0; i<map.GetSize() ;i++) {
      CObBlob *pBlob = new CObBlob;
      int index = map[i]->GetAt(0);
      pBlob->m_Label = GetAt(index)->m_Label;
      pBlob->m_Count = GetAt(index)->m_Count;
      pBlob->m_Region = GetAt(index)->m_Region;
      for (int j=1; j<map[i]->GetSize() ;j++) {
        index = map[i]->GetAt(j);
        pBlob->m_Count += GetAt(index)->m_Count;
        pBlob->m_Region |= GetAt(index)->m_Region;
      }
      newblob.Add(pBlob);
    }

	// 뒤처리
	for (i=0; i<map.GetSize() ;i++)
	  delete map[i];
	map.RemoveAll();

	DestroyContents();
	Copy(newblob);
	newblob.RemoveAll();	// new blob의 element가 모두 this 로 copy 되었으므로, 다시 delete하면 곤란하므로, RemoveAll을 부른다.

/*
{
TRACE("AFTER: ");
for (int i=0; i<GetSize() ;i++)
  TRACE("(%d %d %d %d)  ", GetAt(i)->m_Region.left, GetAt(i)->m_Region.top, GetAt(i)->m_Region.right, GetAt(i)->m_Region.bottom);
TRACE("\n");
}
*/

	return GetSize();
}

RectArray *CObBlobArray::GetRectArray()
{
	RectArray *pRects = new RectArray;
	for (int i=0; i<GetSize() ;i++)
	  pRects->Add(GetAt(i)->m_Region);

	return pRects;
}

CObBlob *CObBlobArray::GetLargestBlob()
{
	if (GetSize()<=0) return NULL;

	int maxC = GetAt(0)->m_Count;
	int maxID = 0;
	for (int i=1; i<GetSize() ;i++)
		if (GetAt(i)->m_Count>maxC) {
			maxC = GetAt(i)->m_Count;
			maxID = i;
		}

	return GetAt(maxID);
}

unsigned char CDib::m_BitMask[8] = {
        0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01
        };
BOOL CDib::m_bUseGrayFast = FALSE;


/*==============================================================*
*																*
*       CWDib                                               	*
*                                                               *
*===============================================================*/
// Constructors and Destructors
CWDib::CWDib()
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
}

CWDib::CWDib(int width, int height)
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
        Allocate(width, height);
}

CWDib::CWDib(const char *pFileName)
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
        LoadFile(pFileName);
}

CWDib::~CWDib()
{
        DestroyContent();
}

// Operations
void CWDib::Serialize(CArchive& ar)
{
        if (ar.IsStoring()) {
          ar << (DWORD) m_Width;
          ar << (DWORD) m_Height;
          ar.Write(m_pPattern, GetSizeByte());
        }
        else {
          DWORD value;
          ar >> value; m_Width = (int) value;
          ar >> value; m_Height = (int) value;
          m_pPattern = new short [GetSizeWord()];
          ar.Read(m_pPattern, GetSizeByte());
        }
}

BOOL CWDib::LoadFile(const char *pFileName)
{
        CFile file;
        if (!file.Open(pFileName, CFile::modeRead | CFile::typeBinary)) return FALSE;
        CArchive ar(&file, CArchive::load);

        Serialize(ar);

        ar.Close();
        file.Close();

        return TRUE;
}

BOOL CWDib::SaveFile(const char *pFileName)
{
        CFile file;
        if (!file.Open(pFileName, CFile::modeCreate | CFile::modeWrite | CFile::typeBinary)) return FALSE;
        CArchive ar(&file, CArchive::store);

        Serialize(ar);

        ar.Close();
        file.Close();

        return TRUE;
}

void CWDib::Allocate(int width, int height)
{
        DestroyContent();

        m_Width = width;
        m_Height = height;
        m_pPattern = new short [m_Width*m_Height];
}

void CWDib::DestroyContent()
{
        if (m_pPattern) delete [] m_pPattern;
        m_pPattern = NULL;
        m_Width = m_Height = 0;
}


void CWDib::GetMinMax(short *pMinValue, short *pMaxValue)
{
        if (!m_pPattern) {
          *pMinValue = 0;
          *pMaxValue = 0;
          return;
        }

        short *ptr = m_pPattern;        
        *pMinValue = *ptr;
        *pMaxValue = *ptr;
        for (int i=0; i<GetSizeWord() ;i++, ptr++) {
          *pMinValue = min(*ptr, *pMinValue);
          *pMaxValue = max(*ptr, *pMaxValue);
        }
}

CDib *CWDib::GetMappedCDib(double gain, double offset)
{
        if (!IsInitialized()) return NULL;

        if (gain==0.) { // gain and offset are not assigned
          short min_value, max_value;
          GetMinMax(&min_value, &max_value);
          if (min_value==max_value) {   // all of the contents are same
            CDib *pDib = new CDib;
            pDib->Allocate(Width(), Height(), 8);
            pDib->SetGrayPalette();
            pDib->ResetContents();

            return pDib;
          }

          gain = 255./((double) (max_value-min_value));
          offset = -255.*((double) min_value)/((double) (max_value-min_value));
        }

        CDib *pDib = new CDib;
        pDib->Allocate(Width(), Height(), 8);
        pDib->SetGrayPalette();

        for (int y=0; y<Height() ;y++) {
          unsigned char *pChar = pDib->GetPointer(0, y);
          short *pWord = GetPointer(0, y);
          for (int x=0; x<Width() ;x++, pChar++, pWord++) 
            *pChar = (unsigned char) min(255., max(0., (gain*((double) *pWord) + offset)));
        }
	    //pDib->UpsideDown();

        return pDib;
}

CDib *CWDib::BinarizationLocalMean()
{
        CDib *pRes = new CDib;
        pRes->Allocate(Width(), Height(), 8);
        pRes->SetGrayPalette();
        pRes->ResetContents();

        int bWidth = Width()/BLOCK_SIZE;
        int bHeight = Height()/BLOCK_SIZE;
        int *pSumArray = new int[bWidth*bHeight];
        memset(pSumArray, 0, sizeof(int)*bWidth*bHeight);

        int i, j, x, y;
        for (i=0; i<bHeight ;i++)
          for (j=0; j<bWidth ;j++) {
            int sum = 0;
            for (y=0; y<BLOCK_SIZE ;y++) {
              short *pWord = GetPointer(j*BLOCK_SIZE, i*BLOCK_SIZE+y);
              for (x=0; x<BLOCK_SIZE ;x++)
                sum += *pWord++;
            }
            *(pSumArray + i*bWidth + j) = sum;
          }

        for (i=0; i<bHeight-1 ;i++)
          for (j=0; j<bWidth-1 ;j++) {
            int sum = *(pSumArray+i*bWidth+j) + *(pSumArray+i*bWidth+j+1)
                    + *(pSumArray+(i+1)*bWidth+j) + *(pSumArray+(i+1)*bWidth+j+1);
            int threshold = (int) (((double) sum)/((double) (BLOCK_SIZE*BLOCK_SIZE))/2. + 0.5);
            for (y=0; y<BLOCK_SIZE ;y++) {
              short *pWord = GetPointer(BLOCK_SIZE/2+j*BLOCK_SIZE, BLOCK_SIZE/2+i*BLOCK_SIZE+y);
              unsigned char *pChar = pRes->GetPointer(BLOCK_SIZE/2+j*BLOCK_SIZE, BLOCK_SIZE/2+i*BLOCK_SIZE+y);
              for (x=0; x<BLOCK_SIZE ;x++, pWord++, pChar++) {
                if ((*pWord)>=threshold) *pChar = 255;
                else *pChar = 0;
              }
            }
          }

        delete [] pSumArray;

        return pRes;
}

CDib *CWDib::ExtractLabeledPattern(int label)
{
	CDib *pDib = new CDib;
	pDib->Allocate(Width(), Height(), 8);
	pDib->SetGrayPalette();

	for (int i=0; i<Height() ;i++) {
		unsigned char *ptr = pDib->GetPointer(0, i);
		short *pW = GetPointer(0, i);
		for (int j=0; j<Width() ;j++, ptr++, pW++) {
			if (*pW==(short) label) *ptr = 255;
			else *ptr = 0;
		}
	}

	return pDib;
}

CWDib *CWDib::ClipWDib(RECT *pRect)
{
	return ClipWDib(pRect->left, pRect->top, pRect->right, pRect->bottom);
}

CWDib *CWDib::ClipWDib(int left, int top, int right, int bottom)
{
	RECT target;
	target.left = max(left, 0);
	target.top = max(top, 0);
	target.right = min(right, (int) Width());
	target.bottom = min(bottom, (int) Height());
	int tWidth = target.right - target.left;
	int tHeight = target.bottom - target.top;

	CWDib *pRes = new CWDib;
	pRes->Allocate(tWidth, tHeight);

	int bwidth = tWidth * sizeof(short);
	for (int i=0; i<tHeight ;i++) 
	  memcpy(pRes->GetPointer(0, i), GetPointer(target.left, target.top+i), bwidth);

    return pRes;
}

BOOL CWDib::CopyRect(RECT *pTargetRect, CWDib *pSrWDib, RECT *pSrcRect)
{
	int tWidth = pTargetRect->right - pTargetRect->left;
	int tHeight = pTargetRect->bottom - pTargetRect->top;
	int sWidth = pSrcRect->right - pSrcRect->left;
	int sHeight = pSrcRect->bottom - pSrcRect->top;

    if ((tWidth!=sWidth) || (tHeight!=sHeight)) {
      AfxMessageBox("??? Can't Copy Rect Region with Different Size\n");
      return false;
    }

    for (int i=0; i<tHeight ;i++) {
        short *pTarget = GetPointer(pTargetRect->left, pTargetRect->top+i);
        short *pSrc = pSrWDib->GetPointer(pSrcRect->left, pSrcRect->top+i);
        memcpy(pTarget, pSrc, sWidth*sizeof(short));
    }

    return true;
}

/*==============================================================*
*																*
*       CIDib                                               	*
*                                                               *
*===============================================================*/
// Constructors and Destructors
CIDib::CIDib()
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
}

CIDib::CIDib(int width, int height)
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
        Allocate(width, height);
}

CIDib::CIDib(const char *pFileName)
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
        LoadFile(pFileName);
}

CIDib::~CIDib()
{
        DestroyContent();
}

// Operations
void CIDib::Serialize(CArchive& ar)
{
        if (ar.IsStoring()) {
          ar << (DWORD) m_Width;
          ar << (DWORD) m_Height;
          ar.Write(m_pPattern, GetSizeByte());
        }
        else {
          DWORD value;
          ar >> value; m_Width = (int) value;
          ar >> value; m_Height = (int) value;
          m_pPattern = new int [GetSizeWord()];
          ar.Read(m_pPattern, GetSizeByte());
        }
}

BOOL CIDib::LoadFile(const char *pFileName)
{
        CFile file;
        if (!file.Open(pFileName, CFile::modeRead | CFile::typeBinary)) return FALSE;
        CArchive ar(&file, CArchive::load);

        Serialize(ar);

        ar.Close();
        file.Close();

        return TRUE;
}

BOOL CIDib::SaveFile(const char *pFileName)
{
        CFile file;
        if (!file.Open(pFileName, CFile::modeCreate | CFile::modeWrite | CFile::typeBinary)) return FALSE;
        CArchive ar(&file, CArchive::store);

        Serialize(ar);

        ar.Close();
        file.Close();

        return TRUE;
}

void CIDib::Allocate(int width, int height)
{
        DestroyContent();

        m_Width = width;
        m_Height = height;
        m_pPattern = new int [m_Width*m_Height];
}

void CIDib::DestroyContent()
{
        if (m_pPattern) delete [] m_pPattern;
        m_pPattern = NULL;
        m_Width = m_Height = 0;
}


void CIDib::GetMinMax(int *pMinValue, int *pMaxValue)
{
        if (!m_pPattern) {
          *pMinValue = 0;
          *pMaxValue = 0;
          return;
        }

        int *ptr = m_pPattern;        
        *pMinValue = *ptr;
        *pMaxValue = *ptr;
        for (int i=0; i<GetSizeWord() ;i++, ptr++) {
          *pMinValue = min(*ptr, *pMinValue);
          *pMaxValue = max(*ptr, *pMaxValue);
        }
}

CDib *CIDib::GetMappedCDib(double gain, double offset)
{
        if (!IsInitialized()) return NULL;

        if (gain==0.) { // gain and offset are not assigned
          int min_value, max_value;
          GetMinMax(&min_value, &max_value);
          if (min_value==max_value) {   // all of the contents are same
            CDib *pDib = new CDib;
            pDib->Allocate(Width(), Height(), 8);
            pDib->SetGrayPalette();
            pDib->ResetContents();

            return pDib;
          }

          gain = 255./((double) (max_value-min_value));
          offset = -255.*((double) min_value)/((double) (max_value-min_value));
        }

        CDib *pDib = new CDib;
        pDib->Allocate(Width(), Height(), 8);
        pDib->SetGrayPalette();

        for (int y=0; y<Height() ;y++) {
          unsigned char *pChar = pDib->GetPointer(0, y);
          int *pWord = GetPointer(0, y);
          for (int x=0; x<Width() ;x++, pChar++, pWord++) 
            *pChar = (unsigned char) min(255., max(0., (gain*((double) *pWord) + offset)));
        }
	    //pDib->UpsideDown();

        return pDib;
}

CDib *CIDib::BinarizationLocalMean()
{
        CDib *pRes = new CDib;
        pRes->Allocate(Width(), Height(), 8);
        pRes->SetGrayPalette();
        pRes->ResetContents();

        int bWidth = Width()/BLOCK_SIZE;
        int bHeight = Height()/BLOCK_SIZE;
        int *pSumArray = new int[bWidth*bHeight];
        memset(pSumArray, 0, sizeof(int)*bWidth*bHeight);

        int i, j, x, y;
        for (i=0; i<bHeight ;i++)
          for (j=0; j<bWidth ;j++) {
            int sum = 0;
            for (y=0; y<BLOCK_SIZE ;y++) {
              int *pWord = GetPointer(j*BLOCK_SIZE, i*BLOCK_SIZE+y);
              for (x=0; x<BLOCK_SIZE ;x++)
                sum += *pWord++;
            }
            *(pSumArray + i*bWidth + j) = sum;
          }

        for (i=0; i<bHeight-1 ;i++)
          for (j=0; j<bWidth-1 ;j++) {
            int sum = *(pSumArray+i*bWidth+j) + *(pSumArray+i*bWidth+j+1)
                    + *(pSumArray+(i+1)*bWidth+j) + *(pSumArray+(i+1)*bWidth+j+1);
            int threshold = (int) (((double) sum)/((double) (BLOCK_SIZE*BLOCK_SIZE))/2. + 0.5);
            for (y=0; y<BLOCK_SIZE ;y++) {
              int *pWord = GetPointer(BLOCK_SIZE/2+j*BLOCK_SIZE, BLOCK_SIZE/2+i*BLOCK_SIZE+y);
              unsigned char *pChar = pRes->GetPointer(BLOCK_SIZE/2+j*BLOCK_SIZE, BLOCK_SIZE/2+i*BLOCK_SIZE+y);
              for (x=0; x<BLOCK_SIZE ;x++, pWord++, pChar++) {
                if ((*pWord)>=threshold) *pChar = 255;
                else *pChar = 0;
              }
            }
          }

        delete [] pSumArray;

        return pRes;
}

/*==============================================================*
*																*
*       CFDib                                               	*
*                                                               *
*===============================================================*/
// Constructors and Destructors
CFDib::CFDib()
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
}

CFDib::CFDib(int width, int height)
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
        Allocate(width, height);
}

CFDib::CFDib(const char *pFileName)
{
        m_pPattern = NULL;
        m_Width = m_Height = 0;
        LoadFile(pFileName);
}

CFDib::~CFDib()
{
        DestroyContent();
}

// Operations
void CFDib::Serialize(CArchive& ar)
{
        if (ar.IsStoring()) {
          ar << (DWORD) m_Width;
          ar << (DWORD) m_Height;
          ar.Write(m_pPattern, GetSizeByte());
        }
        else {
          DWORD value;
          ar >> value; m_Width = (int) value;
          ar >> value; m_Height = (int) value;
          m_pPattern = new float [GetSizeFloat()];
          ar.Read(m_pPattern, GetSizeByte());
        }
}

BOOL CFDib::LoadFile(const char *pFileName)
{
        CFile file;
        if (!file.Open(pFileName, CFile::modeRead | CFile::typeBinary)) return FALSE;
        CArchive ar(&file, CArchive::load);

        Serialize(ar);

        ar.Close();
        file.Close();

        return TRUE;
}

BOOL CFDib::SaveFile(const char *pFileName)
{
        CFile file;
        if (!file.Open(pFileName, CFile::modeCreate | CFile::modeWrite | CFile::typeBinary)) return FALSE;
        CArchive ar(&file, CArchive::store);

        Serialize(ar);

        ar.Close();
        file.Close();

        return TRUE;
}

void CFDib::Allocate(int width, int height)
{
        DestroyContent();

        m_Width = width;
        m_Height = height;
        m_pPattern = new float [m_Width*m_Height];
}

void CFDib::DestroyContent()
{
        if (m_pPattern) delete [] m_pPattern;
        m_pPattern = NULL;
        m_Width = m_Height = 0;
}


void CFDib::GetMinMax(float *pMinValue, float *pMaxValue)
{
        if (!m_pPattern) {
          *pMinValue = 0;
          *pMaxValue = 0;
          return;
        }

        float *ptr = m_pPattern;        
        *pMinValue = *ptr;
        *pMaxValue = *ptr;
        for (int i=0; i<GetSizeFloat() ;i++, ptr++) {
          *pMinValue = min(*ptr, *pMinValue);
          *pMaxValue = max(*ptr, *pMaxValue);
        }
}

CDib *CFDib::GetMappedCDib(double gain, double offset)
{
        if (!IsInitialized()) return NULL;

        if (gain==0.) { // gain and offset are not assigned
          float min_value, max_value;
          GetMinMax(&min_value, &max_value);
          if (min_value==max_value) {   // all of the contents are same
            CDib *pDib = new CDib;
            pDib->Allocate(CDib::ByteWidth(Width()*8), Height(), 8);
            pDib->SetGrayPalette();
            pDib->ResetContents();

            return pDib;
          }

          gain = 255./((double) (max_value-min_value));
          offset = 255.*((double) min_value)/((double) (max_value-min_value));
        }

        CDib *pDib = new CDib;
        pDib->Allocate(CDib::ByteWidth(Width()*8), Height(), 8);
        pDib->SetGrayPalette();

        for (int y=0; y<Height() ;y++) {
          unsigned char *pChar = pDib->GetPointer(0, y);
          float *pFloat = GetPointer(0, y);
          for (int x=0; x<Width() ;x++, pChar++, pFloat++) 
            *pChar = (unsigned char) min(255., max(0., (gain*((double) *pFloat) + offset)));
        }
	    pDib->UpsideDown();

        return pDib;
}

CDib *CFDib::BinarizationLocalMean()
{
        CDib *pRes = new CDib;
        pRes->Allocate(CDib::ByteWidth(Width()*8), Height(), 8);
        pRes->SetGrayPalette();
        pRes->ResetContents();

        int bWidth = Width()/BLOCK_SIZE;
        int bHeight = Height()/BLOCK_SIZE;
        float *pSumArray = new float[bWidth*bHeight];
        memset(pSumArray, 0, sizeof(int)*bWidth*bHeight);

        int i, j, x, y;
        for (i=0; i<bHeight ;i++)
          for (j=0; j<bWidth ;j++) {
            float sum = 0;
            for (y=0; y<BLOCK_SIZE ;y++) {
              float *pFloat = GetPointer(j*BLOCK_SIZE, i*BLOCK_SIZE+y);
              for (x=0; x<BLOCK_SIZE ;x++)
                sum += *pFloat++;
            }
            *(pSumArray + i*bWidth + j) = sum;
          }

        for (i=0; i<bHeight-1 ;i++)
          for (j=0; j<bWidth-1 ;j++) {
            float sum = *(pSumArray+i*bWidth+j) + *(pSumArray+i*bWidth+j+1)
                    + *(pSumArray+(i+1)*bWidth+j) + *(pSumArray+(i+1)*bWidth+j+1);
            float threshold = (float) (((double) sum)/((double) (BLOCK_SIZE*BLOCK_SIZE))/2. + 0.5);
            for (y=0; y<BLOCK_SIZE ;y++) {
              float *pFloat = GetPointer(BLOCK_SIZE/2+j*BLOCK_SIZE, BLOCK_SIZE/2+i*BLOCK_SIZE+y);
              unsigned char *pChar = pRes->GetPointer(BLOCK_SIZE/2+j*BLOCK_SIZE, BLOCK_SIZE/2+i*BLOCK_SIZE+y);
              for (x=0; x<BLOCK_SIZE ;x++, pFloat++, pChar++) {
                if ((*pFloat)>=threshold) *pChar = 255;
                else *pChar = 0;
              }
            }
          }

        delete [] pSumArray;

        return pRes;
}


float *CFDib::DistanceTransformLine(float *pBuf, int length)
{
	float *pRes = new float [length];
	int *pV = new int [length];
	float *pZ = new float [length+1];

	int k = 0;
	pV[0] = 0;
	pZ[0] = -INF;
	pZ[1] = INF;
	int q;
	for (q=1; q<length ;q++) {
		float s = ((pBuf[q] + square(q))-(pBuf[pV[k]]+square(pV[k])))/(2*q-2*pV[k]);
		while (s<=pZ[k]) {
			k--;
			s = ((pBuf[q] + square(q))-(pBuf[pV[k]]+square(pV[k])))/(2*q-2*pV[k]);
		}
		k++;
		pV[k] = q;
		pZ[k] = s;
		pZ[k+1] = INF;
	}

	k = 0;
	for (q=0; q<length ;q++) {
		while (pZ[k+1]<q) 
			k++;
		pRes[q] = square(q-pV[k]) + pBuf[pV[k]];
	}

	delete [] pV;
	delete [] pZ;

	return pRes;
}

void CFDib::DistanceTransform()
{
	float *pBuf = new float [max(Width(), Height())];

	// columns
	int x, y;
	for (x=0; x<Width() ;x++) {
		for (y=0; y<Height() ;y++)
			pBuf[y] = *GetPointer(x, y);
		float *pRes = DistanceTransformLine(pBuf, Height());
		for (y=0; y<Height() ;y++)
			*GetPointer(x, y) = pRes[y];
		delete [] pRes;
	}

	// rows
	for (y=0; y<Height() ;y++) {
		float *ptr = GetPointer(0, y);
		for (x=0; x<Width() ;x++)
			pBuf[x] = *ptr++;
		float *pRes = DistanceTransformLine(pBuf, Width());

		ptr = GetPointer(0, y);
		for (x=0; x<Width() ;x++)
			*ptr++ = pRes[x];
		delete [] pRes;
	}

	delete [] pBuf;
}

CDib *CFDib::DistanceTransform(CDib *pDib)
{
	CFDib *pFDib = new CFDib;
	pFDib->Allocate(pDib->Width(), pDib->Height());

	int skip = pDib->ByteWidth() - pDib->Width();
	unsigned char *ptr = pDib->GetPattern();
	float *pOut = pFDib->GetPattern();
	for (int i=0; i<pDib->Height() ;i++) {
		for (int j=0; j<pDib->Width() ;j++, ptr++, pOut++)
			if (*ptr!=0) *pOut = 0;
			else *pOut = INF;
		ptr += skip;
	}

	pFDib->DistanceTransform();
	CDib *pRes = pFDib->GetMappedCDib();

	delete pFDib;
	return pRes;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//  CDib starts here
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Constructors and Destructors
CDib::CDib()
{
        ResetVariables();
}

CDib::CDib(const char *pFileName)
{
        ResetVariables();
        ReadImage(pFileName);
}

CDib::CDib(int width, int height, int bitcount, unsigned char *pPattern, RGBQUAD *pRgbQuad)
{
        ResetVariables();
        Initialize(width, height, bitcount, pPattern, pRgbQuad);
}

CDib::~CDib()
{
        DestroyContents();
}

/////////////////////////////////////////////////////////////////
// Protected Operations
void CDib::ResetVariables()
{
        m_pPackedDib = NULL;
        m_pInfo = NULL;
        m_pInfoHeader = NULL;
        m_pRgbQuad = NULL;
        m_pPattern = NULL;
}

void CDib::DestroyContents()
{
        delete [] m_pPackedDib;
        ResetVariables();
}

void CDib::AssignReferenceVariables()
// AssignReferenceVariables는 최소한 BITMAPINFOHEADER가 setting된 이후 call 해야 한다.
{
        m_pInfo = (BITMAPINFO *) m_pPackedDib;
        m_pInfoHeader = (BITMAPINFOHEADER *) m_pPackedDib;
        m_pRgbQuad = (RGBQUAD *) (m_pPackedDib + sizeof(BITMAPINFOHEADER));
        m_pPattern = m_pPackedDib + sizeof(BITMAPINFOHEADER) + PaletteSize();
}

BITMAPINFO *CDib::GetCopyOfBitmapInfo()
{
        if (!IsInitialized()) return NULL;

        UINT size = sizeof(BITMAPINFOHEADER) + (UINT) PaletteSize();
        BITMAPINFO *pInfo = (BITMAPINFO *) (new unsigned char [size]);

        memcpy(pInfo, m_pInfo, size);

        return pInfo;
}

/////////////////////////////////////////////////////////////////
// Static Functions
DWORD CDib::CalcPackedDibSize(BITMAPINFOHEADER *pBi)
{
        DWORD size = sizeof(BITMAPINFOHEADER);
        
        switch (pBi->biBitCount) {
          case 1 :
            size += ( sizeof(RGBQUAD)*2 + ByteWidth(pBi->biWidth*pBi->biBitCount)*pBi->biHeight );
            break;
          case 4 :
            size += ( sizeof(RGBQUAD)*16 + ByteWidth(pBi->biWidth*pBi->biBitCount)*pBi->biHeight );
            break;
          case 8 :
            size += ( sizeof(RGBQUAD)*256 + ByteWidth(pBi->biWidth*pBi->biBitCount)*pBi->biHeight );
            break;
          case 16 :
          case 24 :
          case 32 :
            size += ( ByteWidth(pBi->biWidth*pBi->biBitCount)*pBi->biHeight );
            break;
          default :
            break;
        }

        return size;
}

void CDib::RGBtoYIQ(RGBQUAD rgb, unsigned char *y, unsigned char *i, unsigned char *q)
{
        *y = (unsigned char) ( 0.299 * (double) rgb.rgbRed 
                             + 0.587 * (double) rgb.rgbGreen
                             + 0.114 * (double) rgb.rgbBlue + 0.5);
        *i = (unsigned char) ( 0.596 * (double) rgb.rgbRed 
                             + 0.274 * (double) rgb.rgbGreen
                             + 0.311 * (double) rgb.rgbBlue + 0.5);
        *q = (unsigned char) ( 0.211 * (double) rgb.rgbRed 
                             + 0.522 * (double) rgb.rgbGreen
                             + 0.299 * (double) rgb.rgbBlue + 0.5);
}

unsigned char CDib::GetBrightness(RGBQUAD rgb)
{
        return (unsigned char) ( 0.299 * (double) rgb.rgbRed 
                             + 0.587 * (double) rgb.rgbGreen
                             + 0.114 * (double) rgb.rgbBlue + 0.5);
}

BOOL CDib::ReadImage(const char *pFileName)
{
    if (IsInitialized()) DestroyContents();

	BITMAPFILEHEADER bf;
	BITMAPINFOHEADER bi;

	//FILE *fp = fopen(pFileName, "rb");
    //if (!fp) {
	FILE *fp;
	errno_t err	= fopen_s(&fp, pFileName, "rb");
	if (err) {
      printf("! File Open Error\n");
	  return false;
	}

	fread(&bf.bfType, 2, 1, fp);
    fread(&bf.bfSize, 4, 1, fp);
	fread(&bf.bfReserved1, 2, 1, fp);
	fread(&bf.bfReserved2, 2, 1, fp);
	fread(&bf.bfOffBits, 4, 1, fp);
	//fread(&bf, sizeof(BITMAPFILEHEADER), 1, fp);
	if (bf.bfType!=TYPE_BITMAP) // this file is NOT a BMP file
		return false; 

    fread(&bi.biSize, sizeof(LONG), 1, fp);
    if (bi.biSize==40) {    // Ordinary BITHAMPINFOHEADER
      fread(&bi.biWidth, sizeof(LONG), 1, fp);
      fread(&bi.biHeight, sizeof(LONG), 1, fp);
      fread(&bi.biPlanes, sizeof(WORD), 1, fp);
      fread(&bi.biBitCount, sizeof(WORD), 1, fp);
      fread(&bi.biCompression, sizeof(DWORD), 1, fp);
      fread(&bi.biSizeImage, sizeof(DWORD), 1, fp);
      fread(&bi.biXPelsPerMeter, sizeof(LONG), 1, fp);
      fread(&bi.biYPelsPerMeter, sizeof(LONG), 1, fp);
      fread(&bi.biClrUsed, sizeof(LONG), 1, fp);
      fread(&bi.biClrImportant, sizeof(LONG), 1, fp);
    }
    else if (bi.biSize==12) {
      WORD temp;
      fread(&temp, sizeof(WORD), 1, fp);
      bi.biWidth = (LONG) temp;
      fread(&temp, sizeof(WORD), 1, fp);
      bi.biHeight = (LONG) temp;
      fread(&bi.biPlanes, sizeof(WORD), 1, fp);
      fread(&bi.biBitCount, sizeof(WORD), 1, fp);
      
      bi.biCompression = BI_RGB;
      bi.biSizeImage = 0;
      bi.biXPelsPerMeter = 0;
      bi.biYPelsPerMeter = 0;
      bi.biClrUsed = 0;
      bi.biClrImportant = 0;

      bi.biSize = 40;
    }

    int p_size; 
    switch (bi.biBitCount) {
      case 1 : p_size = 2 * sizeof(PALETTEENTRY); break;
      case 4 : p_size = 16 * sizeof(PALETTEENTRY); break;
      case 8 : p_size = 256 * sizeof(PALETTEENTRY); break;
      default : p_size = 0;
    }
    
	int skip = bf.bfOffBits - sizeof(BITMAPFILEHEADER) - bi.biSize - p_size;
    if (skip>0) 	// skip additional info
	  fseek(fp, skip, SEEK_CUR);

    DWORD size = CalcPackedDibSize(&bi);

    m_pPackedDib = new unsigned char[size];
    memcpy(m_pPackedDib, &bi, sizeof(BITMAPINFOHEADER));

	fread(m_pPackedDib+sizeof(BITMAPINFOHEADER), size-sizeof(BITMAPINFOHEADER), 1, fp);

    AssignReferenceVariables();

	fclose(fp);

	return true;
}

BOOL CDib::SaveImage(const char *pFileName)
{
    if (!IsInitialized()) return false;

    // write bitmap file header
    BITMAPFILEHEADER bf;
    bf.bfType = TYPE_BITMAP;
    bf.bfOffBits = (DWORD) 14 //sizeof(BITMAPFILEHEADER)
                 + (DWORD) sizeof(BITMAPINFOHEADER)
                 + (DWORD) PaletteSize();
    bf.bfSize = bf.bfOffBits + ByteWidth()*Height();
    bf.bfReserved1 = bf.bfReserved2 = 0;

	//FILE *fp = fopen(pFileName, "wb");
    //if (!fp) return false;
	FILE *fp;
	errno_t err = fopen_s(&fp, pFileName, "wb");
	if (err) return false;

	//fwrite(&bf, sizeof(BITMAPFILEHEADER), 1, fp);
	fwrite(&bf.bfType, 2, 1, fp);
	fwrite(&bf.bfSize, 4, 1, fp);
	fwrite(&bf.bfReserved1, 2, 1, fp);
	fwrite(&bf.bfReserved2, 2, 1, fp);
	fwrite(&bf.bfOffBits, 4, 1, fp);
	fwrite(m_pInfoHeader, sizeof(BITMAPINFOHEADER), 1, fp);

    // write palette
    if (PaletteSize()>0) fwrite(m_pRgbQuad, PaletteSize(), 1, fp);

    // write image data
	fwrite(m_pPattern, ByteWidth(), Height(), fp);

	fclose(fp);

    return true;
}

BOOL CDib::GetImageInfoBmp(const char *pFileName, UINT *pWidth, UINT *pHeight, UINT *pBitCount)
{
	//FILE *fp = fopen(pFileName, "rb");
    //if (!fp) return false;
	FILE *fp;
	errno_t err = fopen_s(&fp, pFileName, "rb");
	if (err) return false;

    fseek(fp, 14, SEEK_SET);
    BITMAPINFOHEADER bi;
    fread(&bi, sizeof(BITMAPINFOHEADER), 1, fp);

    *pWidth = bi.biWidth;
    *pHeight = bi.biHeight;
    *pBitCount = bi.biBitCount;

    return true;
}

void CDib::Initialize(int width, int height, int bitcount, unsigned char *pPattern,
						 RGBQUAD *pRgbQuad)
{
        // if data already exist, destroy the contents
        if (IsInitialized()) DestroyContents();

        // assign bitmap info header
        BITMAPINFOHEADER biInfo;
        biInfo.biSize = sizeof(BITMAPINFOHEADER);
        biInfo.biWidth = (LONG) width;
        biInfo.biHeight = (LONG) height;
        biInfo.biPlanes = 1;
        biInfo.biBitCount = bitcount;
        biInfo.biCompression = BI_RGB;
        biInfo.biSizeImage = 0;
        biInfo.biXPelsPerMeter = 0;
        biInfo.biYPelsPerMeter = 0;
        biInfo.biClrUsed = 0;
        biInfo.biClrImportant = 0;
        		
        // memory allocation
        DWORD size = CalcPackedDibSize(&biInfo);
        m_pPackedDib = new unsigned char [size];
                
        // copy bitmap info header                                        
        memcpy(m_pPackedDib, &biInfo, sizeof(BITMAPINFOHEADER));

        AssignReferenceVariables();

        // copy palette table
        switch (bitcount) {
          case 1 :
		    memcpy(m_pRgbQuad, pRgbQuad, sizeof(RGBQUAD)*2);
            break;
          case 4 :
		    memcpy(m_pRgbQuad, pRgbQuad, sizeof(RGBQUAD)*16);
            break;
          case 8 :
		    memcpy(m_pRgbQuad, pRgbQuad, sizeof(RGBQUAD)*256);
            break;
          case 16 :
          case 24 :
          case 32 :
          default :
            break;
        }
		
        // copy pattern
        memcpy(m_pPattern, pPattern, ByteWidth(width*bitcount)*height);
}

void CDib::Allocate(int width, int height, int bitcount)
{
        // if data already exist, destroy the contents
        if (IsInitialized()) DestroyContents();

        // assign bitmap info header
        BITMAPINFOHEADER biInfo;
        biInfo.biSize = sizeof(BITMAPINFOHEADER);	// 40
        biInfo.biWidth = (LONG) width;
        biInfo.biHeight = (LONG) height;
        biInfo.biPlanes = 1;
        biInfo.biBitCount = bitcount;
        biInfo.biCompression = BI_RGB;
        biInfo.biSizeImage = 0;
        biInfo.biXPelsPerMeter = 0;
        biInfo.biYPelsPerMeter = 0;
        biInfo.biClrUsed = 0;
        biInfo.biClrImportant = 0;
        
        // memory allocation
        DWORD size = CalcPackedDibSize(&biInfo);
        m_pPackedDib = new unsigned char [size];
        memset(m_pPackedDib, 0, size);
                
        // copy bitmap info header                                        
        memcpy(m_pPackedDib, &biInfo, sizeof(BITMAPINFOHEADER));

        AssignReferenceVariables();
}          

void CDib::LoadPackedDib(void *pPackedDib)
{
        if (IsInitialized()) DestroyContents();

        DWORD size = CalcPackedDibSize((BITMAPINFOHEADER *) pPackedDib);
        m_pPackedDib = new unsigned char [size];

        memcpy(m_pPackedDib, pPackedDib, size);
        AssignReferenceVariables();
}

/////////////////////////////////////////////////////////////////
// Information Functions
DWORD CDib::PaletteSize()
{
        switch (BitCount()) {
          case 1 :
            return sizeof(PALETTEENTRY)*2;
          case 4 :
            return sizeof(PALETTEENTRY)*16;
          case 8 :
            return sizeof(PALETTEENTRY)*256;
          default :
            return 0;
        }
}

DWORD CDib::NumberOfColors()
{
        switch (BitCount()) {
          case 1 :
            return 2;
          case 4 :
            return 16;
          case 8 :
            return 256;
          case 16 :
            return 0x00010000;
          case 24 :
          case 32 :
            return 0x01000000;
          default :
            return 0;
        }
}

unsigned char * CDib::GetPointer(int x, int y)
{
        if ( (x<0) || (x>=Width()) || (y<0) || (y>=Height()) ) return NULL;

        unsigned char *pLine = m_pPattern + (Height()-1-y)*ByteWidth();

        switch (BitCount()) {
          case 1 :
            return pLine + x/8;
          case 4 :
            return pLine + x/2;
          case 8 :
            return pLine + x;
          case 16 :
            return pLine + x*2;
          case 24 :
            return pLine + x*3;
          case 32 :
            return pLine + x*4;
          default :
            return NULL;
        }
}

// convert RGB color model to YIQ color model
unsigned char CDib::Intensity(int x, int y)
{
        unsigned char value = *GetPointer(x, y);
        unsigned char intensity=0;

        switch (BitCount()) {
          case 1 :
            if (value & m_BitMask[(x%8)]) value = 1;
            else value = 0;
            intensity = GetBrightness(m_pRgbQuad[value]);
            break;
          case 4 :
            if ((x%2)==0) value >>= 4;
            else value &= 0x0F;
            intensity = GetBrightness(m_pRgbQuad[value]);
            break;
          case 8 :
            intensity = GetBrightness(m_pRgbQuad[value]);
            break;
          case 16 :
            {
              WORD *pPtr = (WORD *) GetPointer(x, y);
              RGBQUAD rgb;
              
              rgb = Decode16(*pPtr);
              intensity = GetBrightness(rgb);
            }
            break;
          case 24 :
          case 32 :
            {
              unsigned char *pPtr = GetPointer(x, y);
              RGBQUAD rgb;
              rgb.rgbBlue = *pPtr++;
              rgb.rgbGreen = *pPtr++;
              rgb.rgbRed = *pPtr++;
              rgb.rgbReserved = 0;
              intensity = GetBrightness(rgb);
            }
            break;
          default :
            break;
        }

        return intensity;
}

unsigned char CDib::GetIndex(int x, int y)
{
        unsigned char value = *GetPointer(x, y);

        switch (BitCount()) {
          case 1 :
            return (m_BitMask[x%8] & value) ? 1 : 0;
          case 4 :
            return ((x%2)==0) ? (value>>4) : (value & 0x0F);
          case 8 :
            return value;
          case 16 :
          case 24 :
          case 32 :
          default :
            return 0;
        }
}

RGBQUAD CDib::GetColor(int x, int y)
{
        switch (BitCount()) {
          case 1 :
          case 4 :
          case 8 :
            return m_pRgbQuad[GetIndex(x,y)];
          case 16 :
            {
              WORD *pPtr = (WORD *) GetPointer(x, y);
              RGBQUAD quad;
              
              quad = Decode16(*pPtr);

              return quad;  
            }
          case 24 :
          case 32 :
            {
              unsigned char *pPtr = GetPointer(x, y);
              RGBQUAD quad;

              quad.rgbBlue = *pPtr++;
              quad.rgbGreen = *pPtr++;
              quad.rgbRed = *pPtr++;
              quad.rgbReserved = 0;

              return quad;
            }
          default :
            {
              RGBQUAD quad;
              memset(&quad, 0, sizeof(RGBQUAD));

              return quad;
            }
        }
}

RGBQUAD CDib::Decode16(WORD data)
{                         
        RGBQUAD quad;
        quad.rgbRed = (unsigned char) ((data>>7) & 0x00F8);
        quad.rgbGreen = (unsigned char) ((data>>2) & 0x00F8);
        quad.rgbBlue = (unsigned char) ((data<<3) & 0x00F8);
        quad.rgbReserved = 0;

        return quad;
}

WORD CDib::Encode16(unsigned char red, unsigned char green, unsigned char blue)
{
        //   0rrr rrgg gggb bbbb
        WORD data = ((((WORD) (red & 0xF8)) << 7) & 0x7C00) |
                    ((((WORD) (green & 0xF8)) << 2) & 0x03E0) |
                    ((((WORD) (blue & 0xF8)) >> 3) & 0x001F);

        return data;
}

/////////////////////////////////////////////////////////////////
// Data Operations
void CDib::UpsideDown()
{
        if (!IsInitialized()) return;

        unsigned char *pBuffer = new unsigned char [ByteWidth()];
        unsigned char *pUp, *pDown;

        for (int i=0; i<Height()/2 ;i++) {
          pUp = GetPointer(0, i);
          pDown = GetPointer(0, Height()-i-1);
          memcpy(pBuffer, pUp, ByteWidth());
          memcpy(pUp, pDown, ByteWidth());
          memcpy(pDown, pBuffer, ByteWidth());
        }

        delete [] pBuffer;
}

void CDib::Inverse()
{
    for (int i=0; i<Height() ;i++) {
      unsigned char *ptr = GetPointer(0, i);
      for (int j=0; j<ByteWidth() ;j++, ptr++)
        *ptr = 255 - *ptr;
    }
}

void CDib::ResetContents()
{
        if (!IsInitialized()) return;

        memset(m_pPattern, 0, ByteWidth()*Height());
}

BOOL CDib::CopyRect(RECT *pTargetRect, CDib *pSrCDib, RECT *pSrcRect, int BackColor)
{
	int tWidth = pTargetRect->right - pTargetRect->left;
	int tHeight = pTargetRect->bottom - pTargetRect->top;
	int sWidth = pSrcRect->right - pSrcRect->left;
	int sHeight = pSrcRect->bottom - pSrcRect->top;

    if (pSrCDib->BitCount()!=BitCount()) {
      AfxMessageBox("??? Can't Copy Pattern with Different BitCount\n");
      return false;
    }

    if ((tWidth!=sWidth) || (tHeight!=sHeight)) {
      AfxMessageBox("??? Can't Copy Rect Region with Different Size\n");
      return false;
    }

    unsigned char *pTarget, *pSrc;

    if ((BackColor==-1) || (BitCount()!=8)) {       // transparent copy is allowed only for 8 bit bitmap
      for (int i=0; i<tHeight ;i++) {
        pTarget = GetPointer(pTargetRect->left, pTargetRect->top+i);
        pSrc = pSrCDib->GetPointer(pSrcRect->left, pSrcRect->top+i);
        memcpy(pTarget, pSrc, sWidth*BitCount()/8);
      }
    }
    else {
      for (int i=0; i<tHeight ;i++) {
        pTarget = GetPointer(pTargetRect->left, pTargetRect->top+i);
        pSrc = pSrCDib->GetPointer(pSrcRect->left, pSrcRect->top+i);
        for (int j=0; j<sWidth ;j++, pTarget++, pSrc++)
          if (*pSrc!=BackColor) *pTarget = *pSrc;
      }
    }

    return true;
}

/////////////////////////////////////////////////////////////////
// Create New CDibs
CDib *CDib::GetGrayCDib()
{
        CDib *pDib = new CDib;

        pDib->Allocate(Width(), Height(), 8);
        pDib->SetGrayPalette();

        for (int y=0; y<Height() ;y++) {
          unsigned char *pNew = pDib->GetPointer(0, y);
          for (int x=0; x<Width() ;x++, pNew++)
            *pNew = Intensity(x, y);
        }

        return pDib;
}

CDib *CDib::GetGrayCDibFast()
{
	CDib *pRes = new CDib;
	pRes->Allocate(Width(), Height(), 8);
	pRes->SetGrayPalette();
	
	int value;
	for (int i=0; i<Height() ;i++) {
		unsigned char *pD = GetPointer(0, i);
		unsigned char *pR = pRes->GetPointer(0, i);
		for (int j=0; j<Width() ;j++) {
			value = (int) *pD++ + (int) *pD++ + (int) *pD++;
			*pR++ = (unsigned char) (value/3);
		}
	}
	
	return pRes;
/*
	if (IsGrayImage()) return (CDib *) CopyCDib();

	CDib *pDib = new CDib;

    pDib->Allocate(Width(), Height(), 8);
    pDib->SetGrayPalette();

	int x, y;
	switch (BitCount()) {
		case 24:
			for (y=0; y<Height() ;y++) {
				unsigned char *pOrg = GetPointer(0, y);
				unsigned char *pNew = pDib->GetPointer(0, y);
				for (x=0; x<Width() ;x++, pOrg+=3)
					*pNew++ = GetIntensityFast(*(pOrg+2), *(pOrg+1), *pOrg);
			}
			break;
		case 32:
			for (y=0; y<Height() ;y++) {
				unsigned char *pOrg = GetPointer(0, y);
				unsigned char *pNew = pDib->GetPointer(0, y);
				for (x=0; x<Width() ;x++, pOrg+=4)
					*pNew++ = GetIntensityFast(*(pOrg+2), *(pOrg+1), *pOrg);
			}
			break;
		default:
			delete pDib;
			pDib = NULL;
			break;
	}

    return pDib;
*/
}

CDib *CDib::CopyCDib()
{
        CDib *pDib = new CDib;

        pDib->Allocate(Width(), Height(), BitCount());
        
        DWORD size = CalcPackedDibSize(m_pInfoHeader);
        memcpy(pDib->GetPackedDibPointer(), GetPackedDibPointer(), size);

        return pDib;
}

CDib *CDib::GetSizedCDib(int width, int height)
{
        CDib *pDib = new CDib;

        if (height==0) {        // set new height so as the aspect ratio is preserved
          height = width * Height() / Width();
        }

        pDib->Allocate(width, height, 24);

        double fx = (double) Width() / (double) width;
        double fy = (double) Height() / (double) height;
        int ix, iy;
        RGBQUAD quad;

        for (int y=0; y<height ;y++) {
          unsigned char *pPtr = pDib->GetPointer(0, y);
          for (int x=0; x<width ;x++) {
            ix = (int) (fx * x + 0.5); ix = min(ix, Width()-1);
            iy = (int) (fy * y + 0.5); iy = min(iy, Height()-1);
            quad = GetColor(ix, iy);
            *pPtr++ = quad.rgbBlue;
            *pPtr++ = quad.rgbGreen;
            *pPtr++ = quad.rgbRed;
          }
        }

        return pDib;
}

CDib *CDib::GetEdgeCDib(double threshold, BOOL bIsGray)
{
        CDib *pGray = bIsGray ? this : GetGrayCDib();
        CDib *pDib = new CDib;
        int h, v;
        double magnitude;
         
        pDib->Allocate(Width(), Height(), 8);
        pDib->SetGrayPalette();

        for (int y=1; y<Height()-1 ;y++) {
          unsigned char *pUp = pGray->GetPointer(1, y-1);
          unsigned char *pNow = pGray->GetPointer(1, y);
          unsigned char *pDown = pGray->GetPointer(1, y+1);
          unsigned char *pNew = pDib->GetPointer(1, y);
          for (int x=1; x<Width()-1 ;x++, pUp++, pNow++, pDown++, pNew++) {
            v = *(pUp-1) + *(pNow-1) + *(pDown-1)
              - *(pUp+1) - *(pNow+1) - *(pDown+1);
            h = *(pUp-1) + *pUp + *(pUp+1)
              - *(pDown-1) - *pDown - *(pDown+1);
            magnitude = sqrt((double) v * (double) v + (double) h * (double) h);
            if (magnitude>threshold) *pNew = 255;
            else *pNew = 0;
          }
        }

        if (!bIsGray) delete pGray;

        return pDib;
}

CDib *CDib::ClipCDib(RECT *pRect)
{
	return ClipCDib(pRect->left, pRect->top, pRect->right, pRect->bottom);
}

CDib *CDib::ClipCDib(int left, int top, int right, int bottom)
{
	RECT target;
	target.left = max(left, 0);
	target.top = max(top, 0);
	target.right = min(right, (int) Width());
	target.bottom = min(bottom, (int) Height());
	int tWidth = target.right - target.left;
	int tHeight = target.bottom - target.top;

	CDib *pRes = new CDib;
	pRes->Allocate(tWidth, tHeight, BitCount());

	if (PaletteSize()>0) memcpy(pRes->GetRgbQuadPointer(), m_pRgbQuad, PaletteSize());

	int bwidth = tWidth * BitCount() / 8;
	for (int i=0; i<tHeight ;i++) 
	  memcpy(pRes->GetPointer(0, i), GetPointer(target.left, target.top+i), bwidth);


    return pRes;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Additional Functions
void CDib::SetGrayPalette()
{
	RGBQUAD *p = m_pRgbQuad;
	int i;
	switch (BitCount()) {
	  case 1:
	    p->rgbRed = p->rgbGreen = p->rgbBlue = p->rgbReserved = 0;
		p++;
		p->rgbRed = p->rgbGreen = p->rgbBlue = 255; p->rgbReserved = 0;
		break;
	  case 4:
	    for (i=0; i<16 ;i++, p++) {
		  p->rgbRed = p->rgbGreen = p->rgbBlue = i * 16;
		  p->rgbReserved = 0;
		}
		break;
	  case 8:
		for (i=0; i<256 ;i++, p++) {
		  p->rgbRed = p->rgbGreen = p->rgbBlue = i;
		  p->rgbReserved = 0;
		}
		break;
	  default:
	    return;
	}
}

BOOL CDib::IsGrayImage()
{
        if (8!=BitCount()) return false;

        RGBQUAD *pQuad = m_pRgbQuad;
        for (int i=0; i<256 ;i++, pQuad++)
          if ((pQuad->rgbRed!=i) || (pQuad->rgbGreen!=i) || (pQuad->rgbBlue!=i)) return false;

        return true;
}

void CDib::SplitChannel(CDib **pDibR, CDib **pDibG, CDib **pDibB)
{
        (*pDibR) = new CDib;
        (*pDibR)->Allocate(Width(), Height(), 8);
        (*pDibR)->SetGrayPalette();
        (*pDibG) = new CDib;
        (*pDibG)->Allocate(Width(), Height(), 8);
        (*pDibG)->SetGrayPalette();
        (*pDibB) = new CDib;
        (*pDibB)->Allocate(Width(), Height(), 8);
        (*pDibB)->SetGrayPalette();

        int nBit = BitCount();
        switch (nBit) {
          case 24 :
          case 32 :
            {
              for (int i=0; i<Height() ;i++) {
                unsigned char *pSrc = GetPointer(0, i);
                unsigned char *pR = (*pDibR)->GetPointer(0, i);
                unsigned char *pG = (*pDibG)->GetPointer(0, i);
                unsigned char *pB = (*pDibB)->GetPointer(0, i);
                for (int j=0; j<Width() ;j++) {
                  *pB++ = *pSrc++;
                  *pG++ = *pSrc++;
                  *pR++ = *pSrc++;
                  if (nBit==32) pSrc++;
                }
              }
            }
            return;
          case 16 :
            {
              RGBQUAD quad;  
              for (int i=0; i<Height() ;i++) {
                unsigned char *pSrc = GetPointer(0, i);
                unsigned char *pR = (*pDibR)->GetPointer(0, i);
                unsigned char *pG = (*pDibG)->GetPointer(0, i);
                unsigned char *pB = (*pDibB)->GetPointer(0, i);
                for (int j=0; j<Width() ;j++, pSrc+=2) {
                  quad = Decode16(*((WORD *) pSrc));
                  *pR++ = quad.rgbRed;
                  *pG++ = quad.rgbGreen;
                  *pB++ = quad.rgbBlue;
                }
              }
            }            
            return;
          case 8 :
            {
              for (int i=0; i<Height() ;i++) {
                unsigned char *pSrc = GetPointer(0, i);
                unsigned char *pR = (*pDibR)->GetPointer(0, i);
                unsigned char *pG = (*pDibG)->GetPointer(0, i);
                unsigned char *pB = (*pDibB)->GetPointer(0, i);
                for (int j=0; j<Width() ;j++, pSrc++) {
                  *pR++ = m_pRgbQuad[*pSrc].rgbRed;
                  *pG++ = m_pRgbQuad[*pSrc].rgbGreen;
                  *pB++ = m_pRgbQuad[*pSrc].rgbBlue;
                }
              }          
            }
            return;    
          case 4 :
          case 1 :
            {
              RGBQUAD quad;
              for (int i=0; i<Height() ;i++) {
                unsigned char *pR = (*pDibR)->GetPointer(0, i);
                unsigned char *pG = (*pDibG)->GetPointer(0, i);
                unsigned char *pB = (*pDibB)->GetPointer(0, i);
                for (int j=0; j<Width() ;j++) {
                  quad = GetColor(j, i);
                  *pR++ = quad.rgbRed;
                  *pG++ = quad.rgbGreen;
                  *pB++ = quad.rgbBlue;
                }
              }          
            }
            return;    
          default :
#ifdef _DEBUG
            AfxMessageBox("??? Unknown BitCount in SplitChannel");
#endif
            return;
        }
}

CDib *CDib::MergeChannel(CDib *pDibR, CDib *pDibG, CDib *pDibB, int bitcount)
{
        if (bitcount<16) {
#ifdef _DEBUG
          AfxMessageBox("??? Bitcount for the result CDib of MergeChannel should be larger or equal to 16");
#endif
          return NULL;
        }

        CDib *pDib = new CDib;
        pDib->Allocate(pDibR->Width(), pDibR->Height(), bitcount);

        for (int i=0; i<pDib->Height() ;i++) {
          unsigned char *pTarget = pDib->GetPointer(0, i);
          unsigned char *pR = pDibR->GetPointer(0, i);
          unsigned char *pG = pDibG->GetPointer(0, i);
          unsigned char *pB = pDibB->GetPointer(0, i);
          for (int j=0; j<pDib->Width() ;j++) {
            switch (bitcount) {
              case 24 :
                *pTarget++ = *pB++;
                *pTarget++ = *pG++;
                *pTarget++ = *pR++;
                break;
              case 32 :
                *pTarget++ = *pB++;
                *pTarget++ = *pG++;
                *pTarget++ = *pR++;
                *pTarget++ = 0;
                break;
              case 16 :
                *((WORD *) pTarget) = Encode16(*pR++, *pG++, *pB++);
                pTarget += 2;
                break;
              default :
#ifdef _DEBUG
                AfxMessageBox("??? Unknown Bitcount in MergeChannel");
#endif
                break;
            }
          }
        }

        return pDib;
}


#ifdef _WINDOWS
/////////////////////////////////////////////////////////////////
// Clipboard Functions
BOOL CDib::GetClipboard()
{
        if (!IsClipboardFormatAvailable(CF_DIB)) return false;
        
        OpenClipboard(NULL);
        HANDLE handle = GetClipboardData(CF_DIB);

        if (!handle) {
          CloseClipboard();
          return false;
        }

        unsigned char *ptr = (unsigned char *) GlobalLock(handle);
        BITMAPINFOHEADER *pHeader = (BITMAPINFOHEADER *) ptr;

        Allocate(pHeader->biWidth, pHeader->biHeight, pHeader->biBitCount);
        memcpy(GetPackedDibPointer(), ptr, CalcPackedDibSize(pHeader));
         
        GlobalUnlock(handle);

        CloseClipboard();

        return true;
}

BOOL CDib::SetClipboard()
{
        if (!IsInitialized()) return false;

        HANDLE handle = GlobalAlloc(GMEM_MOVEABLE | GMEM_DDESHARE, CalcPackedDibSize(GetBitmapInfoHeader()));
        if (!handle) return false;
        unsigned char *ptr = (unsigned char *) GlobalLock(handle);
        memcpy(ptr, GetPackedDibPointer(), CalcPackedDibSize(GetBitmapInfoHeader()));
        GlobalUnlock(handle);

        OpenClipboard(NULL);
        SetClipboardData(CF_DIB, handle);
        CloseClipboard();

        return true;
}

CBitmap *CDib::GetCBitmap(CDC *pDC)
{
        int byte_width = (int) ByteWidth(((long) m_pInfoHeader->biWidth)*((long) m_pInfoHeader->biBitCount));
        long image_size = ((long) byte_width) * ((long) m_pInfoHeader->biHeight);
        HDC hDC;

        if (pDC) hDC = pDC->GetSafeHdc();
        else hDC = ::GetDC(NULL);

        CBitmap *pBitmap = new CBitmap;
        BITMAPINFO *pBInfo = (BITMAPINFO *) GetBitmapInfo();
        HBITMAP hBitmap = ::CreateDIBitmap(hDC, m_pInfoHeader, CBM_INIT,
                                GetPattern(), pBInfo, DIB_RGB_COLORS); //DIB_PAL_COLORS);
        pBitmap->Attach(hBitmap);
        //pBitmap->CreateBitmap(Width(), Height(), 1, BitCount(), GetPattern());

        return pBitmap;
}                

void CDib::GetPaletteEntries(UINT nStartIndex, UINT nNumEntries, PALETTEENTRY *pEntries)
{
        int num_color = NumberOfColors();
        num_color = (num_color<=256) ? num_color : 0;
        if (nNumEntries>(UINT) num_color) {
#ifdef _DEBUG
          AfxMessageBox("??? This CDib does not have so many colors");
#endif
          return;
        }
        
        RgbQuadToPaletteEntry(m_pRgbQuad+nStartIndex, pEntries, nNumEntries);
}

void CDib::SetPaletteEntries(UINT nStartIndex, UINT nNumEntries, PALETTEENTRY *pEntries)
{
        int num_color = NumberOfColors();
        num_color = (num_color<=256) ? num_color : 0;
        if (nNumEntries>(UINT) num_color) {
#ifdef _DEBUG
          AfxMessageBox("??? This CDib does not have so many colors");
#endif
          return;
        }

        PaletteEntryToRgbQuad(pEntries, m_pRgbQuad+nStartIndex, nNumEntries);
}                                

/////////////////////////////////////////////////////////////////
// GDI Functions
void CDib::PaletteEntryToRgbQuad(PALETTEENTRY *pEntries, RGBQUAD *pRgbQuad, int count)
{
        for (int i=0; i<count ;i++, pEntries++, pRgbQuad++) {
          pRgbQuad->rgbRed = pEntries->peRed;
          pRgbQuad->rgbGreen = pEntries->peGreen;
          pRgbQuad->rgbBlue = pEntries->peBlue;
          pRgbQuad->rgbReserved = 0;
        }
}


void CDib::RgbQuadToPaletteEntry(RGBQUAD *pRgbQuad, PALETTEENTRY *pEntries, int count)
{
        for (int i=0; i<count ;i++, pEntries++, pRgbQuad++) {
          pEntries->peRed = pRgbQuad->rgbRed;
          pEntries->peGreen = pRgbQuad->rgbGreen;
          pEntries->peBlue = pRgbQuad->rgbBlue;
          pEntries->peFlags = 0;
        }
}

CPalette *CDib::GetPaletteNoFlag()
{
        if (!IsInitialized() || !HavePalette()) return NULL;

        unsigned char *pBuffer = new unsigned char [sizeof(LOGPALETTE) + sizeof(PALETTEENTRY)*256];
        LOGPALETTE *pLogPalette = (LOGPALETTE *) pBuffer;
        pLogPalette->palVersion = 0x300;
        pLogPalette->palNumEntries = (unsigned short) NumberOfColors();
        RgbQuadToPaletteEntry(m_pRgbQuad, pLogPalette->palPalEntry, NumberOfColors());

        CPalette *pPalette = new CPalette;
        pPalette->CreatePalette(pLogPalette);

        delete [] pBuffer;

        return pPalette;
}

void CDib::LoadImage(CDC *pDC, CRect *pTargetRect, CRect *pSrcRect, DWORD dwRop)
{
        if (!IsInitialized()) return;

        CRect draw_rect;
        if (!pTargetRect) draw_rect.SetRect(0, 0, Width(), Height());
        else draw_rect = *pTargetRect;

        CRect src_rect;
        if (!pSrcRect) src_rect.SetRect(0, 0, Width(), Height());
        else src_rect = *pSrcRect;

        int sy = max(0, (Height()-draw_rect.Height()));
        if (BitCount()>8) {   // Full Color Image
          StretchDIBits(pDC->GetSafeHdc(), draw_rect.left, draw_rect.top, draw_rect.Width(), draw_rect.Height(),
                        src_rect.left, src_rect.top, src_rect.Width(), src_rect.Height(),
                        m_pPattern, GetBitmapInfo(), DIB_RGB_COLORS, dwRop);
        }
        else {          // Palette Image
          BITMAPINFO *pInfo = GetCopyOfBitmapInfo();
          WORD *pIndex = (WORD *) pInfo->bmiColors;
          for (int i=0; i<(int) NumberOfColors() ;i++) pIndex[i] = i;

          StretchDIBits(pDC->GetSafeHdc(), draw_rect.left, draw_rect.top, draw_rect.Width(), draw_rect.Height(),
                        src_rect.left, src_rect.top, src_rect.Width(), src_rect.Height(),
                        m_pPattern, pInfo, DIB_PAL_COLORS, dwRop);

          delete pInfo;
        }
}

void CDib::LoadPaletteImage(CDC *pDC, CRect *pTargetRect, CRect *pSrcRect, DWORD dwRop)
{
        if (!IsInitialized()) return;

        CRect draw_rect;
        if (!pTargetRect) draw_rect.SetRect(0, 0, Width(), Height());
        else draw_rect = *pTargetRect;

        CRect src_rect;
        if (!pSrcRect) src_rect.SetRect(0, 0, Width(), Height());
        else src_rect = *pSrcRect;

        int sy = max(0, (Height()-draw_rect.Height()));
        if (BitCount()>8) {   // Full Color Image
          StretchDIBits(pDC->GetSafeHdc(), draw_rect.left, draw_rect.top, draw_rect.Width(), draw_rect.Height(),
                        src_rect.left, src_rect.top, src_rect.Width(), src_rect.Height(),
                        m_pPattern, GetBitmapInfo(), DIB_RGB_COLORS, dwRop);
        }
        else {          // Palette Image
          BITMAPINFO *pInfo = GetCopyOfBitmapInfo();
          WORD *pIndex = (WORD *) pInfo->bmiColors;
          for (int i=0; i<(int) NumberOfColors() ;i++) pIndex[i] = i;

          CPalette *pPalette = GetPaletteNoFlag();
          CPalette *pOldPalette = pDC->SelectPalette(pPalette, false);
          pDC->RealizePalette();

          StretchDIBits(pDC->GetSafeHdc(), draw_rect.left, draw_rect.top, draw_rect.Width(), draw_rect.Height(),
                        src_rect.left, src_rect.top, src_rect.Width(), src_rect.Height(),
                        m_pPattern, pInfo, DIB_PAL_COLORS, dwRop);

          pDC->SelectPalette(pOldPalette, false);
          delete pPalette;

          delete pInfo;
        }
}

/////////////////////////////////////////////////////////////////
// Serialization
void CDib::Serialize(CArchive& ar)
{
        if (ar.IsStoring()) {
          if (!IsInitialized()) return;

          // write bitmap file header
          BITMAPFILEHEADER bfHeader;
          bfHeader.bfType = TYPE_BITMAP;
          bfHeader.bfOffBits = (DWORD) sizeof(BITMAPFILEHEADER)
                             + (DWORD) sizeof(BITMAPINFOHEADER)
                             + (DWORD) PaletteSize();
          bfHeader.bfSize = bfHeader.bfOffBits + ByteWidth()*Height();
          bfHeader.bfReserved1 = bfHeader.bfReserved2 = 0;

          ar.Write(&bfHeader, sizeof(BITMAPFILEHEADER));

          // write bitmap info header
          ar.Write(m_pInfoHeader, sizeof(BITMAPINFOHEADER));

          // write palette
          if (PaletteSize()>0) 
            ar.Write(m_pRgbQuad, PaletteSize());

          // write image data
          ar.Write(m_pPattern, ByteWidth()*Height());
        }
        else {
          if (IsInitialized()) DestroyContents();

          // read in BITMAPFILEHEADER
          BITMAPFILEHEADER bfHeader;
          ar.Read(&bfHeader, sizeof(BITMAPFILEHEADER));
          if (bfHeader.bfType!=TYPE_BITMAP) {
            TRACE0("??? This File is NOT a BMP File");
            return;
          }
          
          // read in BITMAPINFOHEADER
          BITMAPINFOHEADER biHeader;
          ar.Read(&biHeader, sizeof(BITMAPINFOHEADER));

          int p_size; 
          switch (biHeader.biBitCount) {
            case 1 : p_size = 2 * sizeof(PALETTEENTRY); break;
            case 4 : p_size = 16 * sizeof(PALETTEENTRY); break;
            case 8 : p_size = 256 * sizeof(PALETTEENTRY); break;
            default : p_size = 0;
          }
          int skip = bfHeader.bfOffBits - sizeof(BITMAPFILEHEADER) - sizeof(BITMAPINFOHEADER) - p_size;
          if (skip>0) {
            char *pTemp = new char [skip];
            ar.Read(pTemp, skip);
            delete [] pTemp;
          }

          DWORD size = CalcPackedDibSize(&biHeader);

          m_pPackedDib = new unsigned char[size];
          memcpy(m_pPackedDib, &biHeader, sizeof(BITMAPINFOHEADER));

          ar.Read(m_pPackedDib+sizeof(BITMAPINFOHEADER), size-sizeof(BITMAPINFOHEADER));

          AssignReferenceVariables();
        }
}
#endif

#ifdef __VERSION_2013

//////////////////////////////////////////////////////////////////////////////////////
//
// Drawing Functions for CDib
//
//////////////////////////////////////////////////////////////////////////////////////
//
// Add Points (cross)
void CDib::AddPoints(FPointArray *pPoints, COLORREF color)
{	
	if (IsGrayImage()) {
      AddPoints(pPoints, GetIntensity(color));
      return;
	}

	int x, y;
	unsigned char *ptr;
	unsigned char r=GetRValue(color), g=GetGValue(color), b=GetBValue(color);
	for (int i=0; i<pPoints->GetSize() ;i++) {
	  x = (int) pPoints->GetAt(i).x;
	  y = (int) pPoints->GetAt(i).y;
	  if ((x<2) || (x>=(Width()-2)) ||
	      (y<2) || (y>=(Height()-2))) continue;
	  ptr = GetPointer(x-2, y);
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  ptr = GetPointer(x, y+2);
	  *ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
	  *ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
											ptr += ByteWidth();
	  *ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
	  *ptr = b; *(ptr+1) = g; *(ptr+2) = r;
	}
}

void CDib::AddPoints(CPointArray *pPoints, COLORREF color)
{	
	if (IsGrayImage()) {
      AddPoints(pPoints, GetIntensity(color));
      return;
	}

	int x, y;
	unsigned char *ptr;
	unsigned char r=GetRValue(color), g=GetGValue(color), b=GetBValue(color);
	for (int i=0; i<pPoints->GetSize() ;i++) {
	  x = pPoints->GetAt(i).x;
	  y = pPoints->GetAt(i).y;
	  if ((x<2) || (x>=(Width()-2)) ||
	      (y<2) || (y>=(Height()-2))) continue;
	  ptr = GetPointer(x-2, y);
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  *ptr++ = b; *ptr++ = g; *ptr++ = r;
	  ptr = GetPointer(x, y+2);
	  *ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
	  *ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
											ptr += ByteWidth();
	  *ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
	  *ptr = b; *(ptr+1) = g; *(ptr+2) = r;
	}
}

// Add Rects	  		
void CDib::AddRects(RectArray *pRects, COLORREF color)
{	
    // if this is a gray image, call gray version of AddRectsToCDib
    if (IsGrayImage()) {
      AddRects(pRects, GetIntensity(color));
      return;
    }

	unsigned char r=GetRValue(color), g=GetGValue(color), b=GetBValue(color);
	unsigned char *ptr1, *ptr2;
	CRect rect;
    CRect canvas(0, 0, Width()-1, Height()-1);
	int i, j;
	for (i=0; i<pRects->GetSize() ;i++) {
	  rect = pRects->GetAt(i);
      rect &= canvas;
	  ptr1 = GetPointer(rect.left, rect.top);
	  ptr2 = GetPointer(rect.left, rect.bottom);
	  for (j=0; j<(rect.right-rect.left) ;j++) {
	    *ptr1++ = b; *ptr1++ = g; *ptr1++ = r;	
	    *ptr2++ = b; *ptr2++ = g; *ptr2++ = r;	
	  }
	  ptr1 = GetPointer(rect.left, rect.bottom);
	  ptr2 = GetPointer(rect.right, rect.bottom);
	  for (j=0; j<(rect.bottom-rect.top) ;j++) {
	    *ptr1 = b; *(ptr1+1) = g; *(ptr1+2) = r; ptr1 += ByteWidth();
	    *ptr2 = b; *(ptr2+1) = g; *(ptr2+2) = r; ptr2 += ByteWidth();
	  }
	}
}

// Draw Dot to CDib
void CDib::DrawDot(int x, int y, COLORREF color)
{
	if ((x<0) || (x>=Width()) || (y<0) || (y>=Height())) return;

    if (IsGrayImage()) DrawDot(x, y, GetIntensity(color));
    else {
	  unsigned char *ptr = GetPointer(x, y);
	  *ptr++ = GetBValue(color);
	  *ptr++ = GetGValue(color);
	  *ptr++ = GetRValue(color);
    }
}

void CDib::DrawCross(int x, int y, COLORREF color)
{
	if ((x<2) || (x>=(Width()-2)) || (y<2) || (y>=(Height()-2))) return;

	unsigned char r=GetRValue(color), g=GetGValue(color), b=GetBValue(color);
	unsigned char *ptr = GetPointer(x-2, y);
	*ptr++ = b; *ptr++ = g; *ptr++ = r;
	*ptr++ = b; *ptr++ = g; *ptr++ = r;
	*ptr++ = b; *ptr++ = g; *ptr++ = r;
	*ptr++ = b; *ptr++ = g; *ptr++ = r;
	*ptr++ = b; *ptr++ = g; *ptr++ = r;
	ptr = GetPointer(x, y+2);
	*ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
	*ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
										  ptr += ByteWidth();
	*ptr = b; *(ptr+1) = g; *(ptr+2) = r; ptr += ByteWidth();
	*ptr = b; *(ptr+1) = g; *(ptr+2) = r;
}

// Draw Line to CDib
void CDib::DrawLine(int x1, int y1, int x2, int y2, COLORREF color)
{
    // if this is a gray image, call gray version of DrawLineToCDib
    if (IsGrayImage()) {
      DrawLine(x1, y1, x2, y2, GetIntensity(color));
      return;
    }

	int dx, dy, s, step;

	dx = abs(x2-x1); dy = abs(y2-y1);
	if (dx>dy) {
	  if (x1>x2) {
	    step = (y1>y2) ? 1 : -1;
		s = x1; x1 = x2; x2 = s; y1 = y2;
	  }
	  else step = (y1<y2) ? 1 : -1;
	  DrawDot(x1, y1, color);
	  s = dx >> 1;
	  while (++x1 <= x2) {
	    if ((s-=dy) < 0) {
		  s += dx; y1 += step;
		}
		DrawDot(x1, y1, color);
	  }
	}
	else {
	  if (y1>y2) {
	    step = (x1>x2) ? 1 : -1;
		s = y1; y1 = y2; y2 = s; x1 = x2;
	  }
	  else step = (x1<x2) ? 1 : -1;
	  DrawDot(x1, y1, color);
	  s = dy >> 1;
	  while (++y1 <= y2) {
	    if ((s-=dx)<0) {
		  s += dy; x1 += step;
		}
		DrawDot(x1, y1, color);
	  }
	}
}

// Draw Hough Lines
void CDib::DrawHLines(HLineArray *pLines, COLORREF color)
{
    // if this is a gray image, call the gray version of DrawHLines
    if (IsGrayImage()) {
      DrawHLines(pLines, GetIntensity(color));
      return;
    }

	POINT pt1, pt2;
	for (int i=0; i<pLines->GetSize() ;i++) {
	  HLine line = pLines->GetAt(i);
	  double a=cos(line.theta), b=sin(line.theta);
	  if (fabs(a)<0.001) {
	    pt1.y = pt2.y = (int) (line.rho+0.5);
		pt1.x = 0;
		pt2.x = Width() - 1;
	  }
	  else if (fabs(b)<0.001) {
	    pt1.x = pt2.x = (int) (line.rho+0.5);
		pt1.y = 0;
		pt2.y = Height() - 1;
	  }
	  else {
        if (line.rho>0.) {
	      pt1.x = 0;
		  pt1.y = (int) (line.rho/b + 0.5);
        }
        else {
          pt1.x = Width()-1;
          pt1.y = (int) ((line.rho-pt1.x*a)/b+0.5);
        }
	    pt2.x = (int) (line.rho/a + 0.5);
		pt2.y = 0;
	  }
	  DrawLine(pt1.x, pt1.y, pt2.x, pt2.y, color);
	}
}

// draw circle 
void CDib::DrawCircle(int xc, int yc, int radius, COLORREF color)
{
    if (IsGrayImage()) {
      DrawCircle(xc, yc, radius, GetIntensity(color));
      return;
    }

	int dx, dy;

	dx = radius; dy = 0;

	while(dx >= dy){
		DrawDot(xc+dx, yc+dy, color);
		DrawDot(xc+dx, yc-dy, color);
		DrawDot(xc-dx, yc+dy, color);
		DrawDot(xc-dx, yc-dy, color);
		DrawDot(xc+dy, yc+dx, color);
		DrawDot(xc+dy, yc-dx, color);
		DrawDot(xc-dy, yc+dx, color);
		DrawDot(xc-dy, yc-dx, color);
		if( (radius -= (dy++ << 1) -1) < 0 )
			radius += ( dx-- -1) << 1;
	}
}

// draw an ellipse
void CDib::DrawEllipse(int xc, int yc, int rx, int ry, COLORREF color)
{
    if (IsGrayImage()) {
      DrawEllipse(xc, yc, rx, ry, GetIntensity(color));
      return;
    }

	int x, x1, y, y1, r;

	if(rx > ry) {
		x = r = rx; y = 0;
		while(x >= y){
			x1 = (int) ((long)x *ry/rx);
			y1 = (int) ((long) y*ry /rx);
			DrawDot(xc + x, yc+y1, color);
			DrawDot(xc + x, yc-y1, color);
			DrawDot(xc - x, yc + y1, color);
			DrawDot(xc - x, yc - y1, color);
			DrawDot(xc - x, yc + y1, color);
			DrawDot(xc + y, yc + x1, color);
			DrawDot(xc + y, yc - x1, color);
			DrawDot(xc - y, yc + x1, color);
			DrawDot(xc - y, yc - x1, color);
			if (( r -= (y++ << 1) - 1 ) < 0 )
				r += (x-- -1) << 1;
		}
	}else{
		x = r = ry; y = 0;
		while( x >= y){
			x1 = (int) ((long) x*rx/ry);
			y1 = (int)((long) y*rx/ry);
			DrawDot(xc + x1, yc + y, color);
			DrawDot(xc + x1, yc - y, color);
			DrawDot(xc - x1, yc + y, color);
			DrawDot(xc - x1, yc - y, color);
			DrawDot(xc - x1, yc + y, color);
			DrawDot(xc + y1, yc + x, color);
			DrawDot(xc + y1, yc - x, color);
			DrawDot(xc - y1, yc + x, color);
			DrawDot(xc - y1, yc - x, color);
			if ((r -= (y++ << 1) - 1 ) < 0 ) 
				r += (x-- -1) << 1;
		}
	}
}

void CDib::DrawEllipse(double xc, double yc, double rx, double ry, double angle, COLORREF color)
{
    double rad = angle * PI / 180.;
    double cosV = cos(rad);
    double sinV = sin(rad);
    double xx, yy;
	int x, x1, y, y1, r;

	if(rx > ry) {
		x = r = (int) rx; y = 0;
		while(x >= y){
			x1 = (int) ((long)x *ry/rx);
			y1 = (int) ((long) y*ry /rx);
            xx = x*cosV + y1*sinV;
            yy = -x*sinV + y1*cosV;
            DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = x*cosV - y1*sinV;
            yy = -x*sinV - y1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -x*cosV + y1*sinV;
            yy = x*sinV + y1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -x*cosV - y1*sinV;
            yy = x*sinV - y1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -x*cosV + y1*sinV;
            yy = x*sinV + y1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = y*cosV + x1*sinV;
            yy = -y*sinV + x1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = y*cosV - x1*sinV;
            yy = -y*sinV - x1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -y*cosV + x1*sinV;
            yy = y*sinV + x1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -y*cosV - x1*sinV;
            yy = y*sinV - x1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
			if (( r -= (y++ << 1) - 1 ) < 0 )
				r += (x-- -1) << 1;
		}
	}else{
		x = r = (int) ry; y = 0;
		while( x >= y){
			x1 = (int) ((long) x*rx/ry);
			y1 = (int)((long) y*rx/ry);
            xx = x1*cosV + y*sinV;
            yy = -x1*sinV + y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = x1*cosV - y*sinV;
            yy = -x1*sinV - y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -x1*cosV + y*sinV;
            yy = x1*sinV + y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -x1*cosV - y*sinV;
            yy = x1*sinV - y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -x1*cosV + y*sinV;
            yy = x1*sinV + y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = y1*cosV + x*sinV;
            yy = -y1*sinV + x*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = y1*cosV - x*sinV;
            yy = -y1*sinV - x*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -y1*cosV + x*sinV;
            yy = y1*sinV + x*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
            xx = -y1*cosV - x*sinV;
            yy = y1*sinV - x*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), color);
			if ((r -= (y++ << 1) - 1 ) < 0 ) 
				r += (x-- -1) << 1;
		}
	}
}


void CDib::DrawRect(RECT rect, BOOL bFillIn, COLORREF color)
{
    if (IsGrayImage()) {
      DrawRect(rect, bFillIn, color);
      return;
    }

    unsigned char r=GetRValue(color), g=GetGValue(color), b=GetBValue(color);
	unsigned char *ptr;
    rect = ClipRect(rect);
    if (bFillIn) {
      for (int i=rect.top; i<rect.bottom ;i++) {
        ptr = GetPointer(rect.left, i);
        for (int j=rect.left; j<rect.right ;j++) {
          *ptr++ = b;
          *ptr++ = g;
          *ptr++ = r;
        }
      }
    }
    else {
      for (int i=rect.top; i<rect.bottom ;i++) {
        ptr = GetPointer(rect.left, i);
        if ((i==rect.top) || (i==(rect.bottom-1))) {
          for (int j=rect.left; j<rect.right ;j++) {
            *ptr++ = b;
            *ptr++ = g;
            *ptr++ = r;
          }
        }
        else {
          *ptr++ = b; *ptr++ = g; *ptr++ = r;
          ptr = GetPointer(rect.right-1, i);
          *ptr++ = b; *ptr++ = g; *ptr++ = r;
        }
      }
    }
}

CDib *CDib::GetTextImage(const char *pCaption, CRect region, const char *pFontName, int size)
{
    // Create Memory DC and Memory Compatible Bitmap
    CDC *pDC = CWnd::GetDesktopWindow()->GetDC();
    CDC memDC;
    memDC.CreateCompatibleDC(pDC);
    CBitmap bitmap;
    bitmap.CreateCompatibleBitmap(pDC, region.Width(), region.Height());
    CBitmap *pOldBitmap = (CBitmap *) memDC.SelectObject(&bitmap);

    CFont font;
	font.CreatePointFont(size, pFontName);
	CFont *pOldFont = memDC.SelectObject(&font);
    memDC.SetTextColor(RGB(255,255,255));
    
    memDC.SetBkMode(TRANSPARENT);

    // Create CDib
    BITMAP BitmapInfo;
    bitmap.GetObject(sizeof(BITMAP), &BitmapInfo);
    CDib *pDib = new CDib;
    pDib->Allocate(region.Width(), region.Height(), BitmapInfo.bmBitsPixel);
    pDib->ResetContents();

    CRect rect(0, 0, region.Width(), region.Height());
    memDC.SetBkColor(RGB(255,255,255));
    memDC.DrawText(pCaption, &rect, DT_CENTER | DT_SINGLELINE | DT_VCENTER);

    memDC.SelectObject(pOldBitmap);
    memDC.SelectObject(pOldFont);

    DWORD dwCount = bitmap.GetBitmapBits(pDib->ByteWidth() * pDib->Height(), pDib->GetPattern());

    // post processing
    CWnd::GetDesktopWindow()->ReleaseDC(pDC);

    pDib->UpsideDown();

    return pDib;
}

void CDib::DrawText(const char *pCaption, CRect region, const char *pFontName, int size, COLORREF color)
{
    if (BitCount()==8) {    // call gray version
      DrawText(pCaption, region, pFontName, size, GetIntensity(color));
      return;
    }

    CDib *pDib = GetTextImage(pCaption, region, pFontName, size);
    int ostep = BitCount()/8;
    int istep = pDib->BitCount()/8;

	CRect image(0, 0, Width(), Height());
	region &= image;

    unsigned char r = GetRValue(color);
    unsigned char g = GetGValue(color);
    unsigned char b = GetBValue(color);
    for (int i=region.top; i<region.bottom ;i++) {
      unsigned char *pO = GetPointer(region.left, i);
      unsigned char *pI = pDib->GetPointer(0, i-region.top);
      for (int j=0; j<region.Width() ;j++, pO+=ostep, pI+=istep) 
        if (*pI!=0) {
          *pO = b;
          *(pO+1) = g;
          *(pO+2) = r;
        }
    }

    delete pDib;
}


/////////////////////////////////////////////////////////////////////////////////
// class CDibDrawing_Gray

void CDib::AddPoints(FPointArray *pPoints, unsigned char value)
{
	int x, y;
	unsigned char *ptr;
	for (int i=0; i<pPoints->GetSize() ;i++) {
	  x = (int) pPoints->GetAt(i).x;
	  y = (int) pPoints->GetAt(i).y;
	  if ((x<2) || (x>=(Width()-2)) ||
	      (y<2) || (y>=(Height()-2))) continue;
	  ptr = GetPointer(x-2, y);
	  *ptr++ = value; *ptr++ = value; *ptr++ = value; *ptr++ = value; *ptr++ = value; 
	  ptr = GetPointer(x, y+2);
	  *ptr = value; ptr += ByteWidth();
	  *ptr = value; ptr += ByteWidth();
	  *ptr = value; ptr += ByteWidth();
	  *ptr = value; ptr += ByteWidth();
	  *ptr = value;
	 }
}

void CDib::AddPoints(CPointArray *pPoints, unsigned char value)
{
	int x, y;
	unsigned char *ptr;
	for (int i=0; i<pPoints->GetSize() ;i++) {
	  x = pPoints->GetAt(i).x;
	  y = pPoints->GetAt(i).y;
	  if ((x<2) || (x>=(Width()-2)) ||
	      (y<2) || (y>=(Height()-2))) continue;
	  ptr = GetPointer(x-2, y);
	  *ptr++ = value; *ptr++ = value; *ptr++ = value; *ptr++ = value; *ptr++ = value; 
	  ptr = GetPointer(x, y+2);
	  *ptr = value; ptr += ByteWidth();
	  *ptr = value; ptr += ByteWidth();
	  *ptr = value; ptr += ByteWidth();
	  *ptr = value; ptr += ByteWidth();
	  *ptr = value;
	 }
}


void CDib::AddRects(RectArray *pRects, unsigned char value)
{	
	unsigned char *ptr1, *ptr2;
	CRect rect;
    CRect canvas(0, 0, Width()-1, Height()-1);
	int i, j;
	for (i=0; i<pRects->GetSize() ;i++) {
	  rect = pRects->GetAt(i);
      rect &= canvas;
	  ptr1 = GetPointer(rect.left, rect.top);
	  ptr2 = GetPointer(rect.left, rect.bottom);
	  for (j=0; j<(rect.right-rect.left) ;j++) {
	    *ptr1++ = value;
	    *ptr2++ = value;
	  }
	  ptr1 = GetPointer(rect.left, rect.bottom);
	  ptr2 = GetPointer(rect.right, rect.bottom);
	  for (j=0; j<(rect.bottom-rect.top) ;j++) {
	    *ptr1 = value; ptr1 += ByteWidth();
	    *ptr2 = value; ptr2 += ByteWidth();
	  }
	}
}

void CDib::SetCircularPalette()
{
    // if palette is not available, return
	if (BitCount()>8) return;

    PALETTEENTRY pal[256];
    pal[0].peRed = pal[0].peGreen = pal[0].peBlue = pal[0].peFlags = 0;
    pal[255].peRed = pal[255].peGreen = pal[255].peBlue = pal[255].peFlags = 255;

    double h, s=1., v=1.;
    RGBQUAD rgb;
    for (int i=1; i<255 ;i++) {
      h = ((double) i)*360./255.;
      rgb = HSVtoRGB(h, s, v);
      pal[i].peRed = rgb.rgbRed;
      pal[i].peGreen = rgb.rgbGreen;
      pal[i].peBlue = rgb.rgbBlue;
      pal[i].peFlags = 0;
    }

    SetPaletteEntries(0, 256, pal);
}

void CDib::DrawDot(int x, int y, unsigned char value)
{
	if ((x<0) || (x>=Width()) || (y<0) || (y>=Height())) return;
	*(GetPointer(x, y)) = value;
}

void CDib::DrawCross(int x, int y, unsigned char value)
{
	if ((x<2) || (x>=(Width()-2)) || (y<2) || (y>=(Height()-2))) return;
	
	// original code backup
	/*unsigned char *ptr = GetPointer(x-2, y);
	*ptr++ = value;
	*ptr++ = value;
	*ptr++ = value;
	*ptr++ = value;
	*ptr++ = value;
	ptr = GetPointer(x, y+2);
	*ptr = value; ptr += ByteWidth();
	*ptr = value; ptr += ByteWidth();
				  ptr += ByteWidth();
	*ptr = value; ptr += ByteWidth();
	*ptr = value;*/


	// modified code by twkim
	if(BitCount() == 8) {
		unsigned char *ptr = GetPointer(x-2, y);
		*ptr++ = value;
		*ptr++ = value;
		*ptr++ = value;
		*ptr++ = value;
		*ptr++ = value;
		ptr = GetPointer(x, y+2);
		*ptr = value; ptr += ByteWidth();
		*ptr = value; ptr += ByteWidth();
					  ptr += ByteWidth();
		*ptr = value; ptr += ByteWidth();
		*ptr = value;
	} else if(BitCount() == 32) {
		
		unsigned char *ptr = GetPointer(x-2, y);
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr++ = 0;	// BGRA in order
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr++ = 0;
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr++ = 0;
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr++ = 0;
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr++ = 0;

		ptr = GetPointer(x, y+2);
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr = 0; ptr += ByteWidth()-3;
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr = 0; ptr += ByteWidth()-3;
		 ptr++;		 ptr++;		 ptr++;			*ptr;	  ptr += ByteWidth()-3;
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr = 0; ptr += ByteWidth()-3;
		*ptr++ = 0; *ptr++ = 0; *ptr++ = value; *ptr = 0;
	}
}

void CDib::DrawLine(int x1, int y1, int x2, int y2, unsigned char value)
{
	int dx, dy, s, step;

	dx = abs(x2-x1); dy = abs(y2-y1);
	if (dx>dy) {
	  if (x1>x2) {
	    step = (y1>y2) ? 1 : -1;
		s = x1; x1 = x2; x2 = s; y1 = y2;
	  }
	  else step = (y1<y2) ? 1 : -1;
	  DrawDot(x1, y1, value);
	  s = dx >> 1;
	  while (++x1 <= x2) {
	    if ((s-=dy) < 0) {
		  s += dx; y1 += step;
		}
		DrawDot(x1, y1, value);
	  }
	}
	else {
	  if (y1>y2) {
	    step = (x1>x2) ? 1 : -1;
		s = y1; y1 = y2; y2 = s; x1 = x2;
	  }
	  else step = (x1<x2) ? 1 : -1;
	  DrawDot(x1, y1, value);
	  s = dy >> 1;
	  while (++y1 <= y2) {
	    if ((s-=dx)<0) {
		  s += dy; x1 += step;
		}
		DrawDot(x1, y1, value);
	  }
	}
}

void CDib::DrawHLines(HLineArray *pLines, unsigned char value)
{
	POINT pt1, pt2;
	for (int i=0; i<pLines->GetSize() ;i++) {
	  HLine line = pLines->GetAt(i);
	  double a=cos(line.theta), b=sin(line.theta);
	  if (fabs(a)<0.001) {
	    pt1.x = pt2.x = (int) (line.rho+0.5);
		pt1.y = 0;
		pt2.y = Height() - 1;
	  }
	  else if (fabs(b)<0.001) {
	    pt1.y = pt2.y = (int) (line.rho+0.5);
		pt1.x = 0;
		pt2.x = Width() - 1;
	  }
	  else {
	    pt1.x = 0;
		pt1.y = (int) (line.rho/b + 0.5);
	    pt2.x = (int) (line.rho/a + 0.5);
		pt2.y = 0;
	  }
	  DrawLine(pt1.x, pt1.y, pt2.x, pt2.y, value);
	}
}

// draw circle 
void CDib::DrawCircle(int xc, int yc, int radius, unsigned char value)
{
	int dx, dy;

	dx = radius; dy = 0;

	while(dx >= dy){
		DrawDot(xc+dx, yc+dy, value);
		DrawDot(xc+dx, yc-dy, value);
		DrawDot(xc-dx, yc+dy, value);
		DrawDot(xc-dx, yc-dy, value);
		DrawDot(xc+dy, yc+dx, value);
		DrawDot(xc+dy, yc-dx, value);
		DrawDot(xc-dy, yc+dx, value);
		DrawDot(xc-dy, yc-dx, value);
		if( (radius -= (dy++ << 1) -1) < 0 )
			radius += ( dx-- -1) << 1;
	}
}

// draw an ellipse
void CDib::DrawEllipse(int xc, int yc, int rx, int ry, unsigned char value)
{
	int x, x1, y, y1, r;

	if(rx > ry) {
		x = r = rx; y = 0;
		while(x >= y){
			x1 = (int) ((long)x *ry/rx);
			y1 = (int) ((long) y*ry /rx);
			DrawDot(xc + x, yc+y1, value);
			DrawDot(xc + x, yc-y1, value);
			DrawDot(xc - x, yc + y1, value);
			DrawDot(xc - x, yc - y1, value);
			DrawDot(xc - x, yc + y1, value);
			DrawDot(xc + y, yc + x1, value);
			DrawDot(xc + y, yc - x1, value);
			DrawDot(xc - y, yc + x1, value);
			DrawDot(xc - y, yc - x1, value);
			if (( r -= (y++ << 1) - 1 ) < 0 )
				r += (x-- -1) << 1;
		}
	}else{
		x = r = ry; y = 0;
		while( x >= y){
			x1 = (int) ((long) x*rx/ry);
			y1 = (int)((long) y*rx/ry);
			DrawDot(xc + x1, yc + y, value);
			DrawDot(xc + x1, yc - y, value);
			DrawDot(xc - x1, yc + y, value);
			DrawDot(xc - x1, yc - y, value);
			DrawDot(xc - x1, yc + y, value);
			DrawDot(xc + y1, yc + x, value);
			DrawDot(xc + y1, yc - x, value);
			DrawDot(xc - y1, yc + x, value);
			DrawDot(xc - y1, yc - x, value);
			if ((r -= (y++ << 1) - 1 ) < 0 ) 
				r += (x-- -1) << 1;
		}
	}
}

void CDib::DrawEllipse(double xc, double yc, double rx, double ry, double angle, unsigned char value)
{
    double rad = angle * PI / 180.;
    double cosV = cos(rad);
    double sinV = sin(rad);
    double xx, yy;
	int x, x1, y, y1, r;

	if(rx > ry) {
		x = r = (int) rx; y = 0;
		while(x >= y){
			x1 = (int) ((long)x *ry/rx);
			y1 = (int) ((long) y*ry /rx);
            xx = x*cosV + y1*sinV;
            yy = -x*sinV + y1*cosV;
            DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = x*cosV - y1*sinV;
            yy = -x*sinV - y1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -x*cosV + y1*sinV;
            yy = x*sinV + y1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -x*cosV - y1*sinV;
            yy = x*sinV - y1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -x*cosV + y1*sinV;
            yy = x*sinV + y1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = y*cosV + x1*sinV;
            yy = -y*sinV + x1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = y*cosV - x1*sinV;
            yy = -y*sinV - x1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -y*cosV + x1*sinV;
            yy = y*sinV + x1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -y*cosV - x1*sinV;
            yy = y*sinV - x1*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
			if (( r -= (y++ << 1) - 1 ) < 0 )
				r += (x-- -1) << 1;
		}
	}else{
		x = r = (int) ry; y = 0;
		while( x >= y){
			x1 = (int) ((long) x*rx/ry);
			y1 = (int)((long) y*rx/ry);
            xx = x1*cosV + y*sinV;
            yy = -x1*sinV + y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = x1*cosV - y*sinV;
            yy = -x1*sinV - y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -x1*cosV + y*sinV;
            yy = x1*sinV + y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -x1*cosV - y*sinV;
            yy = x1*sinV - y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -x1*cosV + y*sinV;
            yy = x1*sinV + y*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = y1*cosV + x*sinV;
            yy = -y1*sinV + x*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = y1*cosV - x*sinV;
            yy = -y1*sinV - x*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -y1*cosV + x*sinV;
            yy = y1*sinV + x*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
            xx = -y1*cosV - x*sinV;
            yy = y1*sinV - x*cosV;
			DrawDot((int) (xc+xx), (int) (yc+yy), value);
			if ((r -= (y++ << 1) - 1 ) < 0 ) 
				r += (x-- -1) << 1;
		}
	}
}

void CDib::DrawRect(RECT rect, BOOL bFillIn, unsigned char value)
{
    rect = ClipRect(rect);
    unsigned char *ptr;
    if (bFillIn) {
      for (int i=rect.top; i<rect.bottom ;i++) {
        ptr = GetPointer(rect.left, i);
        for (int j=rect.left; j<rect.right ;j++) 
          *ptr++ = value;
      }
    }
    else {
      for (int i=rect.top; i<rect.bottom ;i++) {
        ptr = GetPointer(rect.left, i);
        if ((i==rect.top) || (i==(rect.bottom-1))) {
          for (int j=rect.left; j<rect.right ;j++) 
            *ptr++ = value;
        }
        else {
          *ptr = value;
          ptr = GetPointer(rect.right-1, i);
          *ptr = value;
        }
      }
    }
}

void CDib::DrawText(const char *pCaption, CRect region, const char *pFontName, int size, unsigned char value)
{
    CDib *pDib = GetTextImage(pCaption, region, pFontName, size);
    int ostep = BitCount()/8;
    int istep = pDib->BitCount()/8;

	CRect image(0, 0, Width(), Height());
	region &= image;

    for (int i=region.top; i<region.bottom ;i++) {
      unsigned char *pO = GetPointer(region.left, i);
      unsigned char *pI = pDib->GetPointer(0, i-region.top);
      for (int j=0; j<region.Width() ;j++, pO++, pI+=istep) 
        if (*pI!=0) *pO = value;
    }

    delete pDib;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Color Conversion Functions
//
//////////////////////////////////////////////////////////////////////////////////////
CDib *CDib::GetFullcolorCDib()
{
    int nBits = BitCount();

    switch (nBits) {
      case 24:
        return (CDib *) CopyCDib();
      case 32:
        {
	      CDib *pRes = new CDib;
	      pRes->Allocate(Width(), Height(), 24);
	      for (int i=0; i<Height() ;i++) {
	        unsigned char *pI = GetPointer(0, i);
		    unsigned char *pO = pRes->GetPointer(0, i);
		    for (int j=0; j<Width() ;j++) {
		      *pO++ = *pI++;
		      *pO++ = *pI++;
		      *pO++ = *pI++;
		      pI++;
		    }
	      }
	      return pRes;
        }
        break;
      case 8:
        {
	      CDib *pRes = new CDib;
	      pRes->Allocate(Width(), Height(), 24);
	      RGBQUAD *rgb = GetRgbQuadPointer();
	      int index;
	      for (int i=0; i<Height() ;i++) {
	        unsigned char *pI = GetPointer(0, i);
		    unsigned char *pO = pRes->GetPointer(0, i);
		    for (int j=0; j<Width() ;j++) {
		      index = (int) *pI++;
		      *pO++ = rgb[index].rgbBlue;
		      *pO++ = rgb[index].rgbGreen;
		      *pO++ = rgb[index].rgbRed;
		    } 
	      }

	      return pRes;
        }
        break;
      default:
    	TRACE("??? UIPL ERROR: GetFullcolorCDib : Input Image Format Not Supported\n");
        break;
    }

	return NULL;
}


RGBQUAD CDib::HSVtoRGB(double h, double s, double v)
{
        double f, i;
        double p1, p2, p3;
        RGBQUAD res;

        if (s<0) s=0; if (s>1.) s=1.;
        if (v<0.) v=0.; if (v>1.) v=1.;
        h = fmod(h, 360.); if (h<0.) h+=360.;
        
        h /= 60.;
        f = modf(h, &i);
        p1 = v*(1-s);
        p2 = v*(1-s*f);
        p3 = v*(1-s*(1-f));

        memset(&res, 0, sizeof(RGBQUAD));
        switch ((int) i) {
          case 0 :
            res.rgbRed = (unsigned char) (v*255);
            res.rgbGreen = (unsigned char) (p3*255);
            res.rgbBlue = (unsigned char) (p1*255);
            break;
          case 1 :
            res.rgbRed = (unsigned char) (p2*255);
            res.rgbGreen = (unsigned char) (v*255);
            res.rgbBlue = (unsigned char) (p1*255);
            break;
          case 2 :
            res.rgbRed = (unsigned char) (p1*255);
            res.rgbGreen = (unsigned char) (v*255);
            res.rgbBlue = (unsigned char) (p3*255);
            break;
          case 3 :
            res.rgbRed = (unsigned char) (p1*255);
            res.rgbGreen = (unsigned char) (p2*255);
            res.rgbBlue = (unsigned char) (v*255);
            break;
          case 4 :
            res.rgbRed = (unsigned char) (p3*255);
            res.rgbGreen = (unsigned char) (p1*255);
            res.rgbBlue = (unsigned char) (v*255);
            break;
          case 5 :
            res.rgbRed = (unsigned char) (v*255);
            res.rgbGreen = (unsigned char) (p1*255);
            res.rgbBlue = (unsigned char) (p2*255);
            break;
          default :
            break;
        }

        return res;
}

CDib *CDib::RGBtoYUV()
{
	if (BitCount()<24) return NULL;

    CDib *pRes = CopyCDib();

	unsigned char Y, U, V;
    for (int y=0; y<Height() ;y++) {
      unsigned char *pO = pRes->GetPointer(0, y);
      for (int x=0; x<Width() ;x++) {
	    RGBtoYUV(GetColor(x,y),&Y,&U,&V);  
        *pO++ = Y;
        *pO++ = U;
        *pO++ = V;
	  }
    }									

    return pRes;
}

void CDib::RGBtoYUV_IP()
{
	if (BitCount()<24) return;

	unsigned char Y, U, V;
    for (int y=0; y<Height() ;y++) {
      unsigned char *pO = GetPointer(0, y);
      for (int x=0; x<Width() ;x++) {
	    RGBtoYUV(GetColor(x,y),&Y,&U,&V);  
        *pO++ = Y;
        *pO++ = U;
        *pO++ = V;
	  }
    }									
}

void CDib::YUVtoRGB_IP()
{
	RGBQUAD quad;
	for (int y=0; y<Height() ;y++) {
	  unsigned char *pO = GetPointer(0, y);
	  unsigned char *pI = GetPointer(0, y);
	  for (int x=0; x<Width() ;x++, pI+=3) {
		quad = YUVtoRGB(*pI, *(pI+1), *(pI+2));
		*pO++ = quad.rgbBlue;
		*pO++ = quad.rgbGreen;
		*pO++ = quad.rgbRed;
	  }
	}
}


//////////////////////////////////////////////////////////////////////////////////////
//
// Other Utility Functions
//
//////////////////////////////////////////////////////////////////////////////////////
unsigned char CDib::GetIntensity(COLORREF color)
{
	int value = (int) (0.299 * (double) GetRValue(color)
					 + 0.587 * (double) GetGValue(color)
					 + 0.114 * (double) GetBValue(color) + 0.5);
	value = min(value, 255);
	value = max(value, 0);

	return (unsigned char) value;
}

unsigned char CDib::GetIntensity(unsigned char r, unsigned char g, unsigned char b)
{
	int value = (int) (0.299 * (double) r + 0.587 * (double) g + 0.114 * (double) b + 0.5);
	value = min(value, 255);
	value = max(value, 0);

	return (unsigned char) value;
}

unsigned char CDib::GetIntensityFast(COLORREF color)
{
	int sum = ((int) GetRValue(color) + (int) GetGValue(color) + (int) GetBValue(color))/3;
	sum = min(sum, 255);
	sum = max(sum, 0);
	return (unsigned char) sum;
}

unsigned char CDib::GetIntensityFast(unsigned char r, unsigned char g, unsigned char b)
{
	int sum = ((int) r + (int) g + (int) b)/3;
	sum = min(sum, 255);
	sum = max(sum, 0);
	return (unsigned char) sum;
}

unsigned char CDib::ClipByte(int value)
{
    if (value<0) return 0;
    else if (value>255) return 255;
    else return (unsigned char) value;
}

CRect CDib::ClipRect(CRect rect, CDib *pDib)
{
	CRect image;
	if (pDib) image.SetRect(0, 0, pDib->Width(), pDib->Height());
	else image.SetRect(0, 0, Width(), Height());

	image &= rect;
	return image;
}



//////////////////////////////////////////////////////////////////////////////////////
//
// Color Format Conversions
//
//////////////////////////////////////////////////////////////////////////////////////
void CDib::RGBtoYUV(RGBQUAD rgb, unsigned char *y, unsigned char *u, unsigned char *v)
{
        //Y Cr=V Cb=U
	// U V is Cb Cr and Shift +128    
	*y = (unsigned char) ( 0.299 * (double) rgb.rgbRed 
                             + 0.587 * (double) rgb.rgbGreen
                             + 0.114 * (double) rgb.rgbBlue );
        // U V is Cb Cr and Shift +128
	*u = (unsigned char) ( -0.169 * (double) rgb.rgbRed 	
                             - 0.331 * (double) rgb.rgbGreen
                             + 0.5 * (double) rgb.rgbBlue +128);
    *v = (unsigned char) ( 0.5 * (double) rgb.rgbRed 
                             - 0.419 * (double) rgb.rgbGreen
                             - 0.081 * (double) rgb.rgbBlue +128);
}

RGBQUAD CDib::YUVtoRGB(unsigned char y, unsigned char u, unsigned char v)
{
    // R = L + 1.40200*Cr;
    // G = L - 0.34414*Cb - 0.71414*Cr
    // B = L + 1.77200*Cb;       
    int L,Cr,Cb ;
    RGBQUAD rgb;
        	
    L= y;
	Cr= v-128;
	Cb= u-128;

    int red = (int) (L+1.40200*Cr);
    int green = (int) ( L - 0.34414*Cb - 0.71414*Cr);
    int blue = (int) ( L + 1.77200*Cb);
    red = max(0, red); red = min(255, red);
    green = max(0, green); green = min(255, green);
    blue = max(0, blue); blue = min(255, blue);
	rgb.rgbRed= (unsigned char) red;
    rgb.rgbGreen=(unsigned char) green;	 
	rgb.rgbBlue=(unsigned char) blue;

    return rgb;
}

BOOL CDib::RGBtoYUV(CDib **ppY, CDib **ppU, CDib **ppV)
{
	if (BitCount()<24) return FALSE;

	*ppY = new CDib; CDib *pY = *ppY; pY->Allocate(Width(), Height(), 8); pY->SetGrayPalette();
	*ppU = new CDib; CDib *pU = *ppU; pU->Allocate(Width(), Height(), 8); pU->SetGrayPalette();
	*ppV = new CDib; CDib *pV = *ppV; pV->Allocate(Width(), Height(), 8); pV->SetGrayPalette();

	unsigned char Y, U, V;
    for (int y=0; y<Height() ;y++) {
      unsigned char *pNY = pY->GetPointer(0, y);
	  unsigned char *pNU = pU->GetPointer(0, y);
	  unsigned char *pNV = pV->GetPointer(0, y);
      for (int x=0; x<Width() ;x++) {
	    RGBtoYUV(GetColor(x,y),&Y,&U,&V);  
        *pNY++ = Y;
		*pNU++ = U;
		*pNV++ = V;
	  }
    }									

    return TRUE;
}

CDib *CDib::YUVtoRGB(CDib *pY, CDib *pU, CDib *pV)
{
	CDib *pRes = new CDib; pRes->Allocate(pY->Width(), pY->Height(), 24);

	RGBQUAD quad;
	for (int y=0; y<pRes->Height() ;y++) {
	  unsigned char *pO = pRes->GetPointer(0, y);
	  unsigned char *pIY = pY->GetPointer(0, y);
	  unsigned char *pIU = pU->GetPointer(0, y);
	  unsigned char *pIV = pV->GetPointer(0, y);
	  for (int x=0; x<pRes->Width() ;x++) {
		quad = YUVtoRGB(*pIY++, *pIU++, *pIV++);
		*pO++ = quad.rgbBlue;
		*pO++ = quad.rgbGreen;
		*pO++ = quad.rgbRed;
	  }
	}

	return pRes;
}


//////////////////////////////////////////////////////////////////////////////////////
//
// Color Moment
//
//////////////////////////////////////////////////////////////////////////////////////
void CDib::CalculateHistogramMoments(int *pHistogram, int size, int SumofItem, double pFeature[3])
{
    if (SumofItem<=0) {
      memset(pFeature, 0, sizeof(double) * 3);
      return;
    }

    double *pNormal = new double [size];
    int i;
    for (i=0; i<size ;i++) 
      pNormal[i] = (double) pHistogram[i] / (double) SumofItem;

    // Histogram의 1차 2차 3차 Central moment를 구함   
    // mean = E[X] , variance =E[X^2]-E[X]^2 ; skew = E[X^3] -3E[X]E[X^2]+2E[X]^3
	double Ex1=0,Ex2=0,Ex3=0,temp=0; 
	
	// 1차 2차 모멘트 계산 
	for (i=0; i<size ;i++) {
	  Ex1=Ex1+ (double) i* pNormal[i];                
	  Ex2=Ex2 +((double) i * (double) i)* pNormal[i];
	}

	pFeature[0]= Ex1 / size;
    Ex1 = Ex1 / size;
        	
    Ex2 = 0;
    for (i=0; i<size ;i++) 
      Ex2 += ((i*pNormal[i]-Ex1)*(i*pNormal[i]-Ex1));
    Ex2 /= size;
          
	temp= Ex2;
	if (temp<0) {
      TRACE("error temp\n");
      return;
    }


	pFeature[1]=sqrt(temp);  // 2차 중심 모멘트 표준편차 
		
	// 3차 모멘트 계산 
	Ex3=0;
	for(i =0; i<size ;i++){
		Ex3=Ex3+ (i*pNormal[i]-Ex1)*(i*pNormal[i]-Ex1)*(i*pNormal[i]-Ex1);
	}

	temp= Ex3/size;
	if(temp<0) {
  	  temp=-temp; 
	  pFeature[2]=pow(temp,1.0/3.0);
	  pFeature[2]=-pFeature[2];
	}
	else pFeature[2]=pow(temp,1.0/3.0);

    delete [] pNormal;
}

void CDib::GetColorMoments(double pFeature[9])
{
    int *pHistR = new int [256]; memset(pHistR, 0, sizeof(int)*256);
    int *pHistG = new int [256]; memset(pHistG, 0, sizeof(int)*256);
    int *pHistB = new int [256]; memset(pHistB, 0, sizeof(int)*256);

	int width=Width();
	int height=Height();
		
    // Get Histogram of RGB color model
	for (int y=0; y< height ;y++) {
      unsigned char *ptr = GetPointer(0, y);
      for (int x=0; x< width ;x++) {
        pHistB[*ptr++]++;
        pHistG[*ptr++]++;
        pHistR[*ptr++]++;
      }
    }

    int count = width * height;
    CalculateHistogramMoments(pHistR, 256, count, &pFeature[0]);
    CalculateHistogramMoments(pHistG, 256, count, &pFeature[3]);
    CalculateHistogramMoments(pHistB, 256, count, &pFeature[6]);

    delete [] pHistR;
    delete [] pHistG;
    delete [] pHistB;
}

void CDib::GetMaskedColorMoments(CDib *pBin, double pFeature[9])
{
    int *pHistR = new int [256]; memset(pHistR, 0, sizeof(int)*256);
    int *pHistG = new int [256]; memset(pHistG, 0, sizeof(int)*256);
    int *pHistB = new int [256]; memset(pHistB, 0, sizeof(int)*256);

	int width=Width();
	int height=Height();
		
    // Get Histogram of RGB color model
    int count=0;
	for (int y=0; y< height ;y++) {
      unsigned char *ptr = GetPointer(0, y);
      unsigned char *pM = pBin->GetPointer(0, y);
      for (int x=0; x< width ;x++, pM++) {
        if (*pM) {
          pHistB[*ptr++]++;
          pHistG[*ptr++]++;
          pHistR[*ptr++]++;
          count++;
        }
      }
    }

    CalculateHistogramMoments(pHistR, 256, count, &pFeature[0]);
    CalculateHistogramMoments(pHistG, 256, count, &pFeature[3]);
    CalculateHistogramMoments(pHistB, 256, count, &pFeature[6]);

    delete [] pHistR;
    delete [] pHistG;
    delete [] pHistB;
}

void CDib::GetYUVMoments(double pFeature[9])
{
    CDib *pYUV = RGBtoYUV();
    pYUV->GetColorMoments(pFeature);
    delete pYUV;
}

void CDib::GetMaskedYUVMoments(CDib *pBin, double pFeature[9])
{
    CDib *pYUV = RGBtoYUV();
    pYUV->GetMaskedColorMoments(pBin, pFeature);
    delete pYUV;
}

void CDib::RGBtoHSV(RGBQUAD &rgb, double *h, double *s, double *v)
{
	double cmax,cmin,cd,hue;
    double rgbRed,rgbGreen,rgbBlue;
    rgbRed=(double)rgb.rgbRed;    // rgb Normalization  [0,255]=>[0,1]
    rgbGreen=(double)rgb.rgbGreen;
    rgbBlue=(double)rgb.rgbBlue; 
	// find max and min value of RGB
	if(rgb.rgbRed>=rgb.rgbGreen) cmax=rgbRed; else cmax=rgbGreen;if(rgb.rgbBlue>=cmax) cmax=rgbBlue;
	if(rgb.rgbRed<=rgb.rgbGreen) cmin=rgbRed; else cmin=rgbGreen;if(rgb.rgbBlue<=cmin) cmin=rgbBlue;

	*v= cmax/255.0;
	cd= cmax-cmin;
		
	if(cmax==0) *s=0;else *s = cd/cmax;
	if(*s==0){*s=0; *h=0; return ;}
	if(rgb.rgbRed==cmax) hue=(rgb.rgbGreen-rgb.rgbBlue)/cd; // Hue의 Angle 300-60 
	else if(rgb.rgbGreen==cmax) hue=2+(rgb.rgbBlue-rgb.rgbRed)/cd; // 60-180 
	else if(rgb.rgbBlue==cmax) hue=4+(rgb.rgbRed-rgb.rgbGreen)/cd;  //180-300 
	hue=hue* 60; if(hue<0) hue+=360;
	*h= hue;
}

void CDib::GetHSVMoments(double pFeature[9])
{
    int *pHistH = new int [360]; memset(pHistH, 0, sizeof(int)*360);
    int *pHistS = new int [256]; memset(pHistS, 0, sizeof(int)*256);
    int *pHistV = new int [256]; memset(pHistV, 0, sizeof(int)*256);
	double h, s, v;
    int saturation, value, hue;
	RGBQUAD rgb;

	int width=Width();
	int height=Height();
		
    // Get Histogram of HSV color model
	for (int y=0; y< height ;y++) 
      for (int x=0; x< width ;x++ ){
        rgb=GetColor(x,y);
	    RGBtoHSV(rgb, &h, &s, &v); // RGB=> HSV 
          
        hue= (int) h;
	    if(hue>360) hue%=360;
	    else if(hue<0) hue= 360+(hue%360);
			 
        saturation=(int) (s* 255.);
        value=(int) (v* 255.);
               
        if(saturation>255)saturation=255;else if(saturation<0) saturation=0;
        if(value>255)value=255;else if(value<0) value=0;

        pHistH[hue]++;
        pHistS[saturation]++;
        pHistV[value]++;
      }

      CalculateHistogramMoments(pHistH, 360, width * height, &pFeature[0]);
      CalculateHistogramMoments(pHistS, 256, width * height, &pFeature[3]);
      CalculateHistogramMoments(pHistV, 256, width * height, &pFeature[6]);

      delete [] pHistH;
      delete [] pHistS;
      delete [] pHistV;
}

void CDib::GetMaskedHSVMoments(CDib *pMask, double pFeature[9])
{
    int *pHistH = new int [360]; memset(pHistH, 0, sizeof(int)*360);
    int *pHistS = new int [256]; memset(pHistS, 0, sizeof(int)*256);
    int *pHistV = new int [256]; memset(pHistV, 0, sizeof(int)*256);
	double h, s, v;
    int saturation, value, hue;
	RGBQUAD rgb;

	int width=Width();
	int height=Height();
		
    // Get Histogram of HSV color model
	int count=0;
	for (int y=0; y< height ;y++) {
      unsigned char *pM = pMask->GetPointer(0, y);
      for (int x=0; x< width ;x++, pM++ ){
		if (*pM==0) continue;

		count++;
        rgb=GetColor(x,y);
	    RGBtoHSV(rgb, &h, &s, &v); // RGB=> HSV 
          
        hue= (int) h;
	    if(hue>360) hue%=360;
	    else if(hue<0) hue= 360+(hue%360);
			 
        saturation=(int) (s* 255.);
        value=(int) (v* 255.);
               
        if(saturation>255)saturation=255;else if(saturation<0) saturation=0;
        if(value>255)value=255;else if(value<0) value=0;

        pHistH[hue]++;
        pHistS[saturation]++;
        pHistV[value]++;
      }
	}
    
	CalculateHistogramMoments(pHistH, 360, count, &pFeature[0]);
    CalculateHistogramMoments(pHistS, 256, count, &pFeature[3]);
    CalculateHistogramMoments(pHistV, 256, count, &pFeature[6]);

    delete [] pHistH;
    delete [] pHistS;
    delete [] pHistV;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Simple Conversions
//
//////////////////////////////////////////////////////////////////////////////////////
CDib *CDib::BlendImage(CDib *pDib1, CDib *pDib2, int weight1, int weight2)
{
	if ( (pDib1->Width()!=pDib2->Width()) ||
		 (pDib1->Height()!=pDib2->Height()) ||
		 (pDib1->BitCount()!=pDib2->BitCount()) ) return NULL;

	CDib *pRes = pDib1->CopyCDib();

	for (int i=0; i<pDib1->Height() ;i++) {
	  unsigned char *p1 = pDib1->GetPointer(0, i);
	  unsigned char *p2 = pDib2->GetPointer(0, i);
	  unsigned char *pO = pRes->GetPointer(0, i);
	  for (int j=0; j<pDib1->ByteWidth() ;j++) 
	    *pO++ = ClipByte((weight1 * (int) *p1++ + weight2 * (int) *p2++) / (weight1 + weight2));
	}

	return pRes;
}

CDib *CDib::Flip_Vertical()
{
	CDib *pRes = CopyCDib();

	for (int i=0; i<Height() ;i++) {
	  unsigned char *pS = GetPointer(0, i);
	  unsigned char *pD = pRes->GetPointer(0, pRes->Height()-i-1);
	  memcpy(pD, pS, pRes->ByteWidth());
	}

	return pRes;
}

void CDib::Flip_Vertical_IP()
{
	unsigned char *pBuf = new unsigned char [ByteWidth()];

	for (int i=0; i<Height()/2 ;i++) {
	  unsigned char *pS = GetPointer(0, i);
	  unsigned char *pD = GetPointer(0, Height()-i-1);

	  memcpy(pBuf, pD, ByteWidth());
	  memcpy(pD, pS, ByteWidth());
	  memcpy(pS, pBuf, ByteWidth());
	}

	delete [] pBuf;
}

CDib *CDib::Flip_Horizontal()
{
		CDib *pRes = CopyCDib();

        int nBytePerPixel;
        switch (BitCount()) {

          case 8 :
            nBytePerPixel = 1;
            break;
          case 16 :
            nBytePerPixel = 2;
          case 24 :
            nBytePerPixel = 3;
            break;
          case 32 :
            nBytePerPixel = 4;
            break;
          case 1 :
          case 4 :
          default :
#ifdef _DEBUG
            AfxMessageBox("??? Mirroring is possible only for the images with higher than 8 bit color resolution");
#endif
            return NULL;
        }

        char buffer[4];
        for (int i=0; i<pRes->Height() ;i++) {
          unsigned char *pSource = pRes->GetPointer(0, i);
          for (int j=0; j<pRes->Width()/2 ;j++) {
            memcpy(buffer, pSource+j*nBytePerPixel, nBytePerPixel);
            memcpy(pSource+j*nBytePerPixel, pSource+(pRes->Width()-j-1)*nBytePerPixel, nBytePerPixel);
            memcpy(pSource+(pRes->Width()-j-1)*nBytePerPixel, buffer, nBytePerPixel);
          }
        }

		return pRes;
}

void CDib::Flip_Horizontal_IP()
{
        int nBytePerPixel;
        switch (BitCount()) {

          case 8 :
            nBytePerPixel = 1;
            break;
          case 16 :
            nBytePerPixel = 2;
          case 24 :
            nBytePerPixel = 3;
            break;
          case 32 :
            nBytePerPixel = 4;
            break;
          case 1 :
          case 4 :
          default :
#ifdef _DEBUG
            AfxMessageBox("??? Mirroring is possible only for the images with higher than 8 bit color resolution");
#endif
            return;
        }

        char buffer[4];
        for (int i=0; i<Height() ;i++) {
          unsigned char *pSource = GetPointer(0, i);
          for (int j=0; j<Width()/2 ;j++) {
            memcpy(buffer, pSource+j*nBytePerPixel, nBytePerPixel);
            memcpy(pSource+j*nBytePerPixel, pSource+(Width()-j-1)*nBytePerPixel, nBytePerPixel);
            memcpy(pSource+(Width()-j-1)*nBytePerPixel, buffer, nBytePerPixel);
          }
        }
}

CDib *CDib::_Rotate(int degree)
{
        int nBytePerPixel;
        switch (BitCount()) {
          case 8 :
            nBytePerPixel = 1;
            break;
          case 16 :
            nBytePerPixel = 2;
          case 24 :
            nBytePerPixel = 3;
            break;
          case 32 :
            nBytePerPixel = 4;
            break;
          case 1 :
          case 4 :
          default :
#ifdef _DEBUG
            AfxMessageBox("??? Rotation is possible only for the images with higher than 8 bit color resolution");
#endif
            return NULL;
        }

        if (degree==180) {
          CDib *pDib = CopyCDib();
		  pDib->UpsideDown();
		  CDib *pRes = pDib->Flip_Horizontal();
		  delete pDib;
		  return pRes;
        }
        
        CDib *pDib = new CDib;
        pDib->Allocate(Height(), Width(), BitCount());
		if (BitCount()==8) pDib->SetGrayPalette();

        if (degree==90) {
          unsigned char **pSource = new unsigned char * [Height()];
          int i, j;
          for (i=0; i<Height() ;i++)
            pSource[i] = GetPointer(0, i);
          for (i=0; i<pDib->Height() ;i++) {
            unsigned char *pTarget = pDib->GetPointer(pDib->Width()-1, i);
            for (j=0; j<pDib->Width() ;j++, pTarget-=nBytePerPixel)
              memcpy(pTarget, pSource[j], nBytePerPixel);
            for (j=0; j<Height() ;j++)
              pSource[j] += nBytePerPixel;
          }

          delete [] pSource;
        }
        else if (degree==-90) {
          unsigned char **pSource = new unsigned char * [Height()];
          int i, j;
          for (i=0; i<Height() ;i++)
            pSource[i] = GetPointer(Width()-1, i);
          for (i=0; i<pDib->Height() ;i++) {
            unsigned char *pTarget = pDib->GetPointer(0, i);
            for (j=0; j<pDib->Width() ;j++, pTarget+=nBytePerPixel)
              memcpy(pTarget, pSource[j], nBytePerPixel);
            for (j=0; j<Height() ;j++)
              pSource[j] -= nBytePerPixel;
          }

          delete [] pSource;
        }
        else {
#ifdef _DEBUG
          AfxMessageBox("??? Only 90, -90, 180 degree rotation is allowed");
#endif
          delete pDib;
          return NULL;
        }
        
        return pDib;
}

CDib *CDib::RotateLeft()
{
	return _Rotate(-90);
}

CDib *CDib::RotateRight()
{
	return _Rotate(90);
}

CDib *CDib::Rotate180()
{
	return _Rotate(180);
}

double CDib::interpolation(double v0, double v1, double v2, double v3, double x)
{
	double P, Q, R, S;
	P = (v3 - v2) - (v0 - v1);
	Q = (v0 - v1) - P;
	R = v2 - v0; 
	S = v1;
	return (P*x*x*x + Q*x*x + R*x + S);
}

CDib* CDib::GetResizedDib(float fRatio, int nType)
{
	int outW, outH;
	outW = (int) (Width()*fRatio);
	outH = (int) (Height()*fRatio);

	return GetResizedDib(outW, outH, nType);
}

#define CDib2ORD(YY, XX)	((YY)*((inWD8+31)/32*4) + (XX)*inD + d)
CDib* CDib::GetResizedDib(int outW, int outH, int nType)
{
	//	nType 1 = bilinear
	//	nType 2 = bicubic

	//	CDib -> Info
	int inW, inH;
	int inD, outD, inWD8, outWD8;
	unsigned char *srcImg, *dstImg;

	inW = Width();
	inH = Height();	
	inD = BitCount()/8;

	CDib* dstCDib = new CDib();
	dstCDib->Allocate(outW, outH, BitCount());
	dstCDib->SetGrayPalette();

//	CDib* dstCDib = new CDib(outW, outH, srcCDib->BitCount(), srcCDib->GetPattern(), srcCDib->GetRgbQuadPointer());
	srcImg = GetPattern();
	dstImg = dstCDib->GetPattern();
	outD = inD;
	inWD8 = inW*inD*8;
	outWD8 = outW*outD*8;

	//	Copy from POSTECH biCubic_resize
	double m, n, am, an;
	double out1, out2, out3, out4, out;
	double sfW, sfH;
	int im, in;
	int j, i, d;	
		
	sfW = (double)outW / (double)inW;
	sfH = (double)outH / (double)inH;

	switch(nType) 
	{	
	case 1 :		//	BILINEAR
		for(j=0; j<outH; j++) 
		{
			n = (double)j * (1.0/sfH); in = (int)(n); an = (n - (double)in);
			if(in<0) in=0; else if(in>=inH-1) in = inH-2;

			for(i=0; i<outW; i++)	
			{
				// get inverse transformation & get the integer of m and n & get frantion to use interpolation
				m = (double)i * (1.0/sfW); im = (int)(m); am = (m - (double)im);			
				if(im<0) im=0; else if(im>=inW-1) im = inW-2;	

				for(d=0; d<inD; d++)
				{
					// bi-linear interpolation
					out = (1.0-an)*	
						((1.0-am)*(double)srcImg[CDib2ORD(in, im)]+am*(double)srcImg[CDib2ORD(in, im+1)]) 
						+ an*(  (1.0-am)*(double)srcImg[CDib2ORD(in+1, im)] + 
						am*(double)srcImg[CDib2ORD(in+1, im+1)]);         

					if(out < 0) out = 0; else if(out > 255) out = 255;
				
					// save the result to outImage
					dstImg[(j)*(((outWD8)+31)/32*4) + (i)*outD + d] = (int)out;
				}
			}
		}
		break;
	case 2 :		//	BICUBIC
		for(j=0; j<outH; j++) 
		{
			n = (double)j * (1.0/sfH); in = (int)(n); an = (n - (double)in);
			if(in<0) in=0; else if(in>=inH-1) in = inH-2;

			for(i=0; i<outW; i++)	
			{
				// get inverse transformation & get the integer of m and n & get frantion to use interpolation
				m = (double)i * (1.0/sfW); im = (int)(m); am = (m - (double)im);			
				if(im<0) im=0; else if(im>=inW-1) im = inW-2;				

				for(d=0; d<inD; d++)
				{
					if(im == 0 || in == 0 || im > inW-3 || in > inH-3)
					{
					// bi-linear interpolation
						out = (1.0-an)*	
							((1.0-am)*(double)srcImg[CDib2ORD(in, im)]+am*(double)srcImg[CDib2ORD(in, im+1)]) 
							+ an*(  (1.0-am)*(double)srcImg[CDib2ORD(in+1, im)] + 
							am*(double)srcImg[CDib2ORD(in+1, im+1)]);         
					}
					else 
					{
					out1 = interpolation( (double)srcImg[CDib2ORD(in-1, im-1)]
						, (double)srcImg[CDib2ORD(in-1, im)]
						, (double)srcImg[CDib2ORD(in-1, im+1)]
						, (double)srcImg[CDib2ORD(in-1, im+2)]
						, am);
					
					out2 = interpolation( (double)srcImg[CDib2ORD(in, im-1)] 
						, (double)srcImg[CDib2ORD(in, im)]
						, (double)srcImg[CDib2ORD(in, im+1)]
						, (double)srcImg[CDib2ORD(in, im+2)]
						, am);
					
					out3 = interpolation( (double)srcImg[CDib2ORD(in+1, im-1)]
						, (double)srcImg[CDib2ORD(in+1, im)]
						, (double)srcImg[CDib2ORD(in+1, im+1)]
						, (double)srcImg[CDib2ORD(in+1, im+2)]                      
						, am);
					
					out4 = interpolation( (double)srcImg[CDib2ORD(in+2, im-1)]
						, (double)srcImg[CDib2ORD(in+2, im)]
						, (double)srcImg[CDib2ORD(in+2, im+1)]
						, (double)srcImg[CDib2ORD(in+2, im+2)]
						, am);					
					
					out = interpolation(out1, out2, out3, out4, an);
					}

					if(out < 0) out = 0; else if(out > 255) out = 255;
				
					// save the result to outImage
					dstImg[(j)*(((outWD8)+31)/32*4) + (i)*outD + d] = (int)out;
				}
			}
		}
		break;
	case 0 :		//	NEAREST NEIGHBOR
	default:
		for(j=0; j<outH; j++) 
		{
			// get inverse transformation & get the integer of m and n
			n = (double)j * (1.0/sfH); in = (int)(n);
			if(in<0) in=0; else if(in>=inH-1) in = inH-2;

			for(i=0; i<outW; i++)	
			{
				// get inverse transformation & get the integer of m and n
				m = (double)i * (1.0/sfW); im = (int)(m); 
				if(im<0) im=0; else if(im>=inW-1) im = inW-2;				

				for(d=0; d<inD; d++)
				{
					out = srcImg[CDib2ORD(in, im)];

					if(out < 0) out = 0; else if(out > 255) out = 255;
					
					dstImg[(j)*(((outWD8)+31)/32*4) + (i)*outD + d] = (int)out;
				}
			}
		}
	}

	return dstCDib;
}

CDib *CDib::AbsDiff(CDib *pDib1, CDib *pDib2)
{
    if ((pDib1->Width()!=pDib2->Width()) ||
        (pDib1->Height()!=pDib2->Height()) ||
        (pDib1->BitCount()!=pDib2->BitCount())) return NULL;

    CDib *pRes = pDib1->CopyCDib();
    for (int i=0; i<pDib1->Height() ;i++) {
      unsigned char *p1 = pDib1->GetPointer(0, i);
      unsigned char *p2 = pDib2->GetPointer(0, i);
      unsigned char *pO = pRes->GetPointer(0, i);
      for (int j=0; j<pDib1->ByteWidth() ;j++)
        *pO++ = abs(*p1++ - *p2++);
    }

    return pRes;
}

CDib *CDib::AbsDiffBin(CDib *pDib1, CDib *pDib2, int threshold)
{
	CDib *pDiff = AbsDiff(pDib1, pDib2);
	CDib *pRes = pDiff->GlobalBinFixed(threshold);
	delete pDiff;

	return pRes;
}

CDib *CDib::AND(CDib *pDib1, CDib *pDib2)
{
    if ((pDib1->Width()!=pDib2->Width()) ||
        (pDib1->Height()!=pDib2->Height()) ||
        (pDib1->BitCount()!=pDib2->BitCount())) return NULL;

    CDib *pRes = pDib1->CopyCDib();
    for (int i=0; i<pDib1->Height() ;i++) {
      unsigned char *p1 = pDib1->GetPointer(0, i);
      unsigned char *p2 = pDib2->GetPointer(0, i);
      unsigned char *pO = pRes->GetPointer(0, i);
      for (int j=0; j<pDib1->ByteWidth() ;j++)
        *pO++ = *p1++ & *p2++;
    }

    return pRes;
}

CDib *CDib::OR(CDib *pDib1, CDib *pDib2)
{
    if ((pDib1->Width()!=pDib2->Width()) ||
        (pDib1->Height()!=pDib2->Height()) ||
        (pDib1->BitCount()!=pDib2->BitCount())) return NULL;

    CDib *pRes = pDib1->CopyCDib();
    for (int i=0; i<pDib1->Height() ;i++) {
      unsigned char *p1 = pDib1->GetPointer(0, i);
      unsigned char *p2 = pDib2->GetPointer(0, i);
      unsigned char *pO = pRes->GetPointer(0, i);
      for (int j=0; j<pDib1->ByteWidth() ;j++)
        *pO++ = *p1++ | *p2++;
    }

    return pRes;
}

CDib *CDib::XOR(CDib *pDib1, CDib *pDib2)
{
    if ((pDib1->Width()!=pDib2->Width()) ||
        (pDib1->Height()!=pDib2->Height()) ||
        (pDib1->BitCount()!=pDib2->BitCount())) return NULL;

    CDib *pRes = pDib1->CopyCDib();
    for (int i=0; i<pDib1->Height() ;i++) {
      unsigned char *p1 = pDib1->GetPointer(0, i);
      unsigned char *p2 = pDib2->GetPointer(0, i);
      unsigned char *pO = pRes->GetPointer(0, i);
      for (int j=0; j<pDib1->ByteWidth() ;j++)
        *pO++ = *p1++ ^ *p2++;
    }

    return pRes;
}


//////////////////////////////////////////////////////////////////////////////////////
//
// Gray Image Operations
//
//////////////////////////////////////////////////////////////////////////////////////
CIPHistogram *CDib::GetHistogram()
{
        CIPHistogram *pHist = new CIPHistogram(256);
        pHist->ResetAll();

        for (int i=0; i<Height() ;i++) {
          unsigned char *pIn = GetPointer(0, i);
          for (int j=0; j<Width() ;j++)
            pHist->Increment(*pIn++);
        }
        
        return pHist;
}

CDib *CDib::HistogramEqualizationG()
{
        CDib *pRes = new CDib;
        pRes->Allocate(Width(), Height(), 8);
        pRes->ResetContents();
        pRes->SetGrayPalette();

        CIPHistogram *pHist = GetHistogram();
        int *pHSum = new int [256];
        unsigned char *pNewLevel = new unsigned char [256];
        memset(pHSum, 0, sizeof(int)*256);

        // calculate normalized sum of histogram
        int sum = 0;
        double scale_factor = 255./(Width()*Height());     // number of pixels
        int i;
        for (i=0; i<256 ;i++) {
          sum += pHist->GetValue(i);
          pHSum[i] = (int) ((sum * scale_factor) + 0.5);
        }

        // Level Change
        for (i=0; i<Height() ;i++) {
          unsigned char *pIn = GetPointer(0, i);
          unsigned char *pOut = pRes->GetPointer(0, i);
          for (int j=0; j<Width() ;j++)
            *pOut++ = pHSum[*pIn++];
        }

        delete [] pNewLevel;
        delete [] pHSum;
        delete pHist;

        return pRes;
}

CDib *CDib::HistogramEqualization()
{
    if (IsGrayImage()) return HistogramEqualizationG();
    if (BitCount()<16) {
#ifdef _DEBUG
      AfxMessageBox("??? Equalization is possible only for Gray Image or Full Color Image");
#endif
      return NULL;
    }

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->HistogramEqualizationG();

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

CDib *CDib::HistogramExpansionG()
{
        CDib *pRes = new CDib;
        pRes->Allocate(Width(), Height(), 8);
        pRes->ResetContents();
        pRes->SetGrayPalette();

        CIPHistogram *pHist = GetHistogram();
        unsigned char *pNewLevel = new unsigned char [256];
        int nStart, nFinal;
        int i;
        for (i=0; i<256 ;i++)
          if (pHist->GetValue(i)!=0) {
            nStart = i;
            break;
          }
        for (i=255; i>=0 ;i--)
          if (pHist->GetValue(i)!=0) {
            nFinal = i;
            break;
          }

        double p;
        for (i=0; i<256 ;i++) {
          p = 255./(double)(nFinal-nStart)*(double) (i-nStart) + 0.5;
          if (p<0.) p = 0.;
          if (p>255.) p = 255.;
          pNewLevel[i] = (unsigned char) p;
        }

        // Level Change
        for (i=0; i<Height() ;i++) {
          unsigned char *pIn = GetPointer(0, i);
          unsigned char *pOut = pRes->GetPointer(0, i);
          for (int j=0; j<Width() ;j++)
            *pOut++ = pNewLevel[*pIn++];
        }

        delete [] pNewLevel;

        return pRes;
}

CDib *CDib::HistogramExpansion()
{
    if (IsGrayImage()) return HistogramExpansionG();
    if (BitCount()<16) {
#ifdef _DEBUG
      AfxMessageBox("??? Histogram Expansion is possible only for Gray Image or Full Color Image");
#endif
      return NULL;
    }

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->HistogramExpansionG();

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

CDib *CDib::BrightnessG(int brightness)
{
	CDib *pRes = CopyCDib();
	unsigned char *bImgInBuf = GetPattern();
	unsigned char *bImgOutBuf = GetPattern();
	int width = Width();
	int height = Height();

	////////////////////////////////////////////////
	int x,y;
	int temp;
	
    for (y=0; y<height ;y++) {
      unsigned char *pIn = GetPointer(0, y);
      unsigned char *pOut = pRes->GetPointer(0, y);
      for (x=0; x<width ;x++) {
        temp = *pIn++ + brightness;
        temp = min(temp, 255);
        temp = max(temp, 0);
        *pOut++ = (unsigned char) temp;
	  }
    }

	return pRes;
}

CDib *CDib::Brightness(int brightness)
{
	if (IsGrayImage()) return BrightnessG(brightness);

    if (BitCount()<16) {
#ifdef _DEBUG
      AfxMessageBox("??? Histogram Expansion is possible only for Gray Image or Full Color Image");
#endif
      return NULL;
    }

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->BrightnessG(brightness);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;
/*
	CDib *pRes = CopyCDib();
	int value;
	for (int i=0; i<pDib->Height() ;i++) {
	  unsigned char *pI = GetPointer(0, i);
	  unsigned char *pO = pRes->GetPointer(0, i);
	  for (int j=0; j<ByteWidth() ;j++) {
	    value = (int) *pI++ + brightness;
		if (value>255) value = 255;
		else if (value<0) value = 0;
		*pO++ = (unsigned char) value;
	  }
	}
*/
    return pRes;
}

CDib *CDib::ContrastG(double contrast)
{
	CDib *pRes = CopyCDib();
	unsigned char *bImgInBuf = GetPattern();
	unsigned char *bImgOutBuf = pRes->GetPattern();
	int width = Width();
	int height = Height();

	////////////////////////////////////////////////
	int x,y;
	long int sum;
	double avg;

	sum = 0;
	
    for (y=0; y<height ;y++) {
      unsigned char *pIn = GetPointer(0, y);
      for (int x=0; x<width ;x++)
        sum += *pIn++;
    }
	avg = (double)sum / (double) width / (double) height;

	int temp;
    for (y=0; y<height ;y++) {
      unsigned char *pIn = GetPointer(0, y);
      unsigned char *pOut = pRes->GetPointer(0, y);
      for (x=0; x<width ;x++) {
        temp = (int)(contrast * (double) (*pIn++ - avg ) + avg);
        temp = min(temp, 255);
        temp = max(temp, 0);
        *pOut++ = (unsigned char) temp;
      }
    }

	return pRes;
}

CDib *CDib::Contrast(double contrast)
{
	if (IsGrayImage()) return ContrastG(contrast);

    if (BitCount()<16) {
#ifdef _DEBUG
      AfxMessageBox("??? Histogram Expansion is possible only for Gray Image or Full Color Image");
#endif
      return NULL;
    }

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->ContrastG(contrast);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

	return pRes;
}

CDib *CDib::GammaCorrectionG(double gamma)
{
	CDib *pRes = CopyCDib();
	unsigned char *bImgInBuf = GetPattern();
	unsigned char *bImgOutBuf = pRes->GetPattern();
	int width = Width();
	int height = Height();

	////////////////////////////////////////////////
	int x,y;
	double min_v = 10000000;
	double max_v = -10000000;
	double temp;

	for(  y=0; y< height; y++ ) {
      unsigned char *pIn = GetPointer(0, y);
      unsigned char *pOut = pRes->GetPointer(0, y);
      for( x=0; x<width; x++ ) {
		temp = pow((double)(*pIn++) , (1./gamma));
        temp = min(temp, max_v);
        temp = max(temp, min_v);

		*pOut++ = (unsigned char) ( (temp - min_v)/ (max_v - min_v) * 255 );
	  }
    }

	return pRes;
}

CDib *CDib::GammaCorrection(double gamma)
{
	if (IsGrayImage()) return GammaCorrectionG(gamma);

    if (BitCount()<16) {
#ifdef _DEBUG
      AfxMessageBox("??? Gamma Adjustment is possible only for Gray Image or Full Color Image");
#endif
      return NULL;
    }

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->GammaCorrectionG(gamma);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

	return pRes;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Edge Operations
//
//////////////////////////////////////////////////////////////////////////////////////
CWDib *CDib::Edge_MagKirschG()
{
	int width = Width();
	int height = Height();

	CWDib *pWDib = new CWDib(width, height);
    pWDib->ResetContents();

	// Edge Magnitude Kirsch : (1,1) point is a center point.
	/*
	K0           K1           K2           K3
	-3 -3  5  &  -3  5  5  &   5  5  5  &   5  5 -3 
	-3  0  5     -3  0  5     -3  0 -3      5  0 -3
	-3 -3  5     -3 -3 -3     -3 -3 -3     -3 -3 -3
    K4           K5           K6           K7
  	 5 -3 -3  &  -3 -3 -3  &  -3 -3 -3  &  -3 -3 -3 
	 5  0 -3      5  0 -3     -3  0 -3     -3  0  5
	 5 -3 -3      5  5 -3      5  5  5     -3  5  5
    */
	int sum0, sum1, sum2, sum3, sum4, sum5, sum6, sum7;  
	int max_sum;
	pWDib->ResetContents();
	for (int i=1; i<(Height()-1) ;i++) {
	  unsigned char *pU=GetPointer(1, i-1);
	  unsigned char *pC=GetPointer(1, i);
	  unsigned char *pD=GetPointer(1, i+1);
	  short *pOut = pWDib->GetPointer(1, i);
	  for (int j=1; j<(Width()-1) ;j++, pU++, pC++, pD++) {
		sum0 = -3*(*(pU-1)) -3*(*pU) +5*(*(pU+1))
		       -3*(*(pC-1)) +5*(*(pC+1))
			   -3*(*(pD-1)) -3*(*pD) +5*(*(pD+1));
		sum1 = -3*(*(pU-1)) +5*(*pU) +5*(*(pU+1))
		       -3*(*(pC-1)) +5*(*(pC+1))
			   -3*(*(pD-1)) -3*(*pD) -3*(*(pD+1));
		sum2 =  5*(*(pU-1)) +5*(*pU) +5*(*(pU+1))
		       -3*(*(pC-1)) -3*(*(pC+1))
			   -3*(*(pD-1)) -3*(*pD) -3*(*(pD+1));
		sum3 =  5*(*(pU-1)) +5*(*pU) -3*(*(pU+1))
		       +5*(*(pC-1)) -3*(*(pC+1))
			   -3*(*(pD-1)) -3*(*pD) -3*(*(pD+1));
		sum4 =  5*(*(pU-1)) -3*(*pU) -3*(*(pU+1))
		       +5*(*(pC-1)) -3*(*(pC+1))
			   +5*(*(pD-1)) -3*(*pD) -3*(*(pD+1));
		sum5 = -3*(*(pU-1)) -3*(*pU) -3*(*(pU+1))
		       +5*(*(pC-1)) -3*(*(pC+1))
			   +5*(*(pD-1)) +5*(*pD) -3*(*(pD+1));
		sum6 = -3*(*(pU-1)) -3*(*pU) -3*(*(pU+1))
		       -3*(*(pC-1)) -3*(*(pC+1))
			   +5*(*(pD-1)) +5*(*pD) +5*(*(pD+1));
		sum7 = -3*(*(pU-1)) -3*(*pU) -3*(*(pU+1))
		       -3*(*(pC-1)) +5*(*(pC+1))
			   -3*(*(pD-1)) +5*(*pD) +5*(*(pD+1));

		max_sum = 
				max (max (ABSMAX(sum0, sum1), ABSMAX(sum2, sum3)), max (ABSMAX(sum4, sum5), ABSMAX(sum6, sum7)));

		*pOut++ = max_sum;
	  }
	}
/*
	const int M_KirschK0[3][3] = {{-3, -3, 5}, {-3, 0, 5}, {-3, -3, 5}};
	const int M_KirschK1[3][3] = {{-3, 5, 5}, {-3, 0, 5}, {-3, -3, -3}};
	const int M_KirschK2[3][3] = {{5, 5, 5}, {-3, 0, -3}, {-3, -3, -3}};
	const int M_KirschK3[3][3] = {{5, 5, -3}, {5, 0, -3}, {-3, -3, -3}};
	const int M_KirschK4[3][3] = {{5, -3, -3}, {5, 0, -3}, {5, -3, -3}};
	const int M_KirschK5[3][3] = {{-3, -3, -3}, {5, 0, -3}, {5, 5, -3}};
	const int M_KirschK6[3][3] = {{-3, -3, -3}, {-3, 0, -3}, {5, 5, 5}};
	const int M_KirschK7[3][3] = {{-3, -3, -3}, {-3, 0, 5}, {-3, 5, 5}};

	int sum, sum0, sum1, sum2, sum3, sum4, sum5, sum6, sum7;  
	int max_sum;
	for (int j=0, i; j<height; j++)
	{
		for(i=0; i<width; i++)
		{
			sum = sum0 = sum1 = sum2 = sum3 = sum4 = sum5 = sum6 = sum7 = 0;
			
			// first mask
			sum0 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_KirschK0[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_KirschK0[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_KirschK0[0][2]
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_KirschK0[1][0]
				+ 0
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_KirschK0[1][2]
				
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_KirschK0[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_KirschK0[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_KirschK0[2][2];
			// second mask
			sum1 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_KirschK1[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_KirschK1[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_KirschK1[0][2]
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_KirschK1[1][0]
				+ 0
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_KirschK1[1][2]
				
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_KirschK1[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_KirschK1[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_KirschK1[2][2];
			// third mask
			sum2 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_KirschK2[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_KirschK2[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_KirschK2[0][2]
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_KirschK2[1][0]
				+ 0
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_KirschK2[1][2]
				
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_KirschK2[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_KirschK2[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_KirschK2[2][2];
			// 4th mask
			sum3 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_KirschK3[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_KirschK3[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_KirschK3[0][2]
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_KirschK3[1][0]
				+ 0
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_KirschK3[1][2]
				
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_KirschK3[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_KirschK3[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_KirschK3[2][2];
			// 5th mask
			sum4 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_KirschK4[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_KirschK4[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_KirschK4[0][2]
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_KirschK4[1][0]
				+ 0
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_KirschK4[1][2]
				
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_KirschK4[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_KirschK4[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_KirschK4[2][2];
			// 6th mask
			sum5 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_KirschK5[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_KirschK5[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_KirschK5[0][2]
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_KirschK5[1][0]
				+ 0
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_KirschK5[1][2]
				
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_KirschK5[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_KirschK5[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_KirschK5[2][2];
			// 7th mask
			sum6 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_KirschK6[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_KirschK6[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_KirschK6[0][2]
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_KirschK6[1][0]
				+ 0
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_KirschK6[1][2]
				
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_KirschK6[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_KirschK6[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_KirschK6[2][2];
			// 8th mask
			sum7 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_KirschK7[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_KirschK7[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_KirschK7[0][2]
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_KirschK7[1][0]
				+ 0
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_KirschK7[1][2]
				
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_KirschK7[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_KirschK7[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_KirschK7[2][2];

			// get maximum value
			
			
			// first method, only maximun value
			
			max_sum = 0;
			int max_sum2th = max_sum;

			max_sum = 
				max (max (ABSMAX(sum0, sum1), ABSMAX(sum2, sum3)), max (ABSMAX(sum4, sum5), ABSMAX(sum6, sum7)));
			sum = max_sum;
			
			bImgOutBuf[j*width + i] = sum;
		}
		
	}
*/
	return pWDib;
}


CWDib *CDib::Edge_MagKirsch()
{
	if (IsGrayImage()) return Edge_MagKirschG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CWDib *pRes = Edge_MagKirschG();
	delete pGray;

	return pRes;
}

CWDib *CDib::Edge_MagLaplacianG()
{
	int width = Width();
	int height = Height();

	CWDib *pWDib = new CWDib(width, height);
    pWDib->ResetContents();

	// Mask Laplacian : (1,1) point is a center point.
	/*
	 0 -1  0
	-1  4 -1 
	 0 -1  0
    */
	int sum;
	pWDib->ResetContents();
	for (int i=1; i<(Height()-1) ;i++) {
	  unsigned char *pU=GetPointer(1, i-1);
	  unsigned char *pC=GetPointer(1, i);
	  unsigned char *pD=GetPointer(1, i+1);
	  short *pOut = pWDib->GetPointer(1, i);
	  for (int j=1; j<(Width()-1) ;j++, pU++, pC++, pD++) {
	    sum = -(*pU) -(*(pC-1)) -(*(pC+1)) -(*pD)
		       + 4*(*pC);
		*pOut++ = abs(sum);
	  }
	}
/*
	const int M_Laplacian[3][3] = {{0, -1, 0}, {-1, 4, -1}, {0, -1, 0}};
	int sum;  
	for (int j=0, i; j<height; j++)
	{
		for(i=0; i<width; i++)
		{
			sum = 0;
			sum = 0
				+ bImgInBuf[ ((int) abs(j-1) )*width + i] * M_Laplacian[0][1]
				+ 0
				
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_Laplacian[1][0]
				+ bImgInBuf[j*width + i] * M_Laplacian[1][1]
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_Laplacian[1][2]
				
				+ 0
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_Laplacian[2][1]
				+ 0;
		
			wImgOutBuf[j*width + i] = abs(sum);
		}
		
	}
*/
	return pWDib;
}


CWDib *CDib::Edge_MagLaplacian()
{
	if (IsGrayImage()) return Edge_MagLaplacianG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CWDib *pRes = pGray->Edge_MagLaplacianG();
	delete pGray;

	return pRes;
}

CWDib *CDib::Edge_MagPrewittG()
{
	int width = Width();
	int height = Height();

	CWDib *pWDib = new CWDib(width, height);
    pWDib->ResetContents();

	// Edge Magnitude Prewit : (1,1) point is a center point.
	/*
	-1 -1 -1  &  -1  0  1
	 0  0  0     -1  0  1
	 1  1  1     -1  0  1
    */
	int sum1, sum2;
	pWDib->ResetContents();
	for (int i=1; i<(Height()-1) ;i++) {
	  unsigned char *pU=GetPointer(1, i-1);
	  unsigned char *pC=GetPointer(1, i);
	  unsigned char *pD=GetPointer(1, i+1);
	  short *pOut = pWDib->GetPointer(1, i);
	  for (int j=1; j<(Width()-1) ;j++, pU++, pC++, pD++) {
	    sum1 = -(*(pU-1)) -(*pU) -(*(pU+1)) 
		       +(*(pD-1)) +(*pD) +(*(pD+1));
		sum2 = -(*(pU-1)) -(*(pC-1)) - (*(pD-1))
			   +(*(pU+1)) +(*(pC-1)) + (*(pD-1));
		*pOut++ = (short) sqrt((double) (sum1*sum1+sum2*sum2));
	  }
	}
/*
	const int M_PrewitX[3][3] = {{-1, -1, -1}, {0, 0, 0}, {1, 1, 1}};
	const int M_PrewitY[3][3] = {{-1, 0, 1}, {-1, 0, 1}, {-1, 0, 1}};
	int sum, sum1, sum2;  
	for (int j=0, i; j<height; j++)
	{
		for(i=0; i<width; i++)
		{
			sum = sum1 = sum2 = 0;
			// first mask : M_PrewitX[3][3] = {{-1, -1, -1}, {0, 0, 0}, {1, 1, 1}}
			sum1 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_PrewitX[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  i ] * M_PrewitX[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_PrewitX[0][2]
				+ 0 + 0 + 0
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1)] * M_PrewitX[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_PrewitX[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_PrewitX[2][2];
			// second mask : M_PrewitY[3][3] = {{-1, 0, 1}, {-1, 0, 1}, {-1, 0, 1}}
			sum2 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_PrewitY[0][0]
				+ 0
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_PrewitY[0][2]
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_PrewitY[1][0]
				+ 0 
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_PrewitY[1][2]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1)] * M_PrewitY[2][0]
				+ 0
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_PrewitY[2][2];

			sum = (int) sqrt((sum1*sum1 + sum2*sum2));
		
			wImgOutBuf[j*width + i] = sum;
		}
	}
*/
	return pWDib;
}

CWDib *CDib::Edge_MagPrewitt()
{
	if (IsGrayImage()) return Edge_MagPrewittG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CWDib *pRes = pGray->Edge_MagPrewittG();
	delete pGray;

	return pRes;
}

CWDib *CDib::Edge_MagRobertG()
{
	int width = Width();
	int height = Height();

	CWDib *pWDib = new CWDib(width, height);
    pWDib->ResetContents();
    
    // Edge magnitude Robert : (2,1) point is a center point.
	/*
	 1  0  &  0  1
	 0 -1    -1  0
    */
	int sum, sum1, sum2;
    
    for (int i=0; i<(Height()-1) ;i++) {
      unsigned char *pC = GetPointer(0, i);
      unsigned char *pN = GetPointer(0, i+1);
      short *pO = pWDib->GetPointer(0, i);
      for (int j=0; j<(Width()-1) ;j++, pC++, pN++) {
        sum1 = *pC - *(pN+1);
        sum2 = *(pC+1) - *pN;
        sum = (short) sqrt((double) (sum1*sum1 + sum2*sum2));
        *pO++ = sum;
      }
    }

/*      
	for (int j=0, i; j<height; j++)
	{
		for(i=0; i<width; i++)
		{
			sum = sum1 = sum2 = 0;
			// first mask : M_RovertD1[2][2] = {{1, 0}, {0, -1}}
			sum1 = bImgInBuf[j*width +  (int) abs(i-1)] * M_RovertD1[0][0] + 0 
				 + 0 + bImgInBuf[(MIRROR((j+1), height) )*width + i] * M_RovertD1[1][1];
			// second mask : M_RovertD2[2][2] = {{0, 1}, {-1, 0}}
			sum2 = 0 + bImgInBuf[j*width + i] * M_RovertD2[0][1]   
				  + bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1) ] * M_RovertD2[1][0] + 0;

			sum = (int) sqrt((sum1*sum1 + sum2*sum2));
		
			wImgOutBuf[j*width + i] = sum;
		}
	}
*/
	return pWDib;
}


CWDib *CDib::Edge_MagRobert()
{
	if (IsGrayImage()) return Edge_MagRobertG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CWDib *pRes = pGray->Edge_MagRobertG();
	delete pGray;

	return pRes;
}

CWDib *CDib::Edge_MagSobelG()
{
	int width = Width();
	int height = Height();

	CWDib *pWDib = new CWDib(width, height);
    pWDib->ResetContents();

// Edge Magnitude Sobel : (1,1) point is a center point.
	/*
	-1 -2 -1  &  -1  0  1
	 0  0  0     -2  0  2
	 1  2  1     -1  0  1
    */
	int sum1, sum2;
	pWDib->ResetContents();
	for (int i=1; i<(Height()-1) ;i++) {
	  unsigned char *pU=GetPointer(1, i-1);
	  unsigned char *pC=GetPointer(1, i);
	  unsigned char *pD=GetPointer(1, i+1);
	  short *pOut = pWDib->GetPointer(1, i);
	  for (int j=1; j<(Width()-1) ;j++, pU++, pC++, pD++) {
	    sum1 = -(*(pU-1)) -2*(*pU) -(*(pU+1)) 
		       +(*(pD-1)) +2*(*pD) +(*(pD+1));
		sum2 = -(*(pU-1)) -2*(*(pC-1)) - (*(pD-1))
			   +(*(pU+1)) +2*(*(pC+1)) + (*(pD+1));
		*pOut++ = (short) sqrt((double) (sum1*sum1+sum2*sum2));
	  }
	}

/*
	const int M_SobelX[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
	const int M_SobelY[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};

	int sum, sum1, sum2;  

	for (int j=0; j<height; j++)
	{
		for(int i=0; i<width; i++)
		{
			// first mask : M_SobelX[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}}
			sum1 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_SobelX[0][0]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  i ] * M_SobelX[0][1]
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_SobelX[0][2]
				//+ 0 + 0 + 0
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1)] * M_SobelX[2][0]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width + i] * M_SobelX[2][1]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_SobelX[2][2];
			// second mask : M_SobelY[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}}
			sum2 = bImgInBuf[ ((int) abs(j-1) )*width +  (int) abs(i-1) ] * M_SobelY[0][0]
				//+ 0
				+ bImgInBuf[ ((int) abs(j-1) )*width +  MIRROR ((i+1), width) ] * M_SobelY[0][2]
				+ bImgInBuf[j*width +  (int) abs(i-1) ] * M_SobelY[1][0]
				//+ 0 
				+ bImgInBuf[j*width +  MIRROR ((i+1), width) ] * M_SobelY[1][2]
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  (int) abs(i-1)] * M_SobelY[2][0]
				//+ 0
				+ bImgInBuf[ (MIRROR((j+1), height) )*width +  MIRROR ((i+1), width) ] * M_SobelY[2][2];

			sum = (int) sqrt(sum1*sum1 + sum2*sum2);
		
			wImgOutBuf[j*width + i] = (short) sum;
		}
	
	}
*/
	return pWDib;
}

CWDib *CDib::Edge_MagSobel()
{
	if (IsGrayImage()) return Edge_MagSobelG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CWDib *pRes = pGray->Edge_MagSobel();
	delete pGray;

	return pRes;
}

CDib *CDib::Edge_AdaptiveSobelG(int divide)
{
        CDib *pRes = new CDib;
        pRes->Allocate(Width(), Height(), 8);
        pRes->ResetContents();
        pRes->SetGrayPalette();

        int h, v, mag, sum;
        for (int y=1; y<Height()-1 ;y++) {
          unsigned char *pUpper = GetPointer(0, y-1);
          unsigned char *pCurr = GetPointer(0, y);
          unsigned char *pLower = GetPointer(0, y+1); 
          unsigned char *pOut = pRes->GetPointer(1, y);
          for (int x=1; x<Width()-1 ;x++, pUpper++, pCurr++, pLower++, pOut++) {
            h = ((int) *(pUpper) + 2 * (int) *(pUpper+1) + (int) *(pUpper+2))/4
              - ((int) *(pLower) + 2 * (int) *(pLower+1) + (int) *(pLower+2))/4;
            v = ((int) *(pUpper) + 2 * (int) *(pCurr) + (int) *(pLower))/4
              - ((int) *(pUpper+2) + 2 * (int) *(pCurr+2) + (int) *(pLower+2))/4;
            sum = (int) *(pUpper) + 2 * (int) *(pUpper+1) + (int) *(pUpper+2)
                + 2 * (int) *(pCurr) + 4 * (int) *(pCurr+1) + 2 * (int) *(pCurr+2)
                + (int) *(pLower) + 2 * (int) *(pLower+1) + (int) *(pLower+2);
            mag = abs(h) + abs(v);
            sum /= divide;
            if (sum==0) sum = 1;
            if ((mag/sum)>1) *pOut = 255;
            else *pOut = 0;
          }
        }

        return pRes;
}

CDib *CDib::Edge_AdaptiveSobel(int divide)
{
	if (IsGrayImage()) return Edge_AdaptiveSobelG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->Edge_AdaptiveSobelG();
	delete pGray;

	return pRes;
}

CDib *CDib::EdgeB_MagKirschG()
{
	CWDib *pWDib = Edge_MagKirschG();
	CDib *pRes = (CDib *) pWDib->GetMappedCDib(1., 0.);
	delete pWDib;

	return pRes;
}

CDib *CDib::EdgeB_MagKirsch()
{
	if (IsGrayImage()) return EdgeB_MagKirschG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->EdgeB_MagKirschG();
	delete pGray;

	return pRes;
}

CDib *CDib::EdgeB_MagLaplacianG()
{
	CWDib *pWDib = Edge_MagLaplacianG();
	CDib *pRes = (CDib *) pWDib->GetMappedCDib(1., 0.);
	delete pWDib;

	return pRes;
}

CDib *CDib::EdgeB_MagLaplacian()
{
	if (IsGrayImage()) return EdgeB_MagLaplacianG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->EdgeB_MagLaplacianG();
	delete pGray;

	return pRes;
}

CDib *CDib::EdgeB_MagPrewittG()
{
	CWDib *pWDib = Edge_MagPrewittG();
	CDib *pRes = (CDib *) pWDib->GetMappedCDib(1., 0.);
	delete pWDib;

	return pRes;
}

CDib *CDib::EdgeB_MagPrewitt()
{
	if (IsGrayImage()) return EdgeB_MagPrewittG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->EdgeB_MagPrewittG();
	delete pGray;

	return pRes;
}

CDib *CDib::EdgeB_MagRobertG()
{
	CWDib *pWDib = Edge_MagRobertG();
	CDib *pRes = (CDib *) pWDib->GetMappedCDib(1., 0.);
	delete pWDib;

	return pRes;
}

CDib *CDib::EdgeB_MagRobert()
{
	if (IsGrayImage()) return EdgeB_MagRobertG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->EdgeB_MagRobert();
	delete pGray;

	return pRes;
}

CDib *CDib::EdgeB_MagSobelG()
{
	CWDib *pWDib = Edge_MagSobelG();
	CDib *pRes = (CDib *) pWDib->GetMappedCDib(1., 0.);
	delete pWDib;

	return pRes;
}

CDib *CDib::EdgeB_MagSobel()
{
	if (IsGrayImage()) return EdgeB_MagSobelG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->EdgeB_MagSobel();
	delete pGray;

	return pRes;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Binarization
//
//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/*
 * Otsu's thresholding algorithm as described in 
 * Haralick and Shapiro, Computer and Robot Vision, Vol. 1, p. 22.
 *  int* Gray_hist 히스토그램, 
 *  int size 히스토그램 배열의 크기 
 *  int Npix 히스토그램의 총합, 즉 영상의 전체 픽셀의 개수 
 */
void CDib::comp_between_group_var(double* Gray_prob,double* BG_var)
{
    register int t, start, end;
    double u, u1, u2, q1, q1_new;

    u = 0;
    for (t=0; t<256; t++)
	u += t*Gray_prob[t];
    start = 0;
    while (Gray_prob[start]==0.0)
	BG_var[start++] = 0;
    end = 255;
    while (Gray_prob[end]==0.0) {
	BG_var[end--] = 0;
    }
    BG_var[end] = 0;
    u1 = 0;
    q1 = 0;
    for (t=start; t<end; t++) {
	q1_new = q1 + Gray_prob[t];
	u1 = (q1*u1+t*Gray_prob[t])/q1_new;
	q1 = q1_new;
	u2 = (u-q1*u1)/(1-q1);
	BG_var[t] = q1*(1-q1)*(u1-u2)*(u1-u2);
    }
}

int CDib::otsu(int* Gray_hist,int size,int Npix, double *pMaxVariance ) 
{
	double *Gray_prob,*BG_var; 
	
	Gray_prob = new double[size];
	BG_var= new double[size];
    
        int i, thresh;
        double max = 0.0;

        // compute gray probabilities
        for (i=0; i<size; i++)
	  Gray_prob[i] = (float) Gray_hist[i]/Npix;

       comp_between_group_var(Gray_prob, BG_var);

        // find the threshold value with max variance
        for (i=0; i<size; i++) {
	  if (BG_var[i] > max) {
	    max = BG_var[i];
	    thresh = i;
	  }
        }

        if (pMaxVariance) *pMaxVariance = max;

	delete []Gray_prob;
	delete []BG_var;

	return thresh;
}

CDib *CDib::GlobalBinFixedG(int threshold)
{
        CDib *pRes = new CDib;
        pRes->Allocate(Width(), Height(), 8);
        pRes->ResetContents();
        pRes->SetGrayPalette();

        for (int y=0; y<Height() ;y++) {
          unsigned char *pIn = GetPointer(0, y);
          unsigned char *pOut = pRes->GetPointer(0, y);
          for (int x=0; x<Width() ;x++, pIn++, pOut++)
            if (*pIn>=threshold) *pOut = 255;
            else *pOut = 0;
        }

        return pRes;
}

CDib *CDib::GlobalBinFixed(int threshold)
{
	if (IsGrayImage()) return GlobalBinFixedG(threshold);

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->GlobalBinFixedG(threshold);
	delete pGray;

	return pRes;
}

CDib *CDib::GlobalBinOtsuG(int shift, int *pThreshold)
{
        int *pHist = new int[256];
        CDib *pRes = new CDib;
        pRes->Allocate(Width(), Height(), 8);
        pRes->ResetContents();
        pRes->SetGrayPalette();

        // Get Histogram
        memset(pHist, 0, sizeof(int)*256);
        int x, y;
        for (y=0; y<Height() ;y++) {
          unsigned char *pChar = GetPointer(0, y);
          for (x=0; x<Width() ;x++) 
            pHist[*pChar++]++;
        }

        // Find the Threshold
        int threshold = otsu(pHist, 256, Width()*Height()) + shift;
		if (pThreshold) *pThreshold = threshold;
        
        // Thresholding
        for (y=0; y<Height() ;y++) {
          unsigned char *pIn = GetPointer(0, y);
          unsigned char *pOut = pRes->GetPointer(0, y);
          for (x=0; x<Width() ;x++, pIn++, pOut++)
            if (((int) *pIn)>=threshold) *pOut = 255;
            else *pOut = 0;
        }

        delete [] pHist;

        return pRes;
}

CDib *CDib::GlobalBinOtsu(int shift, int *pThreshold)
{
	if (IsGrayImage()) return GlobalBinOtsuG(shift, pThreshold);

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->GlobalBinOtsuG(shift, pThreshold);
	delete pGray;

	return pRes;
}

#define BLOCK_SIZE              16
#define TH_FLAT                 5
CDib *CDib::LocalBinOtsuG()
{
        int *pHist = new int[256];
        int bWidth = Width()/BLOCK_SIZE;
        int bHeight = Height()/BLOCK_SIZE;
        int i, j, x, y;
        CDib *pRes = new CDib;
        pRes->Allocate(Width(), Height(), 8);
        pRes->ResetContents();
        pRes->SetGrayPalette();

        for (y=0; y<bHeight-1 ;y++)
          for (x=0; x<bWidth-1 ;x++) {

            // Get Histogram
            int sx = x*BLOCK_SIZE;
            int sy = y*BLOCK_SIZE;
            int ex = (x+2)*BLOCK_SIZE;
            int ey = (y+2)*BLOCK_SIZE;
            memset(pHist, 0, sizeof(int)*256);
            for (i=sy; i<ey ;i++) {
              unsigned char *pChar = GetPointer(sx, i);
              for (j=sx; j<ex ;j++) 
                pHist[*pChar++]++;
            }

            // Find Threshold
            double max_vari;
            int threshold = otsu(pHist, 256, 4*BLOCK_SIZE*BLOCK_SIZE, &max_vari);
            //if (max_vari<10.) threshold = 999;

            // Binarization
            sx = x*BLOCK_SIZE + BLOCK_SIZE/2;
            sy = y*BLOCK_SIZE + BLOCK_SIZE/2;
            ex = sx + BLOCK_SIZE;
            ey = sy + BLOCK_SIZE;
            for (i=sy; i<ey ;i++) {
              unsigned char *pIn = GetPointer(sx, i);
              unsigned char *pOut = pRes->GetPointer(sx, i);
              for (j=sx; j<ex ;j++, pIn++, pOut++)
                if (((int) (*pIn))>=threshold) *pOut = 255;
                else *pOut = 0;
            }
          }

        delete [] pHist;
                        
        return pRes;
}

CDib *CDib::LocalBinOtsu()
{
	if (IsGrayImage()) return LocalBinOtsuG();

	CDib *pGray = NULL;
	if (m_bUseGrayFast) pGray = GetGrayCDibFast();
	else pGray = GetGrayCDib();
	CDib *pRes = pGray->LocalBinOtsuG();
	delete pGray;

	return pRes;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Filtering
//
//////////////////////////////////////////////////////////////////////////////////////
int CDib::compare_int(const void *p1, const void *p2)
{
	int v1 = *((int *) p1);
	int v2 = *((int *) p2);

	if (v1>v2) return 1;
	else if (v1==v2) return 0;
	else return -1;
}

CDib *CDib::Median3x3G()
{
	int mask[9];
	CDib *pRes = CopyCDib();
	for (int i=1; i<(Height()-1) ;i++) {
	  unsigned char *pU = GetPointer(1, i-1);
	  unsigned char *pC = GetPointer(1, i);
	  unsigned char *pD = GetPointer(1, i+1);
	  unsigned char *pO = pRes->GetPointer(1, i);
	  for (int j=1; j<(Width()-1) ;j++, pU++, pC++, pD++) {
	    mask[0] = *(pU-1); mask[1] = *pU; mask[2] = *(pU+1);
		mask[3] = *(pC-1); mask[4] = *pC; mask[5] = *(pC+1);
		mask[6] = *(pD-1); mask[7] = *pD; mask[8] = *(pD+1);
		qsort(mask, 9, sizeof(int), compare_int);
		*pO++ = mask[4];
	  }
	}

	return pRes;
}

CDib *CDib::Median3x3()
{
    if (IsGrayImage()) return Median3x3G();

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->Median3x3G();

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

CDib *CDib::Median5x5G()
{
	int mask[25];
	CDib *pRes = CopyCDib();
	for (int i=2; i<(Height()-2) ;i++) {
	  unsigned char *pUU = GetPointer(2, i-2);
	  unsigned char *pU = GetPointer(2, i-1);
	  unsigned char *pC = GetPointer(2, i);
	  unsigned char *pD = GetPointer(2, i+1);
	  unsigned char *pDD = GetPointer(2, i+2);
	  unsigned char *pO = pRes->GetPointer(2, i);
	  for (int j=2; j<(Width()-2) ;j++, pUU++, pU++, pC++, pD++, pDD++) {
	    mask[0] = *(pUU-2); mask[1] = *(pUU-1); mask[2] = *pUU; mask[3] = *(pUU+1); mask[4] = *(pUU+2);
	    mask[5] = *(pUU-2); mask[6] = *(pUU-1); mask[7] = *pUU; mask[8] = *(pUU+1); mask[9] = *(pUU+2);
	    mask[10] = *(pUU-2); mask[11] = *(pUU-1); mask[12] = *pUU; mask[13] = *(pUU+1); mask[14] = *(pUU+2);
	    mask[15] = *(pUU-2); mask[16] = *(pUU-1); mask[17] = *pUU; mask[18] = *(pUU+1); mask[19] = *(pUU+2);
	    mask[20] = *(pUU-2); mask[21] = *(pUU-1); mask[22] = *pUU; mask[23] = *(pUU+1); mask[24] = *(pUU+2);
		qsort(mask, 25, sizeof(int), compare_int);
		*pO++ = mask[12];
	  }
	}

	return pRes;
}

CDib *CDib::Median5x5()
{
    if (IsGrayImage()) return Median5x5G();

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->Median5x5G();

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

CDib *CDib::Lowpass3x3G()
{
	CDib *pRes = CopyCDib();

	int *pBuf = new int [Width()];
    int i, j;
	for (i=1; i<(Height()-1) ;i++) {
	  unsigned char *pU = GetPointer(0, i-1);
	  unsigned char *pC = GetPointer(0, i);
	  unsigned char *pD = GetPointer(0, i+1);
	  unsigned char *pO = pRes->GetPointer(1, i);
	  for (j=0; j<Width() ;j++)
	    pBuf[j] = (int) *pU++ + (int) *pC++ + (int) *pD++;
	  for (j=1; j<(Width()-1) ;j++)
	    *pO++ = (unsigned char) ((pBuf[j-1] + pBuf[j] + pBuf[j+1] + 4)/9);
	}

	delete [] pBuf;

	return pRes;
}

CDib *CDib::Lowpass3x3()
{
    if (IsGrayImage()) return Lowpass3x3G();

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->Lowpass3x3G();

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

CDib *CDib::Lowpass5x5G()
{
	CDib *pRes = CopyCDib();

	int *pBuf = new int [Width()];
    int i, j;
	for (i=2; i<(Height()-2) ;i++) {
	  unsigned char *pUU = GetPointer(0, i-2);
	  unsigned char *pU = GetPointer(0, i-1);
	  unsigned char *pC = GetPointer(0, i);
	  unsigned char *pD = GetPointer(0, i+1);
	  unsigned char *pDD = GetPointer(0, i+2);
	  unsigned char *pO = pRes->GetPointer(2, i);
	  for (j=0; j<Width() ;j++)
	    pBuf[j] = *pUU++ + *pU++ + *pC++ + *pD++ + *pDD++;
	  for (j=2; j<(Width()-2) ;j++)
	    *pO++ = (unsigned char) ((pBuf[j-2] + pBuf[j-1] + pBuf[j] + pBuf[j+1] + pBuf[j+2] + 12)/25);
	}

	delete [] pBuf;

	return pRes;
}

CDib *CDib::Lowpass5x5()
{
    if (IsGrayImage()) return Lowpass5x5G();

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->Lowpass5x5G();

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Morphology Operations
//
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Morphology Operations
CDib *CDib::DilationG(int SE_size)
// SE_size : structuring element size, only use "odd" number!
{
	/*
	 * Gray level Dilation ---------------------------------------
	 * struncturing elementsize의 mask를 사용하여
	 *   ( mask안의 영상 값 ) + ( mask 값 ) 의 최대값을 사용한다.
	 *            # ------------------------------------ #
	 *            # 영상의 외각에는 0이 있다고 생각한다. #
	 *            #   즉, 영상의 외각은 0으로 padding    #
	 *            # ------------------------------------ #
	 * 여기서 mask값은 모두 1로 정의한다.
	 */

	unsigned char *bImgInBuf = GetPattern();
	int width = Width();
    int bwidth = ByteWidth();
	int height = Height();

	CDib *pRes = new CDib;
	pRes->Allocate(width, height, 8); pRes->SetGrayPalette(); pRes->ResetContents();
	unsigned char *bImgOutBuf = pRes->GetPattern();

	int i, j, x, y;
	int nMask = SE_size/2;
	int max_pel, pel;

	// structuring element size check!
	if((SE_size%2) != 1) {
		//Structuring Element의 크기는 홀수 이어야 합니다!
		return NULL;
	}
	if((SE_size>15) || (SE_size<3)) {
		//Structuring Element 크기의 범위를 벗어 났습니다!
		return NULL;
	}

	for(y=0; y<height; y++) {
		for(x=0; x<width; x++) {
			max_pel = 0;
			for(j=0; j<SE_size; j++) {
				for(i=0; i<SE_size; i++) {
					if(((y - nMask + j) < 0) ||		// 영상의 외각일 경우, 255
					   ((x - nMask + i) < 0) ||
					   ((y - nMask + j) >= height) || 
					   ((x - nMask + i) >= width)) {
						pel = 0;
					}
					else {										// 영상 안 일 경우
						pel = bImgInBuf[(y-nMask+j)*bwidth + (x-nMask+i)];
					}

					if(max_pel<pel)
						max_pel = pel;
				}
			}
			
			if(max_pel<254)
				bImgOutBuf[y*bwidth+x] = (unsigned char)(max_pel+1);
			else
				bImgOutBuf[y*bwidth+x] = 255;
		}
	}

	return pRes;
}

CDib *CDib::ErosionG(int SE_size)
// SE_size : structuring element size, only use "odd" number!
{
	/*
	 * Gray level Erosion ---------------------------------------
	 * struncturing elementsize의 mask를 사용하여
	 *   ( mask안의 영상 값 ) - ( mask 값 ) 의 최소값을 사용한다.
	 *            # -------------------------------------- #
	 *            # 영상의 외각에는 255이 있다고 생각한다. #
	 *            #   즉, 영상의 외각은 255으로 padding    #
	 *            # -------------------------------------- #
	 * 여기서 mask값은 모두 1로 정의한다.
	 */

	unsigned char *bImgInBuf = GetPattern();
	int width = Width();
    int bwidth = ByteWidth();
	int height = Height();

	CDib *pRes = new CDib;
	pRes->Allocate(width, height, 8); pRes->SetGrayPalette(); pRes->ResetContents();
	unsigned char *bImgOutBuf = pRes->GetPattern();

	int x, y, i, j;
	int min_pel, pel;
	int nMask = SE_size/2;

	// structuring element size check!
	if((SE_size%2) != 1) {
		//Structuring Element의 크기는 홀수 이어야 합니다!
		return NULL;
	}

	if((SE_size>15) || (SE_size<3)) {
		//Structuring Element 크기의 범위를 벗어 났습니다!
		return NULL;
	}

	for(y=0; y<height; y++) {
		for(x=0; x<width; x++) {
			min_pel = 255;
			for(j=0; j<SE_size; j++) {
				for(i=0; i<SE_size; i++) {
					if(((y - nMask + j) < 0) ||
					   ((x - nMask + i) < 0) ||
					   ((y - nMask + j) >= height) || 
					   ((x - nMask + i) >= width)) {
						pel = 0;
					}
					else {						
						pel = bImgInBuf[(y-nMask+j)*bwidth + (x-nMask+i)];
					}

					if(min_pel>pel)
						min_pel = pel;
				}
			}
			
			if(min_pel>1)
				bImgOutBuf[y*bwidth+x] = (unsigned char)(min_pel-1);
			else
				bImgOutBuf[y*bwidth+x] = 0;
		}
	}

	return pRes;
}

CDib *CDib::OpeningG(int SE_size, int repeat)
{
	CDib *pDib = CopyCDib();
	int i;

	for (i=0; i<repeat ;i++) {
		CDib *pErosion = pDib->ErosionG(SE_size);
		delete pDib;
		pDib = pErosion;
	}

	for (i=0; i<repeat ;i++) {
		CDib *pDilation = pDib->DilationG(SE_size);
		delete pDib;
		pDib = pDilation;
	}

	return pDib;
}

CDib *CDib::ClosingG(int SE_size, int repeat)
{
	CDib *pDib = CopyCDib();
	int i;

	for (i=0; i<repeat ;i++) {
		CDib *pDilation = pDib->DilationG(SE_size);
		delete pDib;
		pDib = pDilation;
	}

	for (i=0; i<repeat ;i++) {
		CDib *pErosion = pDib->ErosionG(SE_size);
		delete pDib;
		pDib = pErosion;
	}

	return pDib;
}

CDib *CDib::Dilation(int SE_size)
{
    if (IsGrayImage()) return DilationG(SE_size);

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->DilationG(SE_size);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

CDib *CDib::Erosion(int SE_size)
{
    if (IsGrayImage()) return ErosionG(SE_size);

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->ErosionG(SE_size);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

CDib *CDib::Opening(int SE_size, int repeat)
{
    if (IsGrayImage()) return OpeningG(SE_size, repeat);

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->OpeningG(SE_size, repeat);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

CDib *CDib::Closing(int SE_size, int repeat)
{
    if (IsGrayImage()) return ClosingG(SE_size, repeat);

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->ClosingG(SE_size, repeat);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}

BYTE *CDib::CreateMorphologyMask(int MaskShape, int MaskLength)
{
	BYTE	*mask	=  new BYTE[(MaskLength)*(MaskLength)];
	
	int half_msk_length;
	int	R;
	int temp;
	half_msk_length	= (MaskLength)/2;
	R				= half_msk_length*half_msk_length; 
	
	if( (MaskShape) == 0) {
		for(int  y= 0; y< (MaskLength); y++ ) {
		  for(int x=0; x< (MaskLength); x++) {
			temp = (y-half_msk_length)*(y-half_msk_length)+(x-half_msk_length)*(x-half_msk_length);
			*( mask+y * (MaskLength) + x) =  ( temp > R ) ? 0 : 255;
		  }
		}
	}
	else if((MaskShape) == 1) {
		for( int y= 0; y< (MaskLength); y++ ) 
		  for(int x=0; x<(MaskLength); x++) 
			*(mask+y * (MaskLength) + x)	=255;
	}
	else {
	  delete [] mask;
	  return NULL;
	}

	return mask;
}

CDib *CDib::DilationB(unsigned char *mask, int MaskLength)
{
	unsigned char *bImgInBuf = GetPattern();
	int width = Width();
    int bwidth = ByteWidth();
	int height = Height();
	CDib *pRes = new CDib; pRes->Allocate(width, height, 8); pRes->SetGrayPalette(); pRes->ResetContents();
	unsigned char *bImgOutBuf = pRes->GetPattern();

	int x, y, i, j, smax;
	int N,M;  
	N = M = MaskLength;
	for( y=N/2; y< height-N/2; y++) {
		for( x=N/2; x<width-N/2;x++) {
			smax=0;
			for( j=-N/2; j<=N/2; j++) {
				for( i=-N/2; i<=N/2; i++) {
					if( *(mask+(N/2+i)*N + (j + N/2) ) ==255) {
						if( bImgInBuf[x+i+(long)(y+j) * bwidth] >smax) {
							smax  = bImgInBuf[x+i+(long)(y+j) * bwidth];
						}
					}				
				}
			}
			bImgOutBuf[x+(long)y * bwidth] = smax;
		}
	}

	return pRes;
}

CDib *CDib::DilationB(int MaskShape, int MaskLength)
{
	unsigned char *mask = CreateMorphologyMask(MaskShape, MaskLength);
	if (!mask) return NULL;

	CDib *pRes = DilationB(mask, MaskLength);

	delete mask;

	return pRes;
}

CDib *CDib::ErosionB(unsigned char *mask, int MaskLength)
{
	unsigned char *bImgInBuf = GetPattern();
	int width = Width();
    int bwidth = ByteWidth();
	int height = Height();
	CDib *pRes = new CDib; pRes->Allocate(width, height, 8); pRes->SetGrayPalette(); pRes->ResetContents();
	unsigned char *bImgOutBuf = pRes->GetPattern();

	int x, y, i, j, smin;
	int N,M;  

	N = M = MaskLength;
	for( y=N/2; y< height-N/2; y++) {
		for( x=N/2; x<width-N/2;x++) {
			smin=255;
			for( j=-N/2; j<=N/2; j++) {
				for( i=-N/2; i<=N/2; i++) {
					if( *(mask+(N/2+i)*N + (j + N/2) ) ==255) {
						if( bImgInBuf[x+i+(long)(y+j) * bwidth] <smin)
							smin  = bImgInBuf[x+i+(long)(y+j) * bwidth];
					}				
				}
			}
			bImgOutBuf[x+(long)y * bwidth] = smin;
		}
	}

	return pRes;
}

CDib *CDib::ErosionB(int MaskShape, int MaskLength)
{
	BYTE *mask	=  CreateMorphologyMask(MaskShape, MaskLength);
	if (!mask) return NULL;
	
	CDib *pRes = ErosionB(mask, MaskLength);

	delete mask;

	return pRes;
}

CDib *CDib::OpeningB(int MaskShape, int MaskLength, int repeat)
{
	CDib *pDib = CopyCDib();
	int i;

	for (i=0; i<repeat ;i++) {
		CDib *pErosion = pDib->ErosionB(MaskShape, MaskLength);
		delete pDib;
		pDib = pErosion;
	}

	for (i=0; i<repeat ;i++) {
		CDib *pDilation = pDib->DilationB(MaskShape, MaskLength);
		delete pDib;
		pDib = pDilation;
	}

	return pDib;
}

CDib *CDib::ClosingB(int MaskShape, int MaskLength, int repeat)
{
	CDib *pDib = CopyCDib();
	int i;

	for (i=0; i<repeat ;i++) {
		CDib *pDilation = pDib->DilationB(MaskShape, MaskLength);
		delete pDib;
		pDib = pDilation;
	}

	for (i=0; i<repeat ;i++) {
		CDib *pErosion = pDib->ErosionB(MaskShape, MaskLength);
		delete pDib;
		pDib = pErosion;
	}

	return pDib;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Binary Operations
//
//////////////////////////////////////////////////////////////////////////////////////
void CDib::ImageSubtraction(CDib* A, CDib* B)
{

	int width = A->Width();
	int height = A->Height();

	for (int i=0; i<height; i++) {
		unsigned char* ptrA = A->GetPointer(0,i);
		unsigned char* ptrB = B->GetPointer(0,i);
		for (int j=0; j<width; j++, ptrA++, *ptrB++) 
		{
			*ptrB -= *ptrA;
		}
	}
}

int CDib::SubThinning(CDib* img, int i, int j, int* a, int* b)
{
//	Return the number of 01 patterns in the sequence of pixels
//	P2 p3 p4 p5 p6 p7 p8 p9.					

	int n,m;
	int width =  img->Width();
	int height = img->Height();

	for (n=0; n<8; n++)
		a[n] = 0;

	unsigned char* ptrP = img->GetPointer(j, i-1);
	unsigned char* ptrC = img->GetPointer(j, i  );
	unsigned char* ptrN = img->GetPointer(j, i+1);

	if (i-1 >= 0)
	{
		a[0] = *ptrP;
		if (j+1 < width)
			a[1] = *(ptrP+1);
		if (j-1 >= 0)
			a[7] = *(ptrP-1);
	}

	if (i+1 < height)
	{
		a[4] = *ptrN;
		if (j+1 < width)
			a[3] = *(ptrN+1);
		if (j-1 >= 0) 
			a[5] = *(ptrN-1);
	}

	if (j+1 < width)
		a[2] = *(ptrC+1); 
	if (j-1 >= 0)
		a[6] = *(ptrC-1);

	m= 0;
	*b = 0;

	for (n=0; n<7; n++)
	{
		if ((a[n]==0) && (a[n+1]==255))
			m++;
		*b = *b + a[n]/255;
	}

	if ((a[7] == 0) && (a[0] == 255))
		m++;
	*b = *b + a[7]/255;

	return m;
}

CDib* CDib::Thinning()
{
	if (!IsGrayImage()) {
		AfxMessageBox("??? Thinning is allowed only for 8 bit binary image");
		return NULL;
	}

	int i,j, cont, br,ar,p1,p2;
	int a[8];
	int width = Width();
	int height = Height();

	CDib* pTemp = CopyCDib();
	CDib* pRes = CopyCDib();
	cont = 1;
	while(cont)
	{
		cont = 0;
		for (i=0; i<height; i++)
		{
			for (j=0; j<width; j++) 
			{		
				unsigned char* ptr = pRes->GetPointer(j,i);
				unsigned char* ptrTemp = pTemp->GetPointer(j,i);
				if( *ptr == 0)
				{
					*ptrTemp = 0;
					continue;
				}
				ar = SubThinning(pRes, i, j, a, &br);	
				p1 = a[0]*a[2]*a[4];
				p2 = a[2]*a[4]*a[6];
					
				if ( (ar == 1) && ((br>=2) && (br<=6)) && (p1 == 0) && (p2 == 0) )
				{
					*ptrTemp = 255;
					cont = 1;
				}
				else *ptrTemp = 0;
			}
		}
		ImageSubtraction(pTemp, pRes);

		for (i=0; i<height; i++)
		{
			for (j=0; j<width; j++)
			{		
				unsigned char* ptr = pRes->GetPointer(j,i);
				unsigned char* ptrTemp = pTemp->GetPointer(j,i);
				if( *ptr == 0)
				{
					*ptrTemp = 0;
					continue;
				}

				ar = SubThinning(pRes, i, j, a, &br);	
				p1 = a[0]*a[2]*a[6];
				p2 = a[0]*a[4]*a[6];
				if ( (ar == 1) && ((br>=2) && (br<=6)) && (p1 == 0) && (p2 == 0) ) 
				{
					*ptrTemp = 255;
					cont = 1;
				}
				else *ptrTemp = 0;

			}
		}
		ImageSubtraction(pTemp, pRes);
	}

	return pRes;
}

void CDib::GetCovarianceMatrix(CPoint center, double cov[2][2])
{
    double covXX=0., covYY=0., covXY=0., covYX=0.;
    int count=0;
    for (int y=0; y<Height() ;y++) {
      unsigned char *pPtr = GetPointer(0, y);
      for (int x=0; x<Width() ;x++, pPtr++) 
        if (*pPtr!=0) {
          covXX += ((x-center.x)*(x-center.x));
          covYY += ((y-center.y)*(y-center.y));
          covXY += ((x-center.x)*(y-center.y));
          //covYX += ((y-center.y)*(x-center.x));
          count++;
        }
    }

    cov[0][0] = covXX / (double) count;
    cov[0][1] = covXY / (double) count;
    cov[1][0] = cov[0][1]; //covYX / (double) count;
    cov[1][1] = covYY / (double) count;
}

/*======================================================

	Intel Jacobi 루틴

	Parameters: A(n, n) - source symmetric matrix (n - rows & columns number),
	            V(n, n) - matrix of its eigenvectors 
                          (i-th row is an eigenvector Vi),
		        E(n)    - vector of its eigenvalues
                          (i-th element is an eigenvalue Ei),
			    eps     - accuracy of diagonalization.

 ======================================================*/

void CDib::cvJacobiEigens_32f ( float* A,
						  float* V,
			              float* E,
				          int n,
					      float eps ) 
{
    int i, j, k, ind;
    float *AA = A, *VV = V;
    double Amax, anorm = 0, ax;

    /*if ( A == NULL || V == NULL || E == NULL ) return CV_NULLPTR_ERR;*/
    /*if ( n <= 0 )                              return CV_BADSIZE_ERR;*/
    if (eps < 1.0e-7f )
        eps = 1.0e-7f;

    /*-------- Prepare --------*/
    for (i = 0; i < n; i++, VV += n, AA += n) {
        for (j = 0; j < i; j++) {
            double Am = AA[j];
            anorm += Am * Am;
        }
        for (j = 0; j < n; j++)
            VV[j] = 0.f;
        VV[i] = 1.f;
    }

    anorm = sqrt( anorm + anorm );
    ax = anorm * eps / n;
    Amax = anorm;

    while ( Amax > ax ) {
        Amax /= n;
        do  /* while (ind) */
        {
            int p, q;
            float *V1 = V, *A1 = A;
            ind = 0;
            for (p = 0; p < n - 1; p++, A1 += n, V1 += n) {
                float * A2 = A + n * (p + 1), *V2 = V + n * (p + 1);
                for (q = p + 1; q < n; q++, A2 += n, V2 += n) {
                    double x, y, c, s, c2, s2, a;
                    float *A3, Apq = A1[q], App, Aqq, Aip, Aiq, Vpi, Vqi;
                    if ( fabs( Apq ) < Amax )
                        continue;

                    ind = 1;

                    /*---- Calculation of rotation angle's sine & cosine ----*/
                    App = A1[p];
                    Aqq = A2[q];
                    y = 5.0e-1 * (App - Aqq);
                    x = -Apq / sqrt(Apq * Apq + y * y);
                    if (y < 0.0)
                        x = -x;
                    s = x / sqrt(2.0 * (1.0 + sqrt(1.0 - x * x)));
                    s2 = s * s;
                    c = sqrt(1.0 - s2);
                    c2 = c * c;
                    a = 2.0 * Apq * c * s;

                    /*---- Apq annulation ----*/
                    A3 = A;
                    for (i = 0; i < p; i++, A3 += n) {
                        Aip = A3[p];
                        Aiq = A3[q];
                        Vpi = V1[i];
                        Vqi = V2[i];
                        A3[p] = (float)(Aip * c - Aiq * s);
                        A3[q] = (float)(Aiq * c + Aip * s);
                        V1[i] = (float)(Vpi * c - Vqi * s);
                        V2[i] = (float)(Vqi * c + Vpi * s);
                    }
                    for (; i < q; i++, A3 += n) {
                        Aip = A1[i];
                        Aiq = A3[q];
                        Vpi = V1[i];
                        Vqi = V2[i];
                        A1[i] = (float)(Aip * c - Aiq * s);
                        A3[q] = (float)(Aiq * c + Aip * s);
                        V1[i] = (float)(Vpi * c - Vqi * s);
                        V2[i] = (float)(Vqi * c + Vpi * s);
                    }
                    for (; i < n; i++) {
                        Aip = A1[i];
                        Aiq = A2[i];
                        Vpi = V1[i];
                        Vqi = V2[i];
                        A1[i] = (float)(Aip * c - Aiq * s);
                        A2[i] = (float)(Aiq * c + Aip * s);
                        V1[i] = (float)(Vpi * c - Vqi * s);
                        V2[i] = (float)(Vqi * c + Vpi * s);
                    }
                    A1[p] = (float)(App * c2 + Aqq * s2 - a);
                    A2[q] = (float)(App * s2 + Aqq * c2 + a);
                    A1[q] = A2[p] = 0.0f;
                } /*q*/
            }     /*p*/
        } while (ind);
        Amax /= n;
    }   /* while ( Amax > ax ) */

    for (i = 0, k = 0; i < n; i++, k += n + 1)
        E[i] = A[k];
    /*printf(" M = %d\n", M);*/

    /* -------- ordering --------*/
    for (i = 0; i < n; i++) {
        int m = i;
        float Em = (float)fabs(E[i]);
        for (j = i + 1; j < n; j++) {
            float Ej = (float)fabs(E[j]);
            m = ( Em < Ej ) ? j : m;
            Em = ( Em < Ej ) ? Ej : Em;
        }
        if ( m != i ) {
            int l;
            float b = E[i];
            E[i] = E[m];
            E[m] = b;
            for (j = 0, k = i * n, l = m * n; j < n; j++, k++, l++) {
                b = V[k];
                V[k] = V[l];
                V[l] = b;
            }
        }
    }

   // return 0;
}



void CDib::cvJacobiEigens_64d( double* A,
                              double* V,
                              double* E,
                              int     n,
                              double  eps )
{
    int i, j, k, p, q, ind;
    double *A1 = A, *V1 = V, *A2=A, *V2=V;
    double Amax = 0.0, anorm = 0.0, ax, deps;

    //if ( A == NULL || V == NULL || E == NULL ) return CV_NULLPTR_ERR;
    //if ( n <= 0 )                              return CV_BADSIZE_ERR;
    if (eps < 1.0e-15 )  eps = 1.0e-15;
    deps = eps/(double)n;

    /*-------- Prepare --------*/
    for(i=0; i<n; i++, V1+=n, A1+=n)
    {
        for(j=0; j<i; j++)
        {
            double Am = A1[j];
            anorm += Am*Am;
        }
        for(j=0; j<n; j++) V1[j] = 0.0;
        V1[i] = 1.0;
    }

    anorm = sqrt( anorm+anorm );
    ax    = anorm*eps/n;
    Amax  = anorm;

    while ( Amax > ax )
    {
        Amax /= n;
        do  /* while (ind) */
        {
            ind = 0;
            A1  = A;
            V1  = V;
            for(p=0; p<n-1; p++, A1+=n, V1+=n)
            {
                A2 = A + n*(p+1);
                V2 = V + n*(p+1);
                for(q=p+1; q<n; q++, A2+=n, V2+=n)
                {
                    double x, y, c, s, c2, s2, a;
                    double *A3, Apq, App, Aqq, App2, Aqq2, Aip, Aiq, Vpi, Vqi;
                    if( fabs( A1[q] ) < Amax ) continue;
                    Apq=A1[q];

                    ind=1;

                    /*---- Calculation of rotation angle's sine & cosine ----*/
                    App = A1[p];
                    Aqq = A2[q];
                    y   = 5.0e-1*(App - Aqq);
                    x = -Apq / sqrt(Apq*Apq + y*y);
                    if(y<0.0) x = -x;
                    s = x / sqrt(2.0*(1.0 + sqrt(1.0 - x*x)));
                    s2 = s*s;
                    c  = sqrt(1.0 - s2);
                    c2 = c*c;
                    a  = 2.0*Apq*c*s;

                    /*---- Apq annulation ----*/
                    A3 = A;
                    for(i=0; i<p; i++, A3+=n)
                    {
                        Aip = A3[p];
                        Aiq = A3[q];
                        Vpi = V1[i];
                        Vqi = V2[i];
                        A3[p] = Aip*c - Aiq*s;
                        A3[q] = Aiq*c + Aip*s;
                        V1[i] = Vpi*c - Vqi*s;
                        V2[i] = Vqi*c + Vpi*s;
                    }
                    for(; i<q; i++, A3+=n)
                    {
                        Aip = A1[i];
                        Aiq = A3[q];
                        Vpi = V1[i];
                        Vqi = V2[i];
                        A1[i] = Aip*c - Aiq*s;
                        A3[q] = Aiq*c + Aip*s;
                        V1[i] = Vpi*c - Vqi*s;
                        V2[i] = Vqi*c + Vpi*s;
                    }
                    for(; i<n; i++)
                    {
                        Aip = A1[i];
                        Aiq = A2[i];
                        Vpi = V1[i];
                        Vqi = V2[i];
                        A1[i] = Aip*c - Aiq*s;
                        A2[i] = Aiq*c + Aip*s;
                        V1[i] = Vpi*c - Vqi*s;
                        V2[i] = Vqi*c + Vpi*s;
                    }
                    App2  = App*c2 + Aqq*s2 - a;
                    Aqq2  = App*s2 + Aqq*c2 + a;
                    A1[p] = App2;
                    A2[q] = Aqq2;
                    A1[q] = A2[p] = 0.0;
                } /*q*/
            }     /*p*/
        } while (ind);
    }   /* while ( Amax > ax ) */

    for(i=0, k=0; i<n; i++, k+=n+1) E[i] = A[k];

    /* -------- ordering --------*/
    for(i=0; i<n; i++)
    {
        int m = i;
        double Em = fabs(E[i]);
        for(j=i+1; j<n; j++)
        {
            double Ej = fabs(E[j]);
            m  = ( Em < Ej ) ?  j :  m;
            Em = ( Em < Ej ) ? Ej : Em;
        }
        if( m != i )
        {
            int l;
            double b = E[i];
            E[i] = E[m];
            E[m] = b;
            for(j=0, k=i*n, l=m*n; j<n; j++, k++, l++)
            {
                b    = V[k];
                V[k] = V[l];
                V[l] = b;
            }
        }
    }

    //return 0;
}

double CDib::GetOrientationPCA(CPoint center)
{
	if (!IsGrayImage()) {
		AfxMessageBox("??? GetOrientationPCA is allowed only for 8 bit binary image");
		return NULL;
	}

    double cov[2][2];
    GetCovarianceMatrix(center, cov);

//    TRACE("%10.2lf %10.2lf\n", cov[0][0], cov[0][1]);
//    TRACE("%10.2lf %10.2lf\n", cov[1][0], cov[1][1]);

	double vector[2][2];
	double eigen[2];
	double eps=0.;
	cvJacobiEigens_64d((double *) cov, (double *) vector, (double *) eigen, 2, eps);

    if (eigen[0]>eigen[1]) return atan2(vector[0][1], vector[0][0]);
    else return atan2(vector[1][1], vector[1][0]);
}

double CDib::LogForm(double value)
{
    if (value>0) return log(value);
    //else if (value<0) return -log(-value);
    else if (value<0) return log(-value);   // absolute에 대하여 log를 취하는 것이 더 합당하지 않을까?
    else return -1000;
}

void CDib::GetInvariantMoments(double value[7])
{
	if (!IsGrayImage()) {
		AfxMessageBox("??? GetInvariantMoments is allowed only for 8 bit binary image");
		return;
	}

    // Invariant Moments
    double m00=0., m10=0., m01=0., m11=0., m20=0., m02=0., m12=0., m21=0., m30=0., m03=0.;
    for (int i=0; i<Height() ;i++) {
      unsigned char *pPtr = GetPointer(0, i);
      for (int j=0; j<Width() ;j++, pPtr++) {
        if (!(*pPtr)) continue;
        double x = (double) j;
        double y = (double) i;
        m00++;
        m10 += x;
        m01 += y;
        m11 += (x*y);
        m20 += (x*x);
        m02 += (y*y);
        m12 += (x*y*y);
        m21 += (x*x*y);
        m30 += (x*x*x);
        m03 += (y*y*y);
      }
    }

    // centural moment
    double mx = m10 / m00, my = m01 / m00;
    double mu00 = m00;
    double mu10 = 0.;
    double mu01 = 0.;
    double mu20 = m20 - mx*m10;
    double mu02 = m02 - my*m01;
    double mu11 = m11 - my*m10;
    double mu30 = m30 - 3*mx*m20 + 2*m10*mx*mx;
    double mu12 = m12 - 2*my*m11 - mx*m02 + 2*my*my*m10;
    double mu21 = m21 - 2*mx*m11 - my*m20 + 2*mx*mx*m01;
    double mu03 = m03 - 3*my*m02 + 2*my*my*m01;

    // normalized central moment
    double n20 = mu20 / pow(mu00, 2.);
    double n02 = mu02 / pow(mu00, 2.);
    double n11 = mu11 / pow(mu00, 2.);
    double n30 = mu30 / pow(mu00, 2.5);
    double n12 = mu12 / pow(mu00, 2.5);
    double n21 = mu21 / pow(mu00, 2.5);
    double n03 = mu03 / pow(mu00, 2.5);

    value[0] = n20 + n02;
    value[1] = (n20-n02)*(n20-n02) + 4*n11*n11;
    value[2] = (n30-3*n12)*(n30-3*n12) + (3*n21-n03)*(3*n21-n03);
    value[3] = (n30+n12)*(n30+n12) + (n21+n03)*(n21+n03);
    value[4] = (n30-3*n12)*(n30+n12)*((n30+n12)*(n30+n12)-3*(n21+n03)*(n21+n03))
             + (3*n21-n03)*(n21+n03)*(3*(n30+n12)*(n30+n12)-(n21+n03)*(n21+n03));
    value[5] = (n20-n02)*((n30+n12)*(n30+n12) - (n21+n03)*(n21+n03))
             + 4*n11*(n30+n12)*(n21+n03);
    value[6] = (3*n21-n03)*(n30+n12)*((n30+n12)*(n30+n12) - 3*(n21+n03)*(n21+n03))
             + (3*n12-n30)*(n21+n03)*(3*(n30+n12)*(n30+n12) - (n21+n03)*(n21+n03));
}

void CDib::GetLogScaleInvariantMoments(double value[7])
{
	if (!IsGrayImage()) {
		AfxMessageBox("??? GetLogScaleInvariantMoments is allowed only for 8 bit binary image");
		return;
	}

    // Invariant Moments
    double m00=0., m10=0., m01=0., m11=0., m20=0., m02=0., m12=0., m21=0., m30=0., m03=0.;
    for (int i=0; i<Height() ;i++) {
      unsigned char *pPtr = GetPointer(0, i);
      for (int j=0; j<Width() ;j++, pPtr++) {
        if (!(*pPtr)) continue;
        double x = (double) j;
        double y = (double) i;
        m00++;
        m10 += x;
        m01 += y;
        m11 += (x*y);
        m20 += (x*x);
        m02 += (y*y);
        m12 += (x*y*y);
        m21 += (x*x*y);
        m30 += (x*x*x);
        m03 += (y*y*y);
      }
    }

    // centural moment
    double mx = m10 / m00, my = m01 / m00;
    double mu00 = m00;
    double mu10 = 0.;
    double mu01 = 0.;
    double mu20 = m20 - mx*m10;
    double mu02 = m02 - my*m01;
    double mu11 = m11 - my*m10;
    double mu30 = m30 - 3*mx*m20 + 2*m10*mx*mx;
    double mu12 = m12 - 2*my*m11 - mx*m02 + 2*my*my*m10;
    double mu21 = m21 - 2*mx*m11 - my*m20 + 2*mx*mx*m01;
    double mu03 = m03 - 3*my*m02 + 2*my*my*m01;

    // normalized central moment
    double n20 = mu20 / pow(mu00, 2.);
    double n02 = mu02 / pow(mu00, 2.);
    double n11 = mu11 / pow(mu00, 2.);
    double n30 = mu30 / pow(mu00, 2.5);
    double n12 = mu12 / pow(mu00, 2.5);
    double n21 = mu21 / pow(mu00, 2.5);
    double n03 = mu03 / pow(mu00, 2.5);

    value[0] = LogForm( n20 + n02 );
    value[1] = LogForm( (n20-n02)*(n20-n02) + 4*n11*n11 );
    value[2] = LogForm( (n30-3*n12)*(n30-3*n12) + (3*n21-n03)*(3*n21-n03) );
    value[3] = LogForm( (n30+n12)*(n30+n12) + (n21+n03)*(n21+n03) );
    value[4] = LogForm( (n30-3*n12)*(n30+n12)*((n30+n12)*(n30+n12)-3*(n21+n03)*(n21+n03))
             + (3*n21-n03)*(n21+n03)*(3*(n30+n12)*(n30+n12)-(n21+n03)*(n21+n03)) );
    value[5] = LogForm( (n20-n02)*((n30+n12)*(n30+n12) - (n21+n03)*(n21+n03))
             + 4*n11*(n30+n12)*(n21+n03) );
    value[6] = LogForm( (3*n21-n03)*(n30+n12)*((n30+n12)*(n30+n12) - 3*(n21+n03)*(n21+n03))
             + (3*n12-n30)*(n21+n03)*(3*(n30+n12)*(n30+n12) - (n21+n03)*(n21+n03)) );
}

CDib *CDib::GetOuterBoundary()
{
	ASSERT(BitCount()==8);

    CDib *pBnd = CopyCDib();
    pBnd->ResetContents();

    // From left
	int i;
    for (i=0; i<Height() ;i++) {
      unsigned char *ptr = GetPointer(0, i);
      for (int j=0; j<Width() ;j++, ptr++)
        if (*ptr!=0) {
          *pBnd->GetPointer(j,i) = 255;
          break;
        }
    }

    // From right
    for (i=0; i<Height() ;i++) {
      unsigned char *ptr = GetPointer(Width()-1, i);
      for (int j=(Width()-1); j>=0 ;j--, ptr--)
        if (*ptr!=0) {
          *pBnd->GetPointer(j,i) = 255;
          break;
        }
    }

    // from top
    for (i=0; i<Width() ;i++) {
      unsigned char *ptr = GetPointer(i, 0);
      for (int j=0; j<Height() ;j++) {
        ptr = GetPointer(i, j);
        if (*ptr!=0) {
          *pBnd->GetPointer(i, j) = 255;
          break;
        }
      }
    }

    // from bottom
    for (i=0; i<Width() ;i++) {
      unsigned char *ptr = GetPointer(i, Height()-1);
      for (int j=(Height()-1); j>=0 ;j--) {
        ptr = GetPointer(i, j);
        if (*ptr!=0) {
          *pBnd->GetPointer(i, j) = 255;
          break;
        }
      }
    }

    return pBnd;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// Connected Component Analysis
//
//////////////////////////////////////////////////////////////////////////////////////
void CDib::enter_equ_table(int a, int b, int equ_table[])
{
    register int temp1, temp2;
    static int tempa = 0, tempb = 0;
    
    if (tempa != a || tempb != b) 
    {  
       temp1 = equ_table[a]; temp2 = equ_table[b];
       while(temp1 != equ_table[temp1]) {
          temp1 = equ_table[temp1];
       }
       while(temp2 != equ_table[temp2]) {
          temp2 = equ_table[temp2];
       }
       if (temp1 < temp2) { 
           equ_table[temp2] = temp1;
       } else {
	   equ_table[temp1] = temp2;
       } /*
       if (temp1 == 0 || temp2 == 0) { 
          printf("a = %d, b = %d, e[a] = %d, e[b] = %d\n", a, b, equ_table[a], equ_table[b]);
       } */ 
       tempa = a; tempb = b;
    }  
}

void CDib::make_mapping_table(CWDib *pWDib, int equ_table[], int mapping_table[], struct label_info lab_info[], struct label_info lab_out[])
{
   register int y, x;
   register int i, k;
   int count = 0, tmpcount = 0, temp;
   
   for (i = 1; i < num_of_label; i++) 
      if (equ_table[i] != i) {
         temp = equ_table[i]; 
	 while(temp != equ_table[temp]) {
           temp = equ_table[temp];
         }
	 equ_table[i] = temp; /*
	 printf("equ_table[%d] = %d\n", i, equ_table[i]); 
         */ 
      }

   for (i = 1; i <= num_of_label; i++) {
      if (equ_table[i] == i) {
	  k = mapping_table[i] = ++count;
	  lab_out[k].count = lab_info[i].count;
	  lab_out[k].gray = lab_info[i].gray;
	  lab_out[k].x_start = lab_info[i].x_start;
	  lab_out[k].x_end = lab_info[i].x_end;
	  lab_out[k].y_start = lab_info[i].y_start;
	  lab_out[k].y_end = lab_info[i].y_end;
      }
   }

   for (i = 1; i <= num_of_label; i++)
      if (equ_table[i] != i) {
          k = mapping_table[i] = mapping_table[equ_table[i]];
		  lab_out[k].count = lab_out[k].count + lab_info[i].count;
	      lab_out[k].gray = lab_out[k].gray + lab_info[i].gray;
          if (lab_out[k].x_start > lab_info[i].x_start)
	      lab_out[k].x_start = lab_info[i].x_start;
          if (lab_out[k].x_end < lab_info[i].x_end)
	      lab_out[k].x_end = lab_info[i].x_end;
          if (lab_out[k].y_start > lab_info[i].y_start)
	      lab_out[k].y_start = lab_info[i].y_start;
          if (lab_out[k].y_end < lab_info[i].y_end)
	      lab_out[k].y_end = lab_info[i].y_end;
      }
  
  
   //printf("Initial label number = %d\n", num_of_label);
   num_of_label = count;
   //ASSERT(count<253);  
   // printf(" Total region count  = %d\n", count);
   for (y=0; y<Height(); y++) {
		unsigned char *pC = GetPointer(0, y);
		short *pWC = pWDib->GetPointer(0, y);
		for (x=0; x <Width(); x++, pC++, pWC++) {
			if (*pWC==0) *pC = BACKG; //255;
			else {
				*pC = (unsigned char) mapping_table[*pWC];
				*pWC = (short) mapping_table[*pWC];
			}
		}
   } 
}

CObBlobArray *CDib::ConnectedComponentAnalysis(CWDib *pLabel)
{
	ASSERT(BitCount()==8);

    register WORD i, y, x, counter=0;
    register unsigned char now, up, left, up_left, up_right;
    WORD label_up, label_left, label_up_left, label_up_right, ltemp;
    //int equ_table[Max], mapping_table[Max];
    //struct label_info lab_info[Max], lab_out[Max];
    int *equ_table = new int [Max];
    int *mapping_table = new int [Max];
    struct label_info *lab_info = new struct label_info [Max];
    struct label_info *lab_out = new struct label_info [Max];

    for (i = 1; i < Max; i++) {
      equ_table[i] = i;
      lab_info[i].count = 0;	
      lab_out[i].count = 0;		
	  lab_info[i].x_start = 99999; //pDib->Height();
      lab_out[i].x_start = 99999; //pDib->Height();	
	  lab_info[i].x_end = 0;
      lab_out[i].x_end = 0;	
	  lab_info[i].y_start = 99999; //pDib->Width();
      lab_out[i].y_start = 99999; //pDib->Width();	
	  lab_info[i].y_end = 0;
      lab_out[i].y_end = 0;	
      lab_info[i].flag = 0;
      lab_out[i].flag = 0;	
    }
    
    for (y = 0; y <Height() ; y++) {
      *GetPointer(0, y) = BACKG;
      *GetPointer(Width()-1, y) = BACKG;
    }

    unsigned char *p1 = GetPointer(0, 0);
    unsigned char *p2 = GetPointer(0, Height()-1);
    for (x=0; x<Width() ;x++) { 
      *p1++ = BACKG;
      *p2++ = BACKG;
    }

    CWDib wdib;
	CWDib *pWDib;
	if (pLabel) pWDib = pLabel;
	else pWDib = &wdib;
    pWDib->Allocate(Width(), Height());
    pWDib->ResetContents();

    for (y=1; y<(Height()-1) ;y++) {
      unsigned char *pU = GetPointer(1, y-1);
      unsigned char *pC = GetPointer(1, y);
      short *pWU = pWDib->GetPointer(1, y-1);
      short *pWC = pWDib->GetPointer(1, y);
      for (x=1; x<(Width()-1) ;x++, pU++, pC++, pWU++, pWC++) {
	    now = *pC;

        if (now==BACKG) *pWC = 0;
	    else {
	      up = *pU;
	      up_right = *(pU+1);
	      left = *(pC-1);
	      up_left = *(pU-1);
	      if ((now!=up && now!=left) &&
	          (now!=up_left && now!=up_right)) 
            *pWC = ++counter;
	      else if (now==up && now!=left) 
            *pWC = *pWU;
	      else if ((now!=up && now==left) && (now!=up_right)) 
	        *pWC = *(pWC-1);
	      else if (((now!=up && now!=left)) && ((now==up_left) && (now!=up_right))) 
	        *pWC = *(pWU-1);
	      else if (((now!=up && now!=left)) && ((now!=up_left) && (now==up_right))) 
	        *pWC = *(pWU+1);
	      else {
	        label_up = *pWU;
	        label_left = *(pWC-1);
	        label_up_left = *(pWU-1);
	        label_up_right = *(pWU+1);
	        if (now == up && now == left) {
	          if (label_up == label_left)
		        *pWC = label_left;
	          else {
		        *pWC = label_left;
		        enter_equ_table(label_up, label_left, equ_table);
	          }
	        }

	        else if (now == up_left && now == up_right) {
              if (label_up_left == label_up_right)
		        *pWC = label_up_left;
	          else {
		        *pWC = label_up_left;
		        enter_equ_table(label_up_left, label_up_right, equ_table);
	          }
            }
		   
	        else if (now == left && now == up_right) {
	          if (label_left == label_up_right)
	            *pWC = label_left;
	          else {
	            *pWC = label_left;
		        enter_equ_table(label_left, label_up_right, equ_table);
              }
	        }
	      }
	    }
	    ltemp = *pWC;
	    lab_info[ltemp].count++; 
/*
	    if (lab_info[ltemp].x_start>y) lab_info[ltemp].x_start = y;
	    if (lab_info[ltemp].x_end<y) lab_info[ltemp].x_end = y;
	    if (lab_info[ltemp].y_start>x) lab_info[ltemp].y_start = x;
	    if (lab_info[ltemp].y_end<x) lab_info[ltemp].y_end = x;
*/
	    if (lab_info[ltemp].x_start>x) lab_info[ltemp].x_start = x;
	    if (lab_info[ltemp].x_end<x) lab_info[ltemp].x_end = x;
	    if (lab_info[ltemp].y_start>y) lab_info[ltemp].y_start = y;
	    if (lab_info[ltemp].y_end<y) lab_info[ltemp].y_end = y;
      } /* for x end */	
    }
    num_of_label = counter;	
    make_mapping_table(pWDib, equ_table, mapping_table, lab_info, lab_out);

	// make object blob array

	CObBlobArray *pRes = new CObBlobArray;
	for (i=1; i<=num_of_label ;i++) {
	  CObBlob *pBlob = new CObBlob;
	  pBlob->m_Label = i;
	  pBlob->m_Count = lab_out[i].count;
	  pBlob->m_Region.SetRect(lab_out[i].x_start, lab_out[i].y_start, lab_out[i].x_end+1, lab_out[i].y_end+1);
	  pRes->Add(pBlob);
	}

    delete [] equ_table;
    delete [] mapping_table;
    delete [] lab_info;
    delete [] lab_out;

	return pRes;
}

////////////////////////////////////////////////////////////////////////////////////
// Functions for Gaussian Smoothing
//
/*******************************************************************************
* PROCEDURE: make_gaussian_kernel
* PURPOSE: Create a one dimensional gaussian kernel.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void CDib::make_gaussian_kernel(float sigma, float **kernel, int *windowsize)
{
   int i, center;
   float x, fx, sum=0.0;

   *windowsize = (int) (1 + 2 * ceil(2.5 * sigma));
   center = (*windowsize) / 2;

   //if(VERBOSE) printf("      The kernel has %d elements.\n", *windowsize);
   if((*kernel = (float *) calloc((*windowsize), sizeof(float))) == NULL){
      fprintf(stderr, "Error callocing the gaussian kernel array.\n");
      exit(1);
   }

   for(i=0;i<(*windowsize);i++){
      x = (float)(i - center);
      fx = (float) (pow(2.71828, -0.5*x*x/(sigma*sigma)) / (sigma * sqrt(6.2831853)));
      (*kernel)[i] = fx;
      sum += fx;
   }

   for(i=0;i<(*windowsize);i++) (*kernel)[i] /= sum;

   /*
   if(VERBOSE){
      printf("The filter coefficients are:\n");
      for(i=0;i<(*windowsize);i++)
         printf("kernel[%d] = %f\n", i, (*kernel)[i]);
   }
   */
}

/*******************************************************************************
* PROCEDURE: gaussian_smooth
* PURPOSE: Blur an image with a gaussian filter.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void CDib::gaussian_smooth(unsigned char *image, int rows, int cols, float sigma,
        short int *smoothedim)
{
   int r, c, rr, cc,     /* Counter variables. */
      windowsize,        /* Dimension of the gaussian kernel. */
      center;            /* Half of the windowsize. */
   float *tempim,        /* Buffer for separable filter gaussian smoothing. */
         *kernel,        /* A one dimensional gaussian kernel. */
         dot,            /* Dot product summing variable. */
         sum;            /* Sum of the kernel weights variable. */

   /****************************************************************************
   * Create a 1-dimensional gaussian smoothing kernel.
   ****************************************************************************/
   //if(VERBOSE) printf("   Computing the gaussian smoothing kernel.\n");
   make_gaussian_kernel(sigma, &kernel, &windowsize);
   center = windowsize / 2;

   /****************************************************************************
   * Allocate a temporary buffer image and the smoothed image.
   ****************************************************************************/
   if((tempim = (float *) calloc(rows*cols, sizeof(float))) == NULL){
      fprintf(stderr, "Error allocating the buffer image.\n");
      exit(1);
   }

   // smoothedim은 외부에서 Allocate됨
   //if(((*smoothedim) = (short int *) calloc(rows*cols,
   //      sizeof(short int))) == NULL){
   //   fprintf(stderr, "Error allocating the smoothed image.\n");
   //   exit(1);
   //}

   /****************************************************************************
   * Blur in the x - direction.
   ****************************************************************************/
   //if(VERBOSE) printf("   Bluring the image in the X-direction.\n");
   for(r=0;r<rows;r++){
      for(c=0;c<cols;c++){
         dot = 0.0;
         sum = 0.0;
         for(cc=(-center);cc<=center;cc++){
            if(((c+cc) >= 0) && ((c+cc) < cols)){
               dot += (float)image[r*cols+(c+cc)] * kernel[center+cc];
               sum += kernel[center+cc];
            }
         }
         tempim[r*cols+c] = dot/sum;
      }
   }

   /****************************************************************************
   * Blur in the y - direction.
   ****************************************************************************/
   //if(VERBOSE) printf("   Bluring the image in the Y-direction.\n");
   for(c=0;c<cols;c++){
      for(r=0;r<rows;r++){
         sum = 0.0;
         dot = 0.0;
         for(rr=(-center);rr<=center;rr++){
            if(((r+rr) >= 0) && ((r+rr) < rows)){
               dot += tempim[(r+rr)*cols+c] * kernel[center+rr];
               sum += kernel[center+rr];
            }
         }
         (smoothedim)[r*cols+c] = (short int)(dot*BOOSTBLURFACTOR/sum + 0.5);
      }
   }

   free(tempim);
   free(kernel);
}


////////////////////////////////////////////////////////////////////////////////////
// Functions for Canny Edge Extraction
//
/*******************************************************************************
* FILE: hysteresis.c
* This code was re-written by Mike Heath from original code obtained indirectly
* from Michigan State University. heath@csee.usf.edu (Re-written in 1996).
*******************************************************************************/


/*******************************************************************************
* PROCEDURE: follow_edges
* PURPOSE: This procedure edges is a recursive routine that traces edgs along
* all paths whose magnitude values remain above some specifyable lower
* threshhold.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void CDib::follow_edges(unsigned char *edgemapptr, short *edgemagptr, short lowval,
   int cols)
{
   short *tempmagptr;
   unsigned char *tempmapptr;
   int i;
   //float thethresh;
   int x[8] = {1,1,0,-1,-1,-1,0,1},
       y[8] = {0,1,1,1,0,-1,-1,-1};

   for(i=0;i<8;i++){
      tempmapptr = edgemapptr - y[i]*cols + x[i];
      tempmagptr = edgemagptr - y[i]*cols + x[i];

      if((*tempmapptr == POSSIBLE_EDGE) && (*tempmagptr > lowval)){
         *tempmapptr = (unsigned char) EDGE;
         follow_edges(tempmapptr,tempmagptr, lowval, cols);
      }
   }
}

/*******************************************************************************
* PROCEDURE: apply_hysteresis
* PURPOSE: This routine finds edges that are above some high threshhold or
* are connected to a high pixel by a path of pixels greater than a low
* threshold.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void CDib::apply_hysteresis(short int *mag, unsigned char *nms, int rows, int cols,
	float tlow, float thigh, unsigned char *edge)
{
   int r, c, pos, numedges, /*lowcount,*/ highcount, lowthreshold, highthreshold,
       /*i,*/ hist[32768]/*, rr, cc*/;
   short int maximum_mag /*, sumpix*/;

   /****************************************************************************
   * Initialize the edge map to possible edges everywhere the non-maximal
   * suppression suggested there could be an edge except for the border. At
   * the border we say there can not be an edge because it makes the
   * follow_edges algorithm more efficient to not worry about tracking an
   * edge off the side of the image.
   ****************************************************************************/
   for(r=0,pos=0;r<rows;r++){
      for(c=0;c<cols;c++,pos++){
	 if(nms[pos] == POSSIBLE_EDGE) edge[pos] = POSSIBLE_EDGE;
	 else edge[pos] = NOEDGE;
      }
   }

   for(r=0,pos=0;r<rows;r++,pos+=cols){
      edge[pos] = NOEDGE;
      edge[pos+cols-1] = NOEDGE;
   }
   pos = (rows-1) * cols;
   for(c=0;c<cols;c++,pos++){
      edge[c] = NOEDGE;
      edge[pos] = NOEDGE;
   }

   /****************************************************************************
   * Compute the histogram of the magnitude image. Then use the histogram to
   * compute hysteresis thresholds.
   ****************************************************************************/
   for(r=0;r<32768;r++) hist[r] = 0;
   for(r=0,pos=0;r<rows;r++){
      for(c=0;c<cols;c++,pos++){
	 if(edge[pos] == POSSIBLE_EDGE) hist[mag[pos]]++;
      }
   }

   /****************************************************************************
   * Compute the number of pixels that passed the nonmaximal suppression.
   ****************************************************************************/
   for(r=1,numedges=0;r<32768;r++){
      if(hist[r] != 0) maximum_mag = r;
      numedges += hist[r];
   }

   highcount = (int)(numedges * thigh + 0.5);

   /****************************************************************************
   * Compute the high threshold value as the (100 * thigh) percentage point
   * in the magnitude of the gradient histogram of all the pixels that passes
   * non-maximal suppression. Then calculate the low threshold as a fraction
   * of the computed high threshold value. John Canny said in his paper
   * "A Computational Approach to Edge Detection" that "The ratio of the
   * high to low threshold in the implementation is in the range two or three
   * to one." That means that in terms of this implementation, we should
   * choose tlow ~= 0.5 or 0.33333.
   ****************************************************************************/
   r = 1;
   numedges = hist[1];
   while((r<(maximum_mag-1)) && (numedges < highcount)){
      r++;
      numedges += hist[r];
   }
   highthreshold = r;
   lowthreshold = (int)(highthreshold * tlow + 0.5);

// if(VERBOSE){
//    printf("The input low and high fractions of %f and %f computed to\n",
//tlow, thigh);
//    printf("magnitude of the gradient threshold values of: %d %d\n",
//lowthreshold, highthreshold);
// }

   /****************************************************************************
   * This loop looks for pixels above the highthreshold to locate edges and
   * then calls follow_edges to continue the edge.
   ****************************************************************************/
   for(r=0,pos=0;r<rows;r++){
      for(c=0;c<cols;c++,pos++){
	 if((edge[pos] == POSSIBLE_EDGE) && (mag[pos] >= highthreshold)){
            edge[pos] = EDGE;
            follow_edges((edge+pos), (mag+pos), lowthreshold, cols);
	 }
      }
   }

   /****************************************************************************
   * Set all the remaining possible edges to non-edges.
   ****************************************************************************/
   for(r=0,pos=0;r<rows;r++){
      for(c=0;c<cols;c++,pos++) if(edge[pos] != EDGE) edge[pos] = NOEDGE;
   }
}

/*******************************************************************************
* PROCEDURE: non_max_supp
* PURPOSE: This routine applies non-maximal suppression to the magnitude of
* the gradient image.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void CDib::non_max_supp(short *mag, short *gradx, short *grady, int nrows, int ncols,
    unsigned char *result) 
{
    int rowcount, colcount,count;
    short *magrowptr,*magptr;
    short *gxrowptr,*gxptr;
    short *gyrowptr,*gyptr,z1,z2;
    short m00,gx,gy;
    float mag1,mag2,xperp,yperp;
    unsigned char *resultrowptr, *resultptr;
    

   /****************************************************************************
   * Zero the edges of the result image.
   ****************************************************************************/
    for(count=0,resultrowptr=result,resultptr=result+ncols*(nrows-1); 
        count<ncols; resultptr++,resultrowptr++,count++){
        *resultrowptr = *resultptr = (unsigned char) 0;
    }

    for(count=0,resultptr=result,resultrowptr=result+ncols-1;
        count<nrows; count++,resultptr+=ncols,resultrowptr+=ncols){
        *resultptr = *resultrowptr = (unsigned char) 0;
    }

   /****************************************************************************
   * Suppress non-maximum points.
   ****************************************************************************/
   for(rowcount=1,magrowptr=mag+ncols+1,gxrowptr=gradx+ncols+1,
      gyrowptr=grady+ncols+1,resultrowptr=result+ncols+1;
      rowcount<nrows-2; 
      rowcount++,magrowptr+=ncols,gyrowptr+=ncols,gxrowptr+=ncols,
      resultrowptr+=ncols){   
      for(colcount=1,magptr=magrowptr,gxptr=gxrowptr,gyptr=gyrowptr,
         resultptr=resultrowptr;colcount<ncols-2; 
         colcount++,magptr++,gxptr++,gyptr++,resultptr++){   
         m00 = *magptr;
         if(m00 == 0){
            *resultptr = (unsigned char) NOEDGE;
         }
         else{
            xperp = -(gx = *gxptr)/((float)m00);
            yperp = (gy = *gyptr)/((float)m00);
         }

         if(gx >= 0){
            if(gy >= 0){
                    if (gx >= gy)
                    {  
                        /* 111 */
                        /* Left point */
                        z1 = *(magptr - 1);
                        z2 = *(magptr - ncols - 1);

                        mag1 = (m00 - z1)*xperp + (z2 - z1)*yperp;
                        
                        /* Right point */
                        z1 = *(magptr + 1);
                        z2 = *(magptr + ncols + 1);

                        mag2 = (m00 - z1)*xperp + (z2 - z1)*yperp;
                    }
                    else
                    {    
                        /* 110 */
                        /* Left point */
                        z1 = *(magptr - ncols);
                        z2 = *(magptr - ncols - 1);

                        mag1 = (z1 - z2)*xperp + (z1 - m00)*yperp;

                        /* Right point */
                        z1 = *(magptr + ncols);
                        z2 = *(magptr + ncols + 1);

                        mag2 = (z1 - z2)*xperp + (z1 - m00)*yperp; 
                    }
                }
                else
                {
                    if (gx >= -gy)
                    {
                        /* 101 */
                        /* Left point */
                        z1 = *(magptr - 1);
                        z2 = *(magptr + ncols - 1);

                        mag1 = (m00 - z1)*xperp + (z1 - z2)*yperp;
            
                        /* Right point */
                        z1 = *(magptr + 1);
                        z2 = *(magptr - ncols + 1);

                        mag2 = (m00 - z1)*xperp + (z1 - z2)*yperp;
                    }
                    else
                    {    
                        /* 100 */
                        /* Left point */
                        z1 = *(magptr + ncols);
                        z2 = *(magptr + ncols - 1);

                        mag1 = (z1 - z2)*xperp + (m00 - z1)*yperp;

                        /* Right point */
                        z1 = *(magptr - ncols);
                        z2 = *(magptr - ncols + 1);

                        mag2 = (z1 - z2)*xperp  + (m00 - z1)*yperp; 
                    }
                }
            }
            else
            {
                if ((gy = *gyptr) >= 0)
                {
                    if (-gx >= gy)
                    {          
                        /* 011 */
                        /* Left point */
                        z1 = *(magptr + 1);
                        z2 = *(magptr - ncols + 1);

                        mag1 = (z1 - m00)*xperp + (z2 - z1)*yperp;

                        /* Right point */
                        z1 = *(magptr - 1);
                        z2 = *(magptr + ncols - 1);

                        mag2 = (z1 - m00)*xperp + (z2 - z1)*yperp;
                    }
                    else
                    {
                        /* 010 */
                        /* Left point */
                        z1 = *(magptr - ncols);
                        z2 = *(magptr - ncols + 1);

                        mag1 = (z2 - z1)*xperp + (z1 - m00)*yperp;

                        /* Right point */
                        z1 = *(magptr + ncols);
                        z2 = *(magptr + ncols - 1);

                        mag2 = (z2 - z1)*xperp + (z1 - m00)*yperp;
                    }
                }
                else
                {
                    if (-gx > -gy)
                    {
                        /* 001 */
                        /* Left point */
                        z1 = *(magptr + 1);
                        z2 = *(magptr + ncols + 1);

                        mag1 = (z1 - m00)*xperp + (z1 - z2)*yperp;

                        /* Right point */
                        z1 = *(magptr - 1);
                        z2 = *(magptr - ncols - 1);

                        mag2 = (z1 - m00)*xperp + (z1 - z2)*yperp;
                    }
                    else
                    {
                        /* 000 */
                        /* Left point */
                        z1 = *(magptr + ncols);
                        z2 = *(magptr + ncols + 1);

                        mag1 = (z2 - z1)*xperp + (m00 - z1)*yperp;

                        /* Right point */
                        z1 = *(magptr - ncols);
                        z2 = *(magptr - ncols - 1);

                        mag2 = (z2 - z1)*xperp + (m00 - z1)*yperp;
                    }
                }
            } 

            /* Now determine if the current point is a maximum point */

            if ((mag1 > 0.0) || (mag2 > 0.0))
            {
                *resultptr = (unsigned char) NOEDGE;
            }
            else
            {    
                if (mag2 == 0.0)
                    *resultptr = (unsigned char) NOEDGE;
                else
                    *resultptr = (unsigned char) POSSIBLE_EDGE;
            }
        } 
    }
}

//*******************************************************************************
//* FUNCTION: angle_radians
//* PURPOSE: This procedure computes the angle of a vector with components x and
//* y. It returns this angle in radians with the answer being in the range
//* 0 <= angle <2*PI.
//*******************************************************************************
double CDib::angle_radians(double x, double y)
{
   double xu, yu, ang;

   xu = fabs(x);
   yu = fabs(y);

   if((xu == 0) && (yu == 0)) return(0);

   ang = atan(yu/xu);

   if(x >= 0){
      if(y >= 0) return(ang);
      else return(2*M_PI - ang);
   }
   else{
      if(y >= 0) return(M_PI - ang);
      else return(M_PI + ang);
   }
}

//*******************************************************************************
//* PROCEDURE: magnitude_x_y
//* PURPOSE: Compute the magnitude of the gradient. This is the square root of
//* the sum of the squared derivative values.
//* NAME: Mike Heath
//* DATE: 2/15/96
//*******************************************************************************
void CDib::magnitude_x_y(short int *delta_x, short int *delta_y, int rows, int cols,
        short int **magnitude)
{
   int r, c, pos, sq1, sq2;

   //****************************************************************************
   //* Allocate an image to store the magnitude of the gradient.
   //****************************************************************************
   if((*magnitude = (short *) calloc(rows*cols, sizeof(short))) == NULL){
      fprintf(stderr, "Error allocating the magnitude image.\n");
      exit(1);
   }

   for(r=0,pos=0;r<rows;r++){
      for(c=0;c<cols;c++,pos++){
         sq1 = (int)delta_x[pos] * (int)delta_x[pos];
         sq2 = (int)delta_y[pos] * (int)delta_y[pos];
         (*magnitude)[pos] = (short)(0.5 + sqrt((float)sq1 + (float)sq2));
      }
   }

}

//*******************************************************************************
//* Procedure: radian_direction
//* Purpose: To compute a direction of the gradient image from component dx and
//* dy images. Because not all derriviatives are computed in the same way, this
//* code allows for dx or dy to have been calculated in different ways.
//*
//* FOR X:  xdirtag = -1  for  [-1 0  1]
//*         xdirtag =  1  for  [ 1 0 -1]
//*
//* FOR Y:  ydirtag = -1  for  [-1 0  1]'
//*         ydirtag =  1  for  [ 1 0 -1]'
//*
//* The resulting angle is in radians measured counterclockwise from the
//* xdirection. The angle points "up the gradient".
//*******************************************************************************
void CDib::radian_direction(short int *delta_x, short int *delta_y, int rows,
    int cols, float **dir_radians, int xdirtag, int ydirtag)
{
   int r, c, pos;
   float *dirim=NULL;
   double dx, dy;

   //****************************************************************************
   //* Allocate an image to store the direction of the gradient.
   //****************************************************************************
   if((dirim = (float *) calloc(rows*cols, sizeof(float))) == NULL){
      fprintf(stderr, "Error allocating the gradient direction image.\n");
      exit(1);
   }
   *dir_radians = dirim;

   for(r=0,pos=0;r<rows;r++){
      for(c=0;c<cols;c++,pos++){
         dx = (double)delta_x[pos];
         dy = (double)delta_y[pos];

         if(xdirtag == 1) dx = -dx;
         if(ydirtag == -1) dy = -dy;

         dirim[pos] = (float)angle_radians(dx, dy);
      }
   }
}

///*******************************************************************************
//* PROCEDURE: derivative_x_y
//* PURPOSE: Compute the first derivative of the image in both the x any y
//* directions. The differential filters that are used are:/
//*
//*                                          -1
//*         dx =  -1 0 +1     and       dy =  0
//*                                          +1
//*
//* NAME: Mike Heath
//* DATE: 2/15/96
//*******************************************************************************
void CDib::derivative_x_y(short int *smoothedim, int rows, int cols,
        short int **delta_x, short int **delta_y)
{
   int r, c, pos;

   //****************************************************************************
   //* Allocate images to store the derivatives.
   //****************************************************************************
   if(((*delta_x) = (short *) calloc(rows*cols, sizeof(short))) == NULL){
      fprintf(stderr, "Error allocating the delta_x image.\n");
      exit(1);
   }
   if(((*delta_y) = (short *) calloc(rows*cols, sizeof(short))) == NULL){
      fprintf(stderr, "Error allocating the delta_x image.\n");
      exit(1);
   }

   //****************************************************************************
   //* Compute the x-derivative. Adjust the derivative at the borders to avoid
   //* losing pixels.
   //****************************************************************************
// if(VERBOSE) printf("   Computing the X-direction derivative.\n");
   for(r=0;r<rows;r++){
      pos = r * cols;
      (*delta_x)[pos] = smoothedim[pos+1] - smoothedim[pos];
      pos++;
      for(c=1;c<(cols-1);c++,pos++){
         (*delta_x)[pos] = smoothedim[pos+1] - smoothedim[pos-1];
      }
      (*delta_x)[pos] = smoothedim[pos] - smoothedim[pos-1];
   }

   //****************************************************************************
   //* Compute the y-derivative. Adjust the derivative at the borders to avoid
   //* losing pixels.
   //****************************************************************************
//   if(VERBOSE) printf("   Computing the Y-direction derivative.\n");
   for(c=0;c<cols;c++){
      pos = c;
      (*delta_y)[pos] = smoothedim[pos+cols] - smoothedim[pos];
      pos += cols;
      for(r=1;r<(rows-1);r++,pos+=cols){
         (*delta_y)[pos] = smoothedim[pos+cols] - smoothedim[pos-cols];
      }
      (*delta_y)[pos] = smoothedim[pos] - smoothedim[pos-cols];
   }
}

//*******************************************************************************
//* PROCEDURE: canny
//* PURPOSE: To perform canny edge detection.
//* NAME: Mike Heath
//* DATE: 2/15/96
//*******************************************************************************
void CDib::canny(unsigned char *image, int rows, int cols, float sigma,
         float tlow, float thigh, unsigned char *edge, char *fname)
{
   FILE *fpdir=NULL;          //* File to write the gradient image to.     
   unsigned char *nms;        //* Points that are local maximal magnitude. 
   short int *smoothedim,     //* The image after gaussian smoothing.      
             *delta_x,        //* The first devivative image, x-direction. 
             *delta_y,        //* The first derivative image, y-direction. 
             *magnitude;      //* The magnitude of the gadient image.      
   //int r, c, pos;
   float *dir_radians=NULL;   //* Gradient direction image.                

   //****************************************************************************
   //* Perform gaussian smoothing on the image using the input standard
   //* deviation.
   //***************************************************************************
   //if(VERBOSE) printf("Smoothing the image using a gaussian kernel.\n");
   if((smoothedim = (short int *) calloc(rows*cols,
         sizeof(short int))) == NULL){
      fprintf(stderr, "Error allocating the smoothed image.\n");
      exit(1);
   }
   gaussian_smooth(image, rows, cols, sigma, smoothedim);

   //****************************************************************************
   //* Compute the first derivative in the x and y directions.
   //***************************************************************************
   //if(VERBOSE) printf("Computing the X and Y first derivatives.\n");
   derivative_x_y(smoothedim, rows, cols, &delta_x, &delta_y);

   //****************************************************************************
   //* This option to write out the direction of the edge gradient was added
   //* to make the information available for computing an edge quality figure
   //* of merit.
   //***************************************************************************
   if(fname != NULL){
      //*************************************************************************
      //* Compute the direction up the gradient, in radians that are
      //* specified counteclockwise from the positive x-axis.
      //************************************************************************
      radian_direction(delta_x, delta_y, rows, cols, &dir_radians, -1, -1);

      //*************************************************************************
      //* Write the gradient direction image out to a file.
      //************************************************************************
      //if((fpdir = fopen(fname, "wb")) == NULL){
	  errno_t err = fopen_s(&fpdir, fname, "wb");
      if (err) {
         fprintf(stderr, "Error opening the file %s for writing.\n", fname);
         exit(1);
      }
      fwrite(dir_radians, sizeof(float), rows*cols, fpdir);
      fclose(fpdir);
      free(dir_radians);
   }

   //****************************************************************************
   //* Compute the magnitude of the gradient.
   //***************************************************************************
   //if(VERBOSE) printf("Computing the magnitude of the gradient.\n");
   magnitude_x_y(delta_x, delta_y, rows, cols, &magnitude);

   //****************************************************************************
   //* Perform non-maximal suppression.
   //***************************************************************************
   //if(VERBOSE) printf("Doing the non-maximal suppression.\n");
   if((nms = (unsigned char *) calloc(rows*cols,sizeof(unsigned char)))==NULL){
      fprintf(stderr, "Error allocating the nms image.\n");
      exit(1);
   }

   non_max_supp(magnitude, delta_x, delta_y, rows, cols, nms);

   //****************************************************************************
   //* Use hysteresis to mark the edge pixels.
   //***************************************************************************
   //if(VERBOSE) printf("Doing hysteresis thresholding.\n");
   // 밖에서 Allocate하는 걸로 바꾸었음
   //if((*edge=(unsigned char *)calloc(rows*cols,sizeof(unsigned char))) ==NULL){
   //   fprintf(stderr, "Error allocating the edge image.\n");
   //   exit(1);
   //}

   apply_hysteresis(magnitude, nms, rows, cols, tlow, thigh, edge);

   //****************************************************************************
   //* Free all of the memory that we allocated except for the edge image that
   //* is still being used to store out result.
   //***************************************************************************
   free(smoothedim);
   free(delta_x);
   free(delta_y);
   free(magnitude);
   free(nms);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Gaussian Smoothing Main Functions
//
/////////////////////////////////////////////////////////////////////////////////////////////////
CDib *CDib::GaussianSmoothingG(float sigma)
{
    if (!IsGrayImage()) return NULL;

    CWDib wdib;
    //wdib.Allocate(Width(), Height());
    wdib.Allocate(ByteWidth(), Height());
    gaussian_smooth(GetPattern(), Height(), ByteWidth(), sigma, wdib.GetPattern());

    CDib *pRes = wdib.GetMappedCDib();
    pRes->UpsideDown();
   
    return pRes;
}

CDib *CDib::GaussianSmoothing(float sigma)
{
    if (IsGrayImage()) return GaussianSmoothingG(sigma);
    if (BitCount()<16) {
#ifdef _DEBUG
      AfxMessageBox("??? Gaussian Smoothing is possible only for Gray Image or Full Color Image");
#endif
      return NULL;
    }

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->GaussianSmoothingG(sigma);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//
//      DoG filter
//
/////////////////////////////////////////////////////////////////////////////////////////////////
CDib *CDib::DoG(CDib *pDib1, CDib *pDib2)
{
    CWDib *pTemp = new CWDib;
    pTemp->Allocate(pDib1->Width(), pDib1->Height());
    pTemp->ResetContents();

    for (int i=0; i<pDib1->Height() ;i++) {
      unsigned char *ptr1 = pDib1->GetPointer(0, i);
      unsigned char *ptr2 = pDib2->GetPointer(0, i);
      short *pO = pTemp->GetPointer(0, i);
      for (int j=0; j<pDib1->Width() ;j++) 
        *pO++ = ((short) *ptr1++ - (short) *ptr2++);
    }

    CDib *pRes = pTemp->GetMappedCDib();
    delete pTemp;

    return pRes;
}

CDib *CDib::AbsDoG(CDib *pDib1, CDib *pDib2)
{
    CWDib *pTemp = new CWDib;
    pTemp->Allocate(pDib1->Width(), pDib1->Height());
    pTemp->ResetContents();

    for (int i=0; i<pDib1->Height() ;i++) {
      unsigned char *ptr1 = pDib1->GetPointer(0, i);
      unsigned char *ptr2 = pDib2->GetPointer(0, i);
      short *pO = pTemp->GetPointer(0, i);
      for (int j=0; j<pDib1->Width() ;j++) 
        *pO++ = abs((int) ((short) *ptr1++ - (short) *ptr2++));
    }

    CDib *pRes = pTemp->GetMappedCDib();
    delete pTemp;

    return pRes;
}

CDib *CDib::DoGFilterG(double sigma1, double sigma2, BOOL bAbs, CDib **pSmooth1, CDib **pSmooth2)
{
    if (!IsGrayImage()) return NULL;

    CDib *pSm1 = GaussianSmoothingG((float) sigma1);
    CDib *pSm2 = GaussianSmoothingG((float) sigma2);

    CDib *pDoG = NULL;
    if (bAbs) pDoG = AbsDoG(pSm1, pSm2);
    else pDoG = DoG(pSm1, pSm2);

    if (pSmooth1) *pSmooth1 = pSm1;
    else delete pSm1;
    if (pSmooth2) *pSmooth2 = pSm2;
    else delete pSm2;

    return pDoG;
}

// 원본이 color건 흑백이건 상관없이, 무조건 Gray Image로 만들어 처리한다.
CDib *CDib::DoGFilterGray(double sigma1, double sigma2, BOOL bAbs, CDib **pSmooth1, CDib **pSmooth2)
{
    if (IsGrayImage()) return DoGFilterG(sigma1, sigma2, bAbs, pSmooth1, pSmooth2);

    CDib *pGray = GetGrayCDib();
    CDib *pRes = pGray->DoGFilterG(sigma1, sigma2, bAbs, pSmooth1, pSmooth2);
    delete pGray;

    return pRes;
}

CDib *CDib::DoGFilter(double sigma1, double sigma2, BOOL bAbs, CDib **pSmooth1, CDib **pSmooth2)
{
    if (IsGrayImage()) return DoGFilterG(sigma1, sigma2, bAbs, pSmooth1, pSmooth2);
    if (BitCount()<16) {
#ifdef _DEBUG
      AfxMessageBox("??? DoF Filter is applied only for Gray Image or Full Color Image");
#endif
      return NULL;
    }

    CDib *pY, *pU, *pV;
    RGBtoYUV(&pY, &pU, &pV);

    CDib *pYNew = pY->DoGFilterG(sigma1, sigma2, bAbs, pSmooth1, pSmooth2);

	CDib *pRes = YUVtoRGB(pYNew, pU, pV);

    delete pY;
    delete pYNew;
    delete pU;
    delete pV;

    return pRes;
}    

/////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Canny Edge Extraction Main Functions
//
/////////////////////////////////////////////////////////////////////////////////////////////////
CDib *CDib::EdgeB_Canny(float sigma, float tlow, float thigh)
{
    CDib *pIn=NULL;
    if (!IsGrayImage()) pIn = GetGrayCDib();
    else pIn = this;

    CDib *pRes = pIn->CopyCDib();
    pRes->ResetContents();

    canny(pIn->GetPattern(), pIn->Height(), pIn->ByteWidth(), sigma, tlow, thigh, pRes->GetPattern(), NULL);

    if (pIn!=this) delete pIn;

    return pRes;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Detect Harris Corner
//
/////////////////////////////////////////////////////////////////////////////////////////////////
/// <summary>
/// Process image looking for corners.
/// </summary>
/// 
/// <param name="image">Source image data to process.</param>
/// 
/// <returns>Returns list of found corners (X-Y coordinates).</returns>
/// 
/// <exception cref="UnsupportedImageFormatException">
///   The source image has incorrect pixel format.
/// </exception>
/// 
const float static SingleEpsilon = 1.1920929E-07f;
//public unsafe List<IntPoint> ProcessImage(UnmanagedImage image)
int CDib::HarrisCornerG(CPointArray& corners, int alg, float K, float threshold, int suppress_range)
{
    // check image format
    /*
    if (
        (image.PixelFormat != PixelFormat.Format8bppIndexed) &&
        (image.PixelFormat != PixelFormat.Format24bppRgb) &&
        (image.PixelFormat != PixelFormat.Format32bppRgb) &&
        (image.PixelFormat != PixelFormat.Format32bppArgb)
        )
    {
        throw new UnsupportedImageFormatException("Unsupported pixel format of the source image.");
    }
    */

    // make sure we have grayscale image
    /*
    if (image.PixelFormat == PixelFormat.Format8bppIndexed)
    {
        grayImage = image;
    }
    else
    {
        // create temporary grayscale image
        grayImage = Grayscale.CommonAlgorithms.BT709.Apply(image);
    }
    */
    CDib *pGray = NULL;
    if (IsGrayImage()) pGray = CopyCDib();
    else pGray = GetGrayCDib();


    // get source image size
    /*
    int width = grayImage.Width;
    int height = grayImage.Height;
    int srcStride = grayImage.Stride;
    int srcOffset = srcStride - width;
    */
    int width = pGray->Width();
    int height = pGray->Height();
    int srcStride = pGray->ByteWidth();
    int srcOffset = srcStride - width;

    // 1. Calculate partial differences
    /*
    float[,] diffx = new float[height, width];
    float[,] diffy = new float[height, width];
    float[,] diffxy = new float[height, width];
    */
    float *diffx = new float [height * width]; ZeroFloat(diffx, height*width);
    float *diffy = new float [height * width]; ZeroFloat(diffy, height*width);
    float *diffxy = new float[height * width]; ZeroFloat(diffxy, height*width);

    //fixed (float* pdx = diffx, pdy = diffy, pdxy = diffxy)
    float *pdx=diffx, *pdy=diffy, *pdxy=diffxy;
    {
        //byte* src = (byte*)grayImage.ImageData.ToPointer() + srcStride + 1;
        byte *src = (byte *) pGray->GetPattern() + srcStride + 1;

        // Skip first row and first column
        float* dx = pdx + width + 1;
        float* dy = pdy + width + 1;
        float* dxy = pdxy + width + 1;

        // for each line
        for (int y = 1; y < height - 1; y++)
        {
            // for each pixel
            for (int x = 1; x < width - 1; x++, src++, dx++, dy++, dxy++)
            {
                // Convolution with horizontal differentiation kernel mask
                float h = ((src[-srcStride + 1] + src[+1] + src[srcStride + 1]) -
                           (src[-srcStride - 1] + src[-1] + src[srcStride - 1])) * 0.166666667f;

                // Convolution vertical differentiation kernel mask
                float v = ((src[+srcStride - 1] + src[+srcStride] + src[+srcStride + 1]) -
                           (src[-srcStride - 1] + src[-srcStride] + src[-srcStride + 1])) * 0.166666667f;

                // Store squared differences directly
                *dx = h * h;
                *dy = v * v;
                *dxy = h * v;
            }

            // Skip last column
            dx++; dy++; dxy++;
            src += srcOffset + 1;
        }

        // Free some resources which wont be needed anymore
        //if (image.PixelFormat != PixelFormat.Format8bppIndexed)
        //    grayImage.Dispose();
        delete pGray;
    }


    // 2. Smooth the diff images
    float sigma = 1.2f;
    if (sigma > 0.0)
    {
        int windowsize;
        float *kernel = NULL;
        make_gaussian_kernel(sigma, &kernel, &windowsize);
        float *pTemp = new float[height * width];

        // Convolve with Gaussian kernel
        convolve(diffx, width, height, pTemp, kernel, windowsize);
        convolve(diffy, width, height, pTemp, kernel, windowsize);
        convolve(diffxy, width, height, pTemp, kernel, windowsize);

        delete [] pTemp;
    }


    // 3. Compute Harris Corner Response Map
    //float[,] map = new float[height, width];
    float *pMap = new float [height * width]; ZeroFloat(pMap, height*width);

//    fixed (float* pdx = diffx, pdy = diffy, pdxy = diffxy, pmap = map)
    //{
    float* dx = pdx;
    float* dy = pdy;
    float* dxy = pdxy;
    float* H = pMap;
    float M, A, B, C;
    //float threshold = 20000.f;

    int x, y;
    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++, dx++, dy++, dxy++, H++)
        {
            A = *dx;
            B = *dy;
            C = *dxy;

            //if (measure == HarrisCornerMeasure.Harris)
            //{
                // Original Harris corner measure
            //    M = (A * B - C * C) - (k * ((A + B) * (A + B)));
            //}
            //else
            //{
                // Harris-Noble corner measure
            //    M = (A * B - C * C) / (A + B + SingleEpsilon);
            //}
            switch (alg) {
              case HARRISCORNER_NOBLE:
                // Harris-Noble corner measure
                M = (A * B - C * C) / (A + B + SingleEpsilon);
                break;
              case HARRISCORNER_HARRIS:
              default:
                // Original Harris corner measure
                M = (A * B - C * C) - (K * ((A + B) * (A + B)));
            }


            if (M > threshold)
            {
                *H = M; // insert value in the map
            }
        }
    }
    //}


    // 4. Suppress non-maximum points
    //List<IntPoint> cornersList = new List<IntPoint>();

    corners.RemoveAll();
    int r = suppress_range;
    // for each row
    int maxX, maxY;
    for (y = r, maxY = height - r; y < maxY; y++)
    {
        // for each pixel
        for (x = r, maxX = width - r; x < maxX; x++)
        {
            //float currentValue = map[y, x];
            float currentValue = pMap[y*width+x];

            // for each windows' row
            for (int i = -r; (currentValue != 0) && (i <= r); i++)
            {
                // for each windows' pixel
                for (int j = -r; j <= r; j++)
                {
                    //if (map[y + i, x + j] > currentValue)
                    if (pMap[(y + i)*width + x + j] > currentValue)
                    {
                        currentValue = 0;
                        break;
                    }
                }
            }

            // check if this point is really interesting
            if (currentValue != 0)
            {
                //cornersList.Add(new IntPoint(x, y));
                CPoint pt(x, height-y);
                corners.Add(pt);
            }
        }
    }

    return corners.GetSize();
}

int CDib::HarrisCorner(CPointArray& corners, int alg, float K, float threshold, int suppress_range)
{
	if (IsGrayImage()) return HarrisCornerG(corners, alg, K, threshold, suppress_range);
	
	CDib *pGray = GetGrayCDib();
	int res = pGray->HarrisCornerG(corners, alg, K, threshold, suppress_range);
	delete pGray;

	return res;
}

/// <summary>
///   Convolution with decomposed 1D kernel.
/// </summary>
/// 
//private static void convolve(float[,] image, float[,] temp, float[] kernel)
void CDib::convolve(float *pImage, int width, int height, float *pTemp, float *pKernel, int wsize)
{
    //int width = image.GetLength(1);
    //int height = image.GetLength(0);
    //int radius = kernel.Length / 2;
    int radius = wsize/2;

    //unsafe
    //{
       //fixed (float* ptrImage = image, ptrTemp = temp)
    float *ptrImage = pImage, *ptrTemp = pTemp;
       //{

    float* src = ptrImage + radius;
    float* tmp = ptrTemp + radius;

    for (int y = 0; y < height; y++)
    {
        for (int x = radius; x < width - radius; x++, src++, tmp++)
        {
            float v = 0;
            //for (int k = 0; k < kernel.Length; k++)
            //    v += src[k - radius] * kernel[k];
            for (int k = 0; k < wsize; k++)
                v += src[k - radius] * pKernel[k];
            *tmp = v;
        }
        src += 2 * radius;
        tmp += 2 * radius;
    }


    for (int x = 0; x < width; x++)
    {
       for (int y = radius; y < height - radius; y++)
       {
            src = ptrImage + y * width + x;
            tmp = ptrTemp + y * width + x;

            float v = 0;
            //for (int k = 0; k < kernel.Length; k++)
            //    v += tmp[width * (k - radius)] * kernel[k];
            for (int k = 0; k < wsize ; k++)
                v += tmp[width * (k - radius)] * pKernel[k];
            *src = v;
        }
    }
        //}
     //}
}

void CDib::ZeroFloat(float *pData, int count)
{
    for (int i=0; i<count ;i++)
      *pData++ = 0.;
}

// Background란 boundary에 닿아 있는 Blob으로 Hole에 해당하지 않는다
void CDib::GetBackground(CDib *pDib, CObBlobArray *pBlobs, char flag[])
{
	CRect image(0, 0, pDib->Width(), pDib->Height());
    for (int i=0; i<pBlobs->GetSize() ;i++) {
      CObBlob *pB = pBlobs->GetAt(i);
	  CRect rect = pB->m_Region; rect.InflateRect(1,1);	// connected component analysis를 할 때 boundary 1 pixel을 지웠던 걸 감안해서, 1만큼 벌려준다.
	  
      if ((rect.left==image.left) || (rect.top==image.top) || (rect.right==image.right) || (rect.bottom==image.bottom)) flag[pB->m_Label] = 1;
      //if (pB->m_Count>500) flag[pB->m_Label] = 1;
    }
}

void CDib::FillHole()
{
	ASSERT(BitCount()==8);

	CDib *pDib = CopyCDib();
    pDib->Inverse();

	CWDib label;
    CObBlobArray *pBlobs = pDib->ConnectedComponentAnalysis(&label);
	char flag[10000]; memset(flag, 0, sizeof(flag));
    GetBackground(pDib, pBlobs, flag);
    delete pBlobs;

    for (int i=0; i<pDib->Height() ;i++) {
      unsigned char *ptr = pDib->GetPointer(0, i);
      unsigned char *pT = GetPointer(0, i);
	  short *pW = label.GetPointer(0, i);
      for (int j=0; j<pDib->Width() ;j++, ptr++, pT++, pW++) {
        if (*ptr) {
			if (flag[*pW]==0) *pT = 255;
        }
      }
    }

    delete pDib;
}

int CDib::CountPixel()
{
	ASSERT(BitCount()==8);

	int cnt=0;
	for (int i=0; i<Height() ;i++) {
		unsigned char *ptr = GetPointer(0, i);
		for (int j=0; j<Width() ;j++, ptr++)
			if (*ptr!=0) cnt++;
	}

	return cnt;
}

void CDib::RemoveNoisyBlobs(int threshold)
{
	ASSERT(BitCount()==8);

	CWDib Label;
	CObBlobArray *pBlobs = ConnectedComponentAnalysis(&Label);

	char flag[10000]; memset(flag, 0, sizeof(flag));
	for (int i=0; i<pBlobs->GetSize() ;i++) 
		if (pBlobs->GetAt(i)->m_Count>threshold) flag[pBlobs->GetAt(i)->m_Label] = 1;

	for (int i=0; i<Height() ;i++) {
		unsigned char *ptr = GetPointer(0, i);
		short *pW = Label.GetPointer(0, i);
		for (int j=0; j<Width() ;j++, ptr++, pW++) 
			if (flag[*pW]==1) *ptr = 255;
			else *ptr = 0;
	}

	delete pBlobs;
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Key word: twkim
//	CDib TwLibrary !!
//	written by t.w.kim
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////


// 입력 받은 마스크가 참인 부분만 그대로 출력하고 
// 나머지 부분은 0으로 처리
// 즉 마스크와 and연산한 것으로 보면 됨
CDib* CDib::GetTrueMaskedImage(CDib* mask)
{

	if ( (mask->Width()!=this->Width()) ||
		 (mask->Height()!=this->Height())) return NULL;
	

	CDib *pRes = this->CopyCDib();

	
	for (int i=0; i<mask->Height() ;i++) {
		unsigned char *pMask = mask->GetPointer(0,i);
		unsigned char *pSrc = this->GetPointer(0,i);
		unsigned char *ptr = pRes->GetPointer(0,i);
      for (int j=0; j<mask->Width() ;j++, pMask++, pSrc++) {
		  if(*pMask == 0) {
			  *ptr++ = 0;	// B
			  *ptr++ = 0;	// G
			  *ptr++ = 0;	// R
			  if(pRes->BitCount() == 32) *ptr++ = 0;
		  } else {
			  ++ptr;	// B
			  ++ptr;	// G
			  ++ptr;	// R
			  if(pRes->BitCount() == 32) ++ptr;
		  }
	  }
        
    }
	
	return pRes;
}


// Structure for region growing
typedef struct _neighbor {
	void neighbor(int ix, int iy, double ival)
	{
		x = ix;
		y = iy;
		val = ival;
	}

	int x;
	int y;
	double val;
}neighbor;



CDib* CDib::RegionGrowing(int seedx, int seedy, double thres)
{
	CDib* src;	// only gray scale image considered
	if(this->BitCount() == 8) {
		src = this->CopyCDib();
	} else if(this->BitCount() > 8) {
		src = this->GetGrayCDib();
	}
	
	int w = src->Width();
	int h = src->Height();
	int bc = src->BitCount();

	int sx = seedx;
	int sy = seedy;

	if(seedx < 0 && seedx >= w && seedy < 0 && seedy >= h) return NULL;

	// segmented binary image
	CDib* reg = new CDib;
	reg->Allocate(w, h, bc);
	reg->SetGrayPalette();


	unsigned char* ptr_reg = reg->GetPointer(seedx, seedy);
	unsigned char* ptr_src = src->GetPointer(seedx, seedy);
	int reg_mean = *ptr_src;	// The mean of the segmented region, initial value
	int reg_size = 1;			// number of pixels in region
	int neg_free = 10000;
	int neg_pos = -1;

	double pixdist = 0;
	double dist = 0;
	int nPixel = w*h;

	
	neighbor *neg_list = (neighbor*)calloc(neg_free, sizeof(neighbor));
	
	

	register int i,j;
	int neigb[4][2] = {-1, 0, 
						1, 0, 
						0,-1, 
						0, 1};
	//int count = 0;
	while(pixdist<thres && reg_size<nPixel)
	{
		// Add new neighbors pixels, up, down, left, right pixels
		for(j=0; j<4; j++)
		{
			// Calculate the neighbor corrdinate
			int xn = sx + neigb[j][0];
			int yn = sy + neigb[j][1];
			
			// Check if neighbor is inside or outside of the image
			bool ins = (xn>=0)&&(yn>=0)&&(xn<w)&&(yn<h);
			
			// Add neighbor if inside and not already part of the segmented area
			ptr_reg = reg->GetPointer(xn, yn);
			if(ins&&*ptr_reg == 0) {
				
				ptr_src = src->GetPointer(xn, yn);
				neg_pos++;

				neighbor nei; nei.neighbor(xn, yn, *ptr_src);	// 이웃(4방향)에 대한 좌표와 픽셀 값 저장
				neg_list[neg_pos] = nei;
				
				*ptr_reg = 200;	// rg image의 이웃의 위치에 값을 참으로
			}
		}


		// Add a new block of free memory
		if(neg_pos+10>neg_free)
		{
			neg_free+=10000;
			neg_list = (neighbor*)realloc(neg_list, neg_free*sizeof(neighbor));
		}


		int index = 0;
		double minVal = abs(neg_list[index].val - reg_mean);
		
		// Add pixel with intensity nearest to the mean of the region, to the region
		for(i=1; i<neg_pos+1; i++)
		{
			dist = abs((double)(neg_list[i].val - (double)reg_mean));
			if(minVal>dist) {
				minVal = dist;
				index = i;
			}
		}

		pixdist = minVal;
		
		ptr_reg = reg->GetPointer(sx, sy);
		*ptr_reg = 255;
		reg_size++;
				
		// Calculate the new mean of the region, ?????
		reg_mean = (reg_mean*reg_size + neg_list[index].val)/(reg_size+1);
		

		// Save the x and y coordinates of the pixel (for the neighbor add process)
		sx = neg_list[index].x;
		sy = neg_list[index].y;

		
		// Remove the pixel from the neighbor (check) list
		neg_list[index] = neg_list[neg_pos];
		neg_pos--;
		
	}

	//printf("count: %d, pixeldist: %lf, reg_size: %d, nPixel: %d \n", count, pixdist, reg_size, nPixel);
	

	delete src;
	delete [] neg_list;

	return reg;
}



CDib* CDib::UnsharpMask()
{
	CDib *src;
	if(BitCount() != 8) {
		src = GetGrayCDib();
	} else if(BitCount() == 8) {
		src = CopyCDib();
	}

	CDib *cpy = src->CopyCDib();


	register int x, y;


	for (y=1; y<Height()-1 ;y++) {
		unsigned char *pNew = src->GetPointer(1, y);
		
		for (x=1; x<Width()-1 ;x++, pNew++) {			
			*pNew = limit(5*(*cpy->GetPointer(x, y)) - *cpy->GetPointer(x, y-1)
				- *cpy->GetPointer(x-1, y) - *cpy->GetPointer(x, y+1) - *cpy->GetPointer(x+1, y));
		}
	}
	
	delete cpy;

	return src;
}


// Sobel operator를 이용해서 orientation을 얻는 코드
double CDib::GetSobelOrientation()
{
	CDib* pDib = NULL;
	if(BitCount() > 8)
		pDib = GetGrayCDib();
	else
		pDib = this;

	double rad=0.0, degree=0.0;
	double mag = 0.0;
	int width = Width();
	int height = Height();
	double ar_angle[180] = {0,};

// Edge Magnitude Sobel : (1,1) point is a center point.
	/*
	-1 -2 -1  &  -1  0  1
	 0  0  0     -2  0  2
	 1  2  1     -1  0  1
    */
	int sumx, sumy;
	for (int i=1; i<(Height()-1) ;i++) {
	  unsigned char *pU=GetPointer(1, i-1);
	  unsigned char *pC=GetPointer(1, i);
	  unsigned char *pD=GetPointer(1, i+1);
	  for (int j=1; j<(Width()-1) ;j++, pU++, pC++, pD++) {
	    sumy = -(*(pU-1)) -2*(*pU) -(*(pU+1)) 
		       +(*(pD-1)) +2*(*pD) +(*(pD+1));
		sumx = -(*(pU-1)) -2*(*(pC-1)) - (*(pD-1))
			   +(*(pU+1)) +2*(*(pC+1)) + (*(pD+1));

		mag = sqrt(pow((double)sumx,2)+pow((double)sumy,2));
		if(mag < 60) continue;


		// calc radian
		rad = atan2((double)sumy, (double)sumx);
		degree = rad*(180/PI);

		if(degree < 0) {
			degree = (int)(degree+180 + 0.5f);
		} else {
			degree = (int)(degree + 0.5f);
		}

		if(degree > 90)
			degree -= 90;

		if(degree >= 0 && degree < 179)
			ar_angle[(int)degree]+=mag;
		
	  }
	}
	
	// Get angle having maximum histogram
	double maxNum = 0.0;
	int index = 0;
	for(int i=0; i<180; i++)
	{
		if(ar_angle[i] > maxNum) {
			maxNum = ar_angle[i];
			index = i;
		}
	}

	rad = (double)index*(PI/180);

	return rad;
}




// 5x5 Sobel operator를 이용해서 orientation을 얻는 코드
double CDib::GetSobel5x5Orientation()
{
	CDib* pDib = NULL;
	if(BitCount() > 8)
		pDib = GetGrayCDib();
	else
		pDib = this;

	double rad=0.0, degree=0.0;
	double mag = 0.0;
	int width = Width();
	int height = Height();
	double ar_angle[180] = {0,};

// Edge Magnitude Sobel : (1,1) point is a center point.
	/*
	-1 -4  -6 -4 -1  &  1   2  0   -2  -1
	-2 -8 -12 -8 -2		4   8  0   -8  -4
	 0  0   0  0  0		6  12  0  -12  -6
	 2  8  12  8  2     4   8  0   -8  -4
	 1  4   6  4  1     1   2  0   -2  -1
    */
	int sumx, sumy;
	for (int i=2; i<(Height()-2) ;i++) {
	  unsigned char *pUU=GetPointer(1, i-2);
	  unsigned char *pU=GetPointer(1, i-1);
	  unsigned char *pC=GetPointer(1, i);
	  unsigned char *pD=GetPointer(1, i+1);
	  unsigned char *pDD=GetPointer(1, i+2);
	  for (int j=2; j<(Width()-2) ;j++, pUU++, pU++, pC++, pD++, pDD++) {
	    
		sumy = -(*(pUU-2)) -4*(*(pUU-1)) -6*(*pUU) -4*(*(pUU+1)) -(*(pUU+2)) 
			   -2*(*(pU-2))-8*(*(pU-1)) -12*(*pU)  -8*(*(pU+1))  -2*(*(pU+2)) 
			   +2*(*(pD-2))+8*(*(pD-1)) +12*(*pD)  +8*(*(pD+1))  +2*(*(pD+2)) 
			   -(*(pDD-2)) -4*(*(pDD-1)) -6*(*pDD) -4*(*(pDD+1)) -(*(pDD+2));

		sumx =  +(*(pUU-2))  +2*(*(pUU-1))  -2*(*(pUU+1))  -(*(pUU+2))
			   +4*(*(pU-2))  +8*(*(pU-1))   -8*(*(pU+1))  -4*(*(pU+2))
			   +6*(*(pC-2))  +12*(*(pC-1))  -12*(*(pC+1)) -6*(*(pC+2))
			   +4*(*(pD-2))  +8*(*(pD-1))   -8*(*(pD+1))  -4*(*(pD+2))
			    +(*(pDD-2))  +2*(*(pDD-1))  -2*(*(pDD+1))  -(*(pDD+2));
			   

		mag = sqrt(pow((double)sumx,2)+pow((double)sumy,2));
		if(mag < 50) continue;

		// calc radian
		rad = atan2((double)sumy, (double)sumx);
		
		degree = rad*(180/PI);

		if(degree < 0) {
			degree = (int)(degree+180 + 0.5f);
		} else {
			degree = (int)(degree + 0.5f);
		}

		if(degree > 90)
			degree -= 90;

		if(degree >= 0 && degree < 179)
			ar_angle[(int)degree]+=mag;
		
	  }
	}
	
	// Get angle having maximum histogram
	double maxNum = 0.0;
	int index = 0;
	for(int i=0; i<180; i++)
	{
		if(ar_angle[i] > maxNum) {
			maxNum = ar_angle[i];
			index = i;
		}
	}

	rad = (double)index*(PI/180);

	return rad;
}



CDib* CDib::GetSobelImage()
{
	
	if(BitCount() > 8)	return NULL;
	
	double mag = 0.0;
	int width = Width();
	int height = Height();

	CDib* pDib = new CDib;
	pDib->Allocate(width, height, BitCount());
	pDib->SetGrayPalette();

// Edge Magnitude Sobel : (1,1) point is a center point.
	/*
	-1 -2 -1  &  -1  0  1
	 0  0  0     -2  0  2
	 1  2  1     -1  0  1
    */
	int sumx, sumy;
	for (int i=1; i<(Height()-1) ;i++) {
	  unsigned char *pU=GetPointer(1, i-1);
	  unsigned char *pC=GetPointer(1, i);
	  unsigned char *pD=GetPointer(1, i+1);
	  unsigned char *ptr=pDib->GetPointer(1,i);
	  for (int j=1; j<(Width()-1) ;j++, pU++, pC++, pD++) {
	    sumy = -(*(pU-1)) -2*(*pU) -(*(pU+1)) 
		       +(*(pD-1)) +2*(*pD) +(*(pD+1));
		sumx = -(*(pU-1)) -2*(*(pC-1)) - (*(pD-1))
			   +(*(pU+1)) +2*(*(pC+1)) + (*(pD+1));

		mag = sqrt((double)(sumx*sumx + sumy*sumy));
		
		
		// TODO
		*ptr++= (short)mag;
	  }
	}
	
	return pDib;
}




CDib* CDib::GetSingleScaleRetinexImage(float sigma)
{
	if(BitCount() <= 8) { return NULL; }
	RGBQUAD rgb;
	CDib* src = this;
	
	// Get HSV image...(only Value image..)
	double h, s, v;
	int width = src->Width();
	int height = src->Height();
	int step = src->BitCount()/8;

	// Result image
	CDib* Res = new CDib;
	Res->Allocate(width, height, BitCount());
	Res->SetGrayPalette();



	// Hue
	double** H = new double*[height];
	for(int i=0; i<height; i++){
		H[i] = new double[width];
		memset(H[i], 0, sizeof(double)*width);
	}

	// Saturation
	double** S = new double*[height];
	for(int i=0; i<height; i++){
		S[i] = new double[width];
		memset(S[i], 0, sizeof(double)*width);
	}

	// Value
	CDib* V = new CDib;
	V->Allocate(width, height, 8);
	V->SetGrayPalette();

	// HSV image 생성
	register int x, y;
	for (y=0; y<height ;y++) {
		unsigned char *pV = V->GetPointer(0, y);
		unsigned char *pSrc = src->GetPointer(0, y);
		for (x=0; x<width ;x++, pV++, pSrc+=step) {			
			rgb.rgbBlue = *(pSrc+0);	rgb.rgbGreen = *(pSrc+1);	rgb.rgbRed = *(pSrc+2);
			RGBtoHSV(rgb, &h, &s, &v);
			H[y][x] = h;
			S[y][x] = s;
			*pV = (int)(v*255);
		}
	}


	// calc L and R
	CDib* L=V->GaussianSmoothingG(sigma);		// Lightness image
	

	double rc=1.0, lc=0.4;

	for(y=0; y<height; y++) {
		unsigned char *pV = V->GetPointer(0,y);
		unsigned char *pL = L->GetPointer(0,y);
		unsigned char *pRes = Res->GetPointer(0,y);

		for(x=0; x<width; x++, pV++, pL++, pRes+=step) {
			// log(I)-log(L)
			double tV = *pV;
			double tL = *pL;

			double R = log((double)(tV/255)) - log((double)(tL/255));
			double vv = exp(R*rc + log((double)(tL/255))*lc);
			rgb = HSVtoRGB(H[y][x], S[y][x], vv);

			*(pRes+0)=rgb.rgbBlue;
			*(pRes+1)=rgb.rgbGreen;
			*(pRes+2)=rgb.rgbRed;
		}
	}
	

	// 메모리 해제
	for(int i=0; i<height; i++) {
		delete [] H[i];
		delete [] S[i];
	}
	delete [] H;
	delete [] S;

	delete V;
	delete L;


	return Res;
}






// Distortion corrected image
// parameter들은 camera calibration을 통해서 구함
CDib* CDib::GetDistortionCorrectedImage(float fx, float fy, float skew, float cx, float cy, float k1, float k2)
{
	CDib* src = this;

	int width = src->Width();
	int height = src->Height();
	int bitcount = src->BitCount();
	

	CDib* Res = new CDib;
	Res->Allocate(width, height, bitcount);
	if(bitcount == 8) Res->SetGrayPalette();

	
	register int x, y;

	// 여기서 왜곡된 좌표라는 의미는 원래 렌즈 굴곡에 의해 왜곡된 이미지를 
	// 추정한 distortion 파라미터를 이용하여 다시 왜곡시킨 것을 의미한다. 
	// 즉 여기서의 왜곡은 원래 이미지를 원상태로 편 것을 의미하는 것과 마찬가지임..
	float y_nu, x_nu;	// normalized image 좌표
	float ru2;			// 중심점과의 거리
	float radial_d;		// 왜곡 모델
	float x_nd, y_nd;	// 왜곡된 normalized 좌표
	float x_pd, y_pd;	// 왜곡된 pixel 좌표
	float k3 = 0.0;
	
	int step = src->BitCount()/8;


	for(y=0; y<height; y++) {
		unsigned char *pSrc = src->GetPointer(0,y);
		unsigned char *pRes = Res->GetPointer(0,y);
		for(x=0; x<width; x++, pSrc+=step, pRes+=step) {
			
			// normalized 좌표
			y_nu = (y-cy)/fy;
			x_nu = (x-cx)/fx - skew*y_nu;

			// 중심까지의 거리 
			ru2 = x_nu*x_nu + y_nu*y_nu;
			radial_d = 1 + k1*ru2 + k2*ru2*ru2 + k3*ru2*ru2*ru2;	// 왜곡 모델

			// normalized image에서 왜곡된 좌표를 계산
			x_nd = radial_d*x_nu;
			y_nd = radial_d*y_nu;
			
			// 왜곡된 좌표를 다시 픽셀 좌표계로 변환
			x_pd = fx*(x_nd + skew*y_nd) + cx;
			y_pd = fy*y_nd + cy;

			if(x_pd >=0 && x_pd < width && y_pd >= 0 && y_pd < height) {
				*(pRes+0) = *(pSrc+0);
				*(pRes+1) = *(pSrc+1);
				*(pRes+2) = *(pSrc+2);
			}

		}
	}

	return Res;
}








/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// CDibArray
//
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
CDibArray::CDibArray()
{
}

CDibArray::~CDibArray()
{
	ResetContents();
}

void CDibArray::RemoveTail(int nFrom)
{
	int count=0;
	for (int i=nFrom; i<GetSize() ;i++) {
	  delete GetAt(i);
	  count++;
	}
	if (count>0) RemoveAt(nFrom, count);
}

void CDibArray::ResetContents()
{
	for (int i=0; i<GetSize() ;i++)
	  delete GetAt(i);
	RemoveAll();
}



#endif















// 코드 백업!!

//
//
//CDib* CDib::RegionGrowing(int seedx, int seedy, double thres)
//{
//	CDib* src;	// only gray scale image considered
//	if(this->BitCount() == 8) {
//		src = this->CopyCDib();
//	} else if(this->BitCount() > 8) {
//		src = this->GetGrayCDib();
//	}
//	
//	int w = src->Width();
//	int h = src->Height();
//	int bc = src->BitCount();
//
//	int sx = seedx;
//	int sy = seedy;
//
//	if(seedx < 0 && seedx >= w && seedy < 0 && seedy >= h) return NULL;
//
//	// segmented binary image
//	CDib* reg = new CDib;
//	reg->Allocate(w, h, bc);
//	reg->SetGrayPalette();
//
//
//	unsigned char* ptr_reg = reg->GetPointer(seedx, seedy);
//	unsigned char* ptr_src = src->GetPointer(seedx, seedy);
//	int reg_mean = *ptr_src;	// The mean of the segmented region, initial value
//	int reg_size = 1;			// number of pixels in region
//	int neg_free = 10000;
//	int neg_pos = 0;
//
//	double pixdist = 0;
//	double dist = 0;
//	int nPixel = w*h;
//
//	
//	
//	
//	list<neighbor> neg_list;
//	list<neighbor>::iterator it;
//	
//	
//
//	register int i,j;
//	int neigb[4][2] = {-1, 0, 
//						1, 0, 
//						0,-1, 
//						0, 1};
//	int count = 0;
//	while(pixdist<thres && reg_size<nPixel)
//	{
//		// Add new neighbors pixels, up, down, left, right pixels
//		for(j=0; j<4; j++)
//		{
//			// Calculate the neighbor corrdinate
//			int xn = sx + neigb[j][0];
//			int yn = sy + neigb[j][1];
//			
//			// Check if neighbor is inside or outside of the image
//			bool ins = (xn>=0)&&(yn>=0)&&(xn<w)&&(yn<h);
//			
//			// Add neighbor if inside and not already part of the segmented area
//			ptr_reg = reg->GetPointer(xn, yn);
//			if(ins&&*ptr_reg == 0) {
//				
//				ptr_src = src->GetPointer(xn, yn);
//
//				neighbor nei; nei.neighbor(xn, yn, *ptr_src);	// 이웃(4방향)에 대한 좌표와 픽셀 값 저장
//				
//				neg_list.push_back(nei);
//				
//				*ptr_reg = 200;	// rg image의 이웃의 위치에 값을 참으로
//			}
//		}
//
//		double minVal = 0;
//		int cnt = -1;
//		// Add pixel with intensity nearest to the mean of the region, to the region
//		for(it=neg_list.begin(); it!=neg_list.end(); it++) {
//			dist = abs((double)it->val - reg_mean);
//			//printf("xn: %d, yn: %d, val: %d\n", it->x, it->y, it->val);
//			//printf("dist: %d minVal: %d, val: %d, reg_mean: %d \n", dist, minVal, it->val, reg_mean);
//			if(minVal>dist) {
//				minVal = dist;
//				sx = it->x;		// Save the x and y coordinates of the pixel (for the neighbor add process)
//				sy = it->y;
//			}
//			cnt++;
//			
//		}
//		pixdist = dist;
//		
//		ptr_reg = reg->GetPointer(sx, sy);
//		*ptr_reg = 255;
//		reg_size++;
//				
//		// Calculate the new mean of the region, ?????
//		reg_mean = (reg_mean*reg_size + minVal)/(reg_size+1);
//		
//
//		// Save the x and y coordinates of the pixel (for the neighbor add process)
//		//sx = tmp_it->x;
//		//sy = tmp_it->y;
//
//		
//		// Remove the pixel from the neighbor (check) list
//		it=neg_list.begin();
//		while(cnt > 0) { it++; cnt--; }
//		if(!cnt) neg_list.erase(it);
//
//		count++;
//		
//	}
//
//	//printf("count: %d, pixeldist: %lf, reg_size: %d, nPixel: %d \n", count, pixdist, reg_size, nPixel);
//	
//
//	delete src;
//
//	return reg;
//}
