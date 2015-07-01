// ConnectSocket.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "ConnectSocket.h"
#include "HT_RTSDlg.h"

#include <WinSock2.h>
#pragma comment(lib, "ws2_32.lib")


// CConnectSocket

CConnectSocket::CConnectSocket()
{
}

CConnectSocket::~CConnectSocket()
{
}


// CConnectSocket 멤버 함수


void CConnectSocket::OnClose(int nErrorCode)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	ShutDown();
	Close();


	CSocket::OnClose(nErrorCode);

	AfxMessageBox("ERROR: Disconnected from server!");
	::PostQuitMessage(0);
}


void CConnectSocket::OnReceive(int nErrorCode)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	TCHAR szBuffer[1024];
	::ZeroMemory(szBuffer, sizeof(szBuffer));

	if(Receive(szBuffer, sizeof(szBuffer))>0)
	{
		// 메시지를 받으면 적절한 처리... 일단은 그냥 프린트!
		printf("Received message: %s \n", szBuffer);	// TODO 
	}

	CSocket::OnReceive(nErrorCode);
}



BOOL CConnectSocket::initClientSocket(CString addr, int port)
{
	// Socket initialization
	while(WSAStartup(MAKEWORD(2,2), &wsaData)!=0);	// Load winsock 2.2 DLL

	memset(&ROBOTADR, 0, sizeof(ROBOTADR));
	ROBOTADR.sin_family = AF_INET;
	ROBOTADR.sin_port = htons(port);	// Port number setting
	ROBOTADR.sin_addr.s_addr = inet_addr(addr);

	//VISIONADR.sin_addr.s_addr = inet_addr("192.168.0.254");		// Staubli IP
	//VISIONADR.sin_addr.s_addr = inet_addr("127.0.0.1");			// Emulator IP
	//VISIONADR.sin_addr.s_addr = inet_addr("129.254.90.68");		// My PC

	
	m_client_sock = socket(PF_INET, SOCK_STREAM, 0);
	if(m_client_sock==INVALID_SOCKET)
	{
		printf("invalid socket\n");
		return SOCKET_ERROR;
	}

	if(connect(m_client_sock, (SOCKADDR*)&ROBOTADR, sizeof(ROBOTADR))==SOCKET_ERROR)
	{
		printf("socket error\n");
		return SOCKET_ERROR;
	}

	return TRUE;
}



void CConnectSocket::SendCoordData(Coordi* rPoint)
{
	
	int i, j;
	
	if(rPoint->inputmode != PC) {
		printf("Input mode should be PC..\n");
		return;
	}

	for(i=0, j=0; i<DOF; i++, j+=DOF-1)
	{
		Data[i+j+0] = rPoint[i].coordflag;
		Data[i+j+1] = rPoint[i].x;
		Data[i+j+2] = rPoint[i].y;
		Data[i+j+3] = rPoint[i].z;
		Data[i+j+4] = rPoint[i].rx;
		Data[i+j+5] = rPoint[i].ry;
		Data[i+j+6] = rPoint[i].rz;
	}


	int DataLen = 0, sendCnt = 0;

	DataLen = COORDDATALEN;
    {
            char Out_Sensor_Data[COORDDATALEN] = {0,};

            memcpy(Out_Sensor_Data, Data, COORDDATALEN);
            while(DataLen - sendCnt)
            {
                    sendCnt += send(m_client_sock, &Out_Sensor_Data[sendCnt], DataLen - sendCnt, 0);
            }
    }
}



void CConnectSocket::ReceiveData(double *pData)
{
	int DataLen = 0, sendCnt = 0, recvCnt = 0;
	char In_Data[RECVLEN*sizeof(double)] = {0,};

	DataLen = RECVLEN;
	while(DataLen - recvCnt)
	{
		recvCnt += recv(m_client_sock, &In_Data[recvCnt], DataLen - recvCnt, 0);
	}
	memcpy(recvData, In_Data, RECVLEN);


	for(int i=0; i<NUMCOORD; i++)
	{
		pData[i] = recvData[i];
	}

	//// print received data...
	/*for(int i=0; i<7; i++)
		printf("%.2lf ", pData[i]);

	printf("\n");*/

}


void CConnectSocket::CloseSocket()
{
	closesocket(m_client_sock);
	WSACleanup();
}
