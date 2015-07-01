#pragma once
#include "stdafx.h"


// CConnectSocket 명령 대상입니다.


// Joint 개수가 늘어날 경우 아래 Coordinatelen 값을 1 늘려줘야 함...	
#define		COORDDATALEN			8*7	// sizeof(double)* num(mode, x, y, z, rx, ry, rz)

//#define COLORBOXDATALEN 56			// origin
#define		DOF					6+1		// 6DOF + 1 mode 

#define		NUMCOORD			20		// The number of coordinate
#define		RECVLEN				20*8	// Cartesian coordinate, joint, on/off, and etc...



class CConnectSocket : public CSocket
{
public:
	CConnectSocket();
	virtual ~CConnectSocket();
	virtual void OnClose(int nErrorCode);
	virtual void OnReceive(int nErrorCode);


public:
	BOOL initClientSocket(CString addr, int port);
	void SendCoordData(Coordi* rPoint);				// 로봇 좌표 데이터 전송
	void ReceiveData(double *pData);
	void CloseSocket();

public:
	WSADATA wsaData;
	SOCKET m_client_sock;
	SOCKADDR_IN ROBOTADR;


private:
	double Data[DOF];
	double recvData[RECVLEN];
};


