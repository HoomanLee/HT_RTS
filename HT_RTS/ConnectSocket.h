#pragma once
#include "stdafx.h"


// CConnectSocket ��� ����Դϴ�.


// Joint ������ �þ ��� �Ʒ� Coordinatelen ���� 1 �÷���� ��...	
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
	void SendCoordData(Coordi* rPoint);				// �κ� ��ǥ ������ ����
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


