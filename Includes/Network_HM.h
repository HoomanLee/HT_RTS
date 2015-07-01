/*
 * Robot Teaching Library
 * Copyright (C) 2014 ETRI, South Korea
 * Author: Hooman Lee
 * email:  lhm85@snu.ac.kr
 * website: https://sites.google.com/site/hoomanleerobot/
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#pragma once

#include <iostream>
#include <stdlib.h>
#include <string.h>

#pragma comment(lib, "ws2_32.lib")

using namespace std;

#define  ERRORDATA			    100
#define  JOINTDATA			    200
#define  POSEDATA				300
#define  VISIONDATA				400
#define  POSEONLYDATA			500
#define  ORIONLYDATA			600
#define  POS_ORIDATA			700
#define  CONTAINER				800

#define  SOCK_CONNECT			301

#define  ERRDATALEN				56
#define  POSEDATALEN			48
#define  POSEONLYDATALEN		32//24였는데 button 추가
#define  ORIONLYDATALEN		    32
#define  JOINTDATALEN			48
#define  VISIONDATALEN			128 // RGBY 박스 순서 각각 x, y, z, rz 4DOF 정보
#define  POS_ORIDATALEN			56
#define  rPOS_ORIDATALEN		56
#define	 CONTDATALEN			160

// #define  Recv_FTSens_Buff_Size	24
// #define  Send_FTSens_Buff_size  24

#define  Recv_Parameters		56
#define  Send_Data_Count		56


#define DESIRED_WINSOCK_VERSION        0x0202
#define MINIMUM_WINSOCK_VERSION        0x0001
 
#define MAXBUF 256

class Net_Class
{
public:
	Net_Class(void);
	~Net_Class(void);

	void CreateServer(void);
	void server_connect(void);
	void server_senddata(int type, double Data[]);
	void server_recvdata(int type, double Data[]);

	int Net_Prepare_ERRORDATA(void);
	int Net_Prepare_POSITION(void);
	int Net_Prepare_JOINT(void);
	int Net_Prepare_VISION(void);
	void RecvData(int type, double Data[]);
	void SendData(int type, double Data[]);
	int SendData_Teaching(int DataLen, double Data[]);
	void Net_Disconnect(void);

	int connect_HM(char * szServerIP, int datatype, int portnum);
	int connect_Teaching(char * szServerIP, int portnum);

	int SendData_TBtnCMD(double Data);
	int connect_TBtn(char * szServerIP, int portnum);

public:
	bool bConnected;
};