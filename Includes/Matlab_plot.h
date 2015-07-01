/*
 * Robot Teaching Library
 * Copyright (C) 2014-2016 ETRI, South Korea
 * Author: Hooman Lee, Taewoo Kim
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

#pragma comment (lib, "libmx.lib")
#pragma comment (lib, "libmex.lib")
#pragma comment (lib, "libeng.lib")
//
#include <engine.h>
//#include <iostream>
#include "Precompiled.h" // 컴파일 시간 단축을 위해..

using namespace std;

class Matlab_plot
{
public:
	Matlab_plot();
	~Matlab_plot();

	void initialize();

	void setplotData(double _Xdata[], double _Ydata[]);
	void drawnow(double _Xdata[], double _Ydata[]);

	void destroy();

public:
	Engine* m_pEngine;

	mxArray *T;
	mxArray *result;
};
