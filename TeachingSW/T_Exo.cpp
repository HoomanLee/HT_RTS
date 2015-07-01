#include "Precompiled.h"  
#include "T_Exo.h"

T_Exo::T_Exo()
{
	connect_stats = false;
	daytatype = 0;
	connection_type = 3; //1 = POSEONLY, 2 = ORIONLY, 3 = POS_ORI 
	connection_mode = 100; //100 = studio, 200 = robot
	memset(temp_joint, 0.0, 6*sizeof(double));  // 원래 사이즈보다 하나 늘렸음.. button 때문에.. Network_HM.cpp도 수정했음!
	memset(temp_point, 0.0, 4*sizeof(double));  // 원래 사이즈보다 하나 늘렸음.. button 때문에.. Network_HM.cpp도 수정했음!
	memset(temp_ori, 0.0, 4*sizeof(double));    // 원래 사이즈보다 하나 늘렸음.. button 때문에.. Network_HM.cpp도 수정했음!
	memset(temp_pos_ori, 0.0, 7*sizeof(double));// 원래 사이즈보다 하나 늘렸음.. button 때문에.. Network_HM.cpp도 수정했음!
	button = -1; // SWITCH_L_X 값이 0임

	bQuit = false;

	v.resize(31);
	buttonclk = 0;
	currclk = 0;    

	scale = 0;

	exoHT_ee_R.setIdentity();
	exoHT_ee_L.setIdentity();

	//공용변수
	shared_v;
	memset(mcu_data, 0, MCUDATALEN*sizeof(BYTE));
	data_analyzed = false;
}
T_Exo::~T_Exo()
{
}

bool T_Exo::Init_Exo(wchar_t* portname)
{
	if(_serial.OpenPort(portname, CBR_115200, 8, ONESTOPBIT , NOPARITY)){   // 실제 사용될 COM Port 를 넣어야합니다.  
		cout<<"Exo was connected!"<<endl;	
		connect_stats = true;
		return true;
	}
	else
		return false;
}

bool T_Exo::Discnt_Exo()
{
	if(_serial.m_bConnected){   // 실제 사용될 COM Port 를 넣어야합니다.  
		_serial.ClosePort();
		cout<<"Exo was disconnected!"<<endl;	
		connect_stats = false;
		return true;
	}
	else
		return false;
}

void T_Exo::DAQ_Exo(vector<float> &Exodata, Matrix4d &HTransform_EE_R, Matrix4d &HTransform_EE_L, Vector3d &pos_elbo_R, Vector3d &pos_elbo_L, int &_button)
{
	int iSize =  (_serial.m_QueueRead).GetSize(); 
	//cout<<iSize<<endl;
	for(int i  = 0 ; i < iSize; i++)//들어온 갯수 만큼 데이터를 읽어 와 화면에 보여줌
	{
		(_serial.m_QueueRead).GetByte(&aByte);//큐에서 데이터 한개를 읽어옴

		if(aByte == '!')
		{
			int dotcount = 0;
			for(int i=0; i<200; i++)
			{
				if(mcu_data[i] == '.')
					dotcount++;
			}
			if(dotcount == 31)
			{
				//puts(ptr); // 잘 나오는 지 찍어보기.
				
				sscanf((const char *)mcu_data, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
					&v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15],
					&v[16], &v[17], &v[18], &v[19], &v[20], &v[21], &v[22], &v[23], &v[24], &v[25], &v[26], &v[27], &v[28], &v[29], &v[30]);
					
				//cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<" "<<v[3]<<" "<<v[4]<<" "<<v[5]<<" "<<v[6]<<" "<<v[7]<<" "<<v[8]<<" "<<v[9]<<" "<<v[10]<<" "<<v[11]<<" "<<v[12]<<" "<<v[13]<<" "<<v[14]<<" "<<v[15]<<" "<<
				//	v[16]<<" "<<v[17]<<" "<<v[18]<<" "<<v[19]<<" "<<v[20]<<" "<<v[21]<<" "<<v[22]<<" "<<v[23]<<" "<<v[24]<<" "<<v[25]<<" "<<v[26]<<" "<<v[27]<<" "<<v[28]<<" "<<v[29]<<" "<<v[30]<<endl;
				
				for(int i=0; i<3; i++){
					for(int j=0; j<3; j++){
						HTransform_EE_R(i, j) = (double)v[i*4 + j];
						HTransform_EE_L(i, j) = (double)v[((i+3)*4+3) + j];
					}
					HTransform_EE_R(i, 3) = (double)v[i*4 + 3];
					HTransform_EE_L(i, 3) = (double)v[((i+3)*4+3) + 3];
					pos_elbo_R(i) = (double)v[12+i];
					pos_elbo_L(i) = (double)v[(12*2+3) +i];
				}
				
				unsigned char switchInput = 0;
				switchInput = (char)v[30];
				//// 클러치는 따로 분기문을 설정해 준다. 
				if((switchInput >> SWITCH_L_FRONT)&0x01 && (switchInput >> SWITCH_R_FRONT)&0x01){
					//cout<<"Both clutch on!!"<<endl;
					_button = SWITCH_L_FRONT + SWITCH_R_FRONT;
				}
				else if((switchInput >> SWITCH_L_FRONT)&0x01){
					//cout<<"Switch SWITCH_L_FRONT on"<<endl;
					_button = SWITCH_L_FRONT;
				}
				else if((switchInput >> SWITCH_R_FRONT)&0x01){
					//cout<<"Switch SWITCH_R_FRONT on"<<endl;
					_button = SWITCH_R_FRONT;
				}

				if((switchInput >> SWITCH_L_X)&0x01){
					//cout<<"Switch SWITCH_L_X on"<<endl;
					_button = SWITCH_L_X;
				}
				if((switchInput >> SWITCH_L_TRI)&0x01){
					//cout<<"Switch SWITCH_L_TRI on"<<endl;
					_button = SWITCH_L_TRI;
				}
				if((switchInput >> SWITCH_L_SQR)&0x01){
					//cout<<"Switch SWITCH_L_SQR on"<<endl;
					_button = SWITCH_L_SQR;
				}
				if((switchInput >> SWITCH_R_X)&0x01){
					//cout<<"Switch SWITCH_R_X on"<<endl;
					_button = SWITCH_R_X;
				}
				if((switchInput >> SWITCH_R_TRI)&0x01){
					//cout<<"Switch SWITCH_R_TRI on"<<endl;
					_button = SWITCH_R_TRI;
				}
				if((switchInput >> SWITCH_R_SQR)&0x01){
					//cout<<"Switch SWITCH_R_SQR on"<<endl;
					_button = SWITCH_R_SQR;
				}
				
				if(!(switchInput >> SWITCH_L_X)&0x01 && !(switchInput >> SWITCH_L_TRI)&0x01 && !(switchInput >> SWITCH_L_SQR)&0x01 
					&& !(switchInput >> SWITCH_L_FRONT)&0x01 && !(switchInput >> SWITCH_R_X)&0x01 && !(switchInput >> SWITCH_R_TRI)&0x01
					&& !(switchInput >> SWITCH_R_SQR)&0x01 && !(switchInput >> SWITCH_R_FRONT)&0x01){
					//cout<<"no button!"<<endl;
					_button = -1;
				}

				//cout<<"rx : "<<v[3]<<"	ry : "<<v[7]<<"	rz : "<<v[11]<<endl;
				//cout<<"lx : "<<v[18]<<"	ly : "<<v[22]<<"	lz : "<<v[26]<<endl;
			}
			commIndex = 0;
			for(int i=0; i<200; i++)
				mcu_data[i] = 0;
		}
		else
		{
			mcu_data[commIndex++] = aByte;
		}
	}
	Exodata.resize(v.size());
	Exodata = v;
}

// Exo와 Robot 좌표계 calibration.. orientation은 그냥 맞춰진다고 생각 함..
// 그런데 양팔일지 여러 팔일 지 뭐 알 수 없는 노릇이니.. 그리고 ori도 장기적으로 고려해서 
// 일단은 matrix로 만들어주고, 양팔까지만 고려해서 4*4 매트릭스 변수를 2 개 만들어주자.
// 그리고 이렇게 경우에 따라 달라지는 것들은 함수 오버로딩을 하자.
void T_Exo::Calib_exo(Vector3d robotHome, Vector3d exoHome)
{
	calib_robotHome.col(3).segment(0, 3) = robotHome;
	//calib_exoHome.col(3).segment(0, 3) = exoHome;
}
void T_Exo::Calib_exo(Vector3d robotHome, Vector3d exoHome, Vector3d robotHome2, Vector3d exoHome2)
{
	calib_robotHome_L.col(3).segment(0, 3) = robotHome;
	calib_exoHome_L.col(3).segment(0, 3) = exoHome;
	calib_robotHome_R.col(3).segment(0, 3) = robotHome2;
	calib_exoHome_R.col(3).segment(0, 3) = exoHome2;
}

// Exo와 Robot 좌표계 mapping.. 근데 Transform이 로봇마다 다른 데 어떻게 해 줄까. => 파라미터로 받자.(rotation_mapper)
//사용하는 단위계 차이 보정 파라미터도 집어넣자
void T_Exo::RHmapper_exo(Matrix4d& rHTranform_target, Matrix3d rotation_mapper, Vector3d robotHome, Vector3d exoHome, Vector3d exoPos, Matrix3d exoOri, double scale)
{
	//dMatrix frame_mapping1(3, 3);
	//frame_mapping1 = Rotate_with_X(M_PI/2);

	//dMatrix frame_mapping2(3, 3);
	//frame_mapping2 = Rotate_with_Y(M_PI/2);

	//dMatrix frame_mapping3(3, 3);
	//frame_mapping3 = Rotate_with_Z(M_PI);//Rotate_with_Z(-M_PI/2);

	//dMatrix frame_mapping1_trans(3, 3);
	//frame_mapping1.transpose(frame_mapping1_trans);
	//dMatrix frame_mapping2_trans(3, 3);
	//frame_mapping2.transpose(frame_mapping2_trans);
	//dMatrix frame_mapping3_trans(3, 3);
	//frame_mapping3.transpose(frame_mapping3_trans);

	///*사용하는 단위계 차이 보정*/
	//double unit_compensate = 1.0/1000.0;

	//rHTranform_target.col(3).segment(0, 3) = (exoPos - calib_exoHome)*units_compensate + calib_robotHome;
	//Exo와 Amiro같은 경우에는 모든 것을 맞추었기 때문에 아무것도 맞춰줄 필요가 없었음.
	rHTranform_target.col(3).segment(0, 3) = scale*rotation_mapper*(exoPos - exoHome) + robotHome;
	rHTranform_target.col(3).segment(0, 3) = scale*rotation_mapper*exoPos;
	rHTranform_target.topLeftCorner(3, 3) = rotation_mapper*exoOri; 
}

// Exo와 Staubli 좌표계 mapping
void T_Exo::Staubli_Exo_mapper(vector <double> &rPos_target, vector <double> &rOri_target, vector <double> robotHome, vector <double> exoHome, vector <float> exoData)
{
	vector <double> exo_home;
	exo_home.resize(3);
	exo_home = exoHome;

	vector <double> temp_point_vector;
	temp_point_vector.resize(3);
	vector <double> temp_ori_vector;
	temp_ori_vector.resize(9);

	for(int i=0; i<3; i++)
		temp_point_vector[i] = (double)exoData[((i+3)*4+3) + 3];; // 왼 손 position
	
	for(int i=0; i<3; i++){
		temp_ori_vector[i] = (double)exoData[15 + i];; // 왼 손 orientation
		temp_ori_vector[3+i] = (double)exoData[19 + i];; // 왼 손 orientation
		temp_ori_vector[6+i] = (double)exoData[23 + i];; // 왼 손 orientation
	}
	
	/*사용하는 단위계 차이 보정*/
	double unit_compensate = 1.0;

	rPos_target.resize(3);
	for(int i=0; i<3; i++)
		rPos_target[i] = (temp_point_vector[i] - exoHome[i])*unit_compensate + robotHome[i];

	rOri_target.resize(9);
	rOri_target = temp_ori_vector;
}

void T_Exo::MakestaubliOri(vector <double> rExoOri, vector <double> rExoOriHome, vector <double> & rStaubliOri)
{
	//좌표계 보정부터 다시. z

	//방.법 X-Y-Z Euler angle (refernce manual 보면 staubli 는 이거 사용 하는 것 처럼 보임)
	double alpha = 0.0;
	double beta = 0.0;
	double gamma = 0.0;

	beta = atan2(rExoOri[2], sqrt(rExoOri[5]*rExoOri[5]+rExoOri[8]*rExoOri[8]));
    gamma = atan2(rExoOri[1]/cos(beta), -rExoOri[0]/cos(beta));
	alpha = atan2(-rExoOri[5]/cos(beta), rExoOri[8]/cos(beta));

	double pi = 3.141592;

	//문제가 하나 있는데, Exo 같은 경우에는 초기 위치로 생각할 것이 마땅치 않은데..
	//우선은 왼 팔을 편하게 땅과 평행 방향으로 두었을 때의 방향을 초기 위치라고 해 두자.
	// 임시로 그냥 팔걸이에 걸었을 때 값으로 캘리브레이션 해보자.
	// 근데 잘 생각해보자... 그러면 직관적이지 못할 수 있다. 시작하자 마자 그 오리엔테이션에 맞추려면...
	// 차라리 축은 맞춰주는 것이 나을 수도 있다.
	vector <double> exo_orihome(3);  
 	exo_orihome[0] = -164.814; 
	exo_orihome[1] = 43.5702;
	exo_orihome[2] = 67.3195;


	vector <double> staubli_orihome(3);
	staubli_orihome[0] = -180.0;
	staubli_orihome[1] =0.0;
	staubli_orihome[2] = -180.0;
	
	vector <double> ori_exo(3);
	ori_exo[0] = alpha * 180/pi;
	ori_exo[1] = beta * 180/pi;
	ori_exo[2] = gamma * 180/pi;

	rStaubliOri.resize(3);
	for(int i=0; i<3; i++)
		rStaubliOri[i] = ori_exo[i] - rExoOriHome[i] + staubli_orihome[i];

	static int printer_count = 0;
	if (printer_count == 10){
		std::cout<<"4."<<alpha * 180/pi<<" "<<beta * 180/pi<<" "<<gamma * 180/pi<<std::endl; // 이거는 exo의 값을 staubli 방식으로 해석한 것.
		std::cout<<rStaubliOri[0]<<" "<<rStaubliOri[1]<<" "<<rStaubliOri[2]<<std::endl;
		printer_count = 0;
	}
	printer_count++;

	//// 아래가 Staubli LLI 코드에서 가져온 원본인데, 참고할 필요가 있음.
	//double l_sinRy;

	//l_sinRy = x_fr->m_ax;
	//// ATTENTION : it may be possible that sinRy > 1.0 or < -1.0 (numerical pbm)
	//if(l_sinRy < (-1.0 + S_SMALL_FLOAT*S_SMALL_FLOAT/2.0))
	//{
	//	*x_Rx = 0;
	//	*x_Ry = -S_PI/2.0;
	//	*x_Rz = atan2(x_fr->m_ny, x_fr->m_nz);
	//} else if(l_sinRy > (1.0 - S_SMALL_FLOAT*S_SMALL_FLOAT/2.0))
	//{
	//	*x_Rx = 0;
	//	*x_Ry = S_PI/2.0;
	//	*x_Rz = atan2(x_fr->m_ny, -x_fr->m_nz);
	//} else
	//{
	//	double l_sign = 1.0;
	//	*x_Ry = asin(l_sinRy);
	//	if((x_fr->m_az < 0.0) && (x_fr->m_nx < 0.0)
	//	   &&((fabs(l_sinRy)>S_VERY_SMALL_FLOAT)||(fabs(x_fr->m_ay)>S_VERY_SMALL_FLOAT)))
	//	{	// if cosRy > 0, and both cosRx and cosRz are < 0, choose Ry so that
	//		// cosRx and cosRz are positive
	//		if(*x_Ry >= 0.0)
	//			*x_Ry = S_PI - *x_Ry;
	//		else
	//			*x_Ry = -S_PI - *x_Ry;
	//		l_sign = -1.0;
	//	}
	//	*x_Rx = atan2(-l_sign * x_fr->m_ay, l_sign * x_fr->m_az);
	//	*x_Rz = atan2(-l_sign * x_fr->m_ox, l_sign * x_fr->m_nx);
	//}
}

////// get orientation as rx, ry, rz according to the x-y-z fixed angle theory
//void T_Exo::GetExoOri(vector <double> rExoOri, vector <double>& rxryrz)
//{
//	rxryrz.resize(3);
//		
//	//방.법 X-Y-Z Euler angle (refernce manual 보면 staubli 는 이거 사용 하는 것 처럼 보임)
//	double alpha = 0.0;
//	double beta = 0.0;
//	double gamma = 0.0;
//
//	beta = atan2(rExoOri[2], sqrt(rExoOri[5]*rExoOri[5]+rExoOri[8]*rExoOri[8]));
//    gamma = atan2(rExoOri[1]/cos(beta), -rExoOri[0]/cos(beta));
//	alpha = atan2(-rExoOri[5]/cos(beta), rExoOri[8]/cos(beta));
//
//	rxryrz[0] = alpha * 180/M_PI;
//	rxryrz[1] = beta * 180/M_PI;
//	rxryrz[2] = gamma * 180/M_PI;
//}
