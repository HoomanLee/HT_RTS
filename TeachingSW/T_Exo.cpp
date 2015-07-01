#include "Precompiled.h"  
#include "T_Exo.h"

T_Exo::T_Exo()
{
	connect_stats = false;
	daytatype = 0;
	connection_type = 3; //1 = POSEONLY, 2 = ORIONLY, 3 = POS_ORI 
	connection_mode = 100; //100 = studio, 200 = robot
	memset(temp_joint, 0.0, 6*sizeof(double));  // ���� ������� �ϳ� �÷���.. button ������.. Network_HM.cpp�� ��������!
	memset(temp_point, 0.0, 4*sizeof(double));  // ���� ������� �ϳ� �÷���.. button ������.. Network_HM.cpp�� ��������!
	memset(temp_ori, 0.0, 4*sizeof(double));    // ���� ������� �ϳ� �÷���.. button ������.. Network_HM.cpp�� ��������!
	memset(temp_pos_ori, 0.0, 7*sizeof(double));// ���� ������� �ϳ� �÷���.. button ������.. Network_HM.cpp�� ��������!
	button = -1; // SWITCH_L_X ���� 0��

	bQuit = false;

	v.resize(31);
	buttonclk = 0;
	currclk = 0;    

	scale = 0;

	exoHT_ee_R.setIdentity();
	exoHT_ee_L.setIdentity();

	//���뺯��
	shared_v;
	memset(mcu_data, 0, MCUDATALEN*sizeof(BYTE));
	data_analyzed = false;
}
T_Exo::~T_Exo()
{
}

bool T_Exo::Init_Exo(wchar_t* portname)
{
	if(_serial.OpenPort(portname, CBR_115200, 8, ONESTOPBIT , NOPARITY)){   // ���� ���� COM Port �� �־���մϴ�.  
		cout<<"Exo was connected!"<<endl;	
		connect_stats = true;
		return true;
	}
	else
		return false;
}

bool T_Exo::Discnt_Exo()
{
	if(_serial.m_bConnected){   // ���� ���� COM Port �� �־���մϴ�.  
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
	for(int i  = 0 ; i < iSize; i++)//���� ���� ��ŭ �����͸� �о� �� ȭ�鿡 ������
	{
		(_serial.m_QueueRead).GetByte(&aByte);//ť���� ������ �Ѱ��� �о��

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
				//puts(ptr); // �� ������ �� ����.
				
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
				//// Ŭ��ġ�� ���� �б⹮�� ������ �ش�. 
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

// Exo�� Robot ��ǥ�� calibration.. orientation�� �׳� �������ٰ� ���� ��..
// �׷��� �������� ���� ���� �� �� �� �� ���� �븩�̴�.. �׸��� ori�� ��������� ����ؼ� 
// �ϴ��� matrix�� ������ְ�, ���ȱ����� ����ؼ� 4*4 ��Ʈ���� ������ 2 �� ���������.
// �׸��� �̷��� ��쿡 ���� �޶����� �͵��� �Լ� �����ε��� ����.
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

// Exo�� Robot ��ǥ�� mapping.. �ٵ� Transform�� �κ����� �ٸ� �� ��� �� �ٱ�. => �Ķ���ͷ� ����.(rotation_mapper)
//����ϴ� ������ ���� ���� �Ķ���͵� �������
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

	///*����ϴ� ������ ���� ����*/
	//double unit_compensate = 1.0/1000.0;

	//rHTranform_target.col(3).segment(0, 3) = (exoPos - calib_exoHome)*units_compensate + calib_robotHome;
	//Exo�� Amiro���� ��쿡�� ��� ���� ���߾��� ������ �ƹ��͵� ������ �ʿ䰡 ������.
	rHTranform_target.col(3).segment(0, 3) = scale*rotation_mapper*(exoPos - exoHome) + robotHome;
	rHTranform_target.col(3).segment(0, 3) = scale*rotation_mapper*exoPos;
	rHTranform_target.topLeftCorner(3, 3) = rotation_mapper*exoOri; 
}

// Exo�� Staubli ��ǥ�� mapping
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
		temp_point_vector[i] = (double)exoData[((i+3)*4+3) + 3];; // �� �� position
	
	for(int i=0; i<3; i++){
		temp_ori_vector[i] = (double)exoData[15 + i];; // �� �� orientation
		temp_ori_vector[3+i] = (double)exoData[19 + i];; // �� �� orientation
		temp_ori_vector[6+i] = (double)exoData[23 + i];; // �� �� orientation
	}
	
	/*����ϴ� ������ ���� ����*/
	double unit_compensate = 1.0;

	rPos_target.resize(3);
	for(int i=0; i<3; i++)
		rPos_target[i] = (temp_point_vector[i] - exoHome[i])*unit_compensate + robotHome[i];

	rOri_target.resize(9);
	rOri_target = temp_ori_vector;
}

void T_Exo::MakestaubliOri(vector <double> rExoOri, vector <double> rExoOriHome, vector <double> & rStaubliOri)
{
	//��ǥ�� �������� �ٽ�. z

	//��.�� X-Y-Z Euler angle (refernce manual ���� staubli �� �̰� ��� �ϴ� �� ó�� ����)
	double alpha = 0.0;
	double beta = 0.0;
	double gamma = 0.0;

	beta = atan2(rExoOri[2], sqrt(rExoOri[5]*rExoOri[5]+rExoOri[8]*rExoOri[8]));
    gamma = atan2(rExoOri[1]/cos(beta), -rExoOri[0]/cos(beta));
	alpha = atan2(-rExoOri[5]/cos(beta), rExoOri[8]/cos(beta));

	double pi = 3.141592;

	//������ �ϳ� �ִµ�, Exo ���� ��쿡�� �ʱ� ��ġ�� ������ ���� ����ġ ������..
	//�켱�� �� ���� ���ϰ� ���� ���� �������� �ξ��� ���� ������ �ʱ� ��ġ��� �� ����.
	// �ӽ÷� �׳� �Ȱ��̿� �ɾ��� �� ������ Ķ���극�̼� �غ���.
	// �ٵ� �� �����غ���... �׷��� ���������� ���� �� �ִ�. �������� ���� �� ���������̼ǿ� ���߷���...
	// ���� ���� �����ִ� ���� ���� ���� �ִ�.
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
		std::cout<<"4."<<alpha * 180/pi<<" "<<beta * 180/pi<<" "<<gamma * 180/pi<<std::endl; // �̰Ŵ� exo�� ���� staubli ������� �ؼ��� ��.
		std::cout<<rStaubliOri[0]<<" "<<rStaubliOri[1]<<" "<<rStaubliOri[2]<<std::endl;
		printer_count = 0;
	}
	printer_count++;

	//// �Ʒ��� Staubli LLI �ڵ忡�� ������ �����ε�, ������ �ʿ䰡 ����.
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
//	//��.�� X-Y-Z Euler angle (refernce manual ���� staubli �� �̰� ��� �ϴ� �� ó�� ����)
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
