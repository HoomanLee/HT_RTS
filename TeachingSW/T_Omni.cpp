#include "Precompiled.h"
#include "T_Omni.h"

T_Omni::T_Omni()
{
	button = 0;
	hGravityWell = HD_INVALID_HANDLE;

	btn_preclk = 0;
	btn_curclk = 0;
	btn_prsdcnt = 0;

	memset(&BtnCmd, 0, sizeof(ButtonStruct));
	//cout<<BtnCmd.button1_prsd_time<<endl;
	//clock_t prsdclk = 0;
	//clock_t endclk = 0;
	//int button1_cnt = 0;
	//int button2_cnt = 0;
	//int button1_prsd_time = 0;
	//int button2_prsd_time = 0;

	connect_stats = false;

	bQuit = false;
	_haptics = false;
	_FT_filter = false;
	_haptics_getpos = false;
	_haptics_getforce = false;
	_haptics_getenc = false;

	HT_target.linear().setIdentity();
	HT_target.translation().setZero();

	hMutex_pos = CreateMutex( NULL, FALSE, NULL );
	hMutex_force = CreateMutex( NULL, FALSE, NULL );
	hMutex_enc = CreateMutex( NULL, FALSE, NULL );
	
	////// 이벤트 (쓰레드 순서 조정 위해) //////
	event_sequence =  CreateEvent(NULL, TRUE, FALSE, NULL);

	////네트워크로 주고받을 데이터 구조체 초기화.
	//memset(&T_ND, 0, sizeof(T_NetData));

	//사용할 변수 메모리 초기화
	memset(&_force_haptics, 0, sizeof(Vector3d));

	//memset(&Staubli, 0, sizeof(Staubli));
}

T_Omni::~T_Omni()
{
	CloseHandle( hMutex_pos );
	CloseHandle( hMutex_force );
	CloseHandle( hMutex_enc );
}

int T_Omni::Haptic_initialize()
{
	HDErrorInfo error;
	/* Initialize the device, must be done before attempting to call any hd 
		functions. Passing in HD_DEFAULT_DEVICE causes the default device to be 
		initialized. */
	_hHD = hdInitDevice(HD_DEFAULT_DEVICE);

	if (HD_DEVICE_ERROR(error = hdGetError())) 
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		fprintf(stderr, "\nPress any key to quit.\n");
		_haptics = false;
		getch();
		//return -1;
		return -1;
	}
	
	if (!initDemo())
	{
		printf("Demo Initialization failed\n");
		printf("Press any key to exit\n");
		_haptics = false;
		
		/* Disable the device. */
		hdDisableDevice(_hHD);
		getch();

		return -1;
	}

	hGravityWell = hdScheduleAsynchronous(
		jointTorqueCallback, this, 
		HD_MAX_SCHEDULER_PRIORITY);

	hdScheduleAsynchronous(
        hm_buttonCallback, &BtnCmd/*&button*/, 
        HD_MAX_SCHEDULER_PRIORITY);
	

	printf("Command Joint Torque Demo!\n");
	printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));


	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();

	connect_stats = true; // 오류가 나서 return -1 하는 거 아니면 성공이라고 보고..
	
	Sleep(100); //햅틱 이니셜라이즈 시간 기다려줌.. 안하면 쓰레기값 나올 때 있는 듯..

	return connect_stats;
}
void T_Omni::Haptic_finish()
{
	hdStopScheduler();
	hdUnschedule(hGravityWell);

	/* Disable the device. */
	hdDisableDevice(_hHD);

	connect_stats = false;
}

void T_Omni::setHapticPos(hduVector3Dd rPos_haptics, HDdouble rOri_haptics[], hduVector3Dd rVel_haptics)
{
	WaitForSingleObject(hMutex_pos,  INFINITE);
	for( int i = 0 ; i < 3 ; i++)
	{
		_pos_haptics[i] = rPos_haptics[i];
		_vel_haptics[i] = rVel_haptics[i];
	}
	for( int i = 0 ; i < 4 ; i++)
		for( int j = 0 ; j < 4 ; j++)
			_ori_haptics(j, i) = rOri_haptics[i*4 + j]; //column-major로 들어왓기 때문에 변환이 필요함
	//_pos_haptics.print(_T("setHapticsPos"));
	//_ori_haptics.print(_T("_ori_haptics"));
	_haptics_getpos = true;
	ReleaseMutex( hMutex_pos );
}

void T_Omni::getHapticPos(Vector3d& rPos, Matrix3d& rOri, Vector3d& rVel)
{
	WaitForSingleObject(hMutex_pos,  INFINITE);
	for( int i = 0 ; i < 3 ; i++)
    {
		rPos[i] = _pos_haptics[i];
		rVel[i] = _vel_haptics[i];
	}
	for( int i = 0 ; i < 3 ; i++)
		for( int j = 0 ; j < 3 ; j++)
			rOri(i, j) = _ori_haptics(i, j); //column-major로 들어왓기 때문에 변환이 필요함
	//rPos.print(_T("getHapticPos"));
	//cout<<rOri.row(0).norm()<<" "<<rOri.row(1).norm()<<" "<<rOri.row(2).norm()<<" "<<rOri.col(0).norm()<<" "<<rOri.col(1).norm()<<" "<<rOri.col(2).norm()<<" "<<endl;
	//static int pp_counter = 0;
	//if (pp_counter == 50){
	//	cout<<rOri(0, 0)<<rOri(0, 1)<<rOri(0, 2)<<endl;
	//	cout<<rOri(1, 0)<<rOri(1, 1)<<rOri(1, 2)<<endl;
	//	cout<<rOri(2, 0)<<rOri(2, 1)<<rOri(2, 2)<<endl;
	//	pp_counter = 0;
	//}
	//pp_counter++;
	_haptics_getpos = false;
	ReleaseMutex( hMutex_pos );
}

void T_Omni::setForce_Haptic(hduVector3Dd& rForce_haptics)
{
	WaitForSingleObject(hMutex_force,  INFINITE);
	for( int i = 0 ; i < 3 ; i++)
	{
		//if(_force_haptics[i] >= 5.0)
		//	_force_haptics[i] = 0.0;
		rForce_haptics[i] = _force_haptics[i];
	}
	_haptics_getforce = false;
	ReleaseMutex( hMutex_force );
}
void T_Omni::getForce_Haptic(Vector3d rForce)
{
	WaitForSingleObject(hMutex_force,  INFINITE);
	for( int i = 0 ; i < 3 ; i++)
		_force_haptics[i] = rForce[i];
	_haptics_getforce = true;
	ReleaseMutex( hMutex_force );
}

void T_Omni::setEnc_Haptic(HDlong rEnc_haptics[]) // gimbal 쪽 encoder 값만 가져온다..
{
	WaitForSingleObject(hMutex_enc,  INFINITE);
	//memcpy(&_enc_haptics, rEnc_haptics, sizeof(long) * 3);
	for( int i = 0 ; i < 3 ; i++)
		_enc_haptics[i] = rEnc_haptics[i];
	_haptics_getenc = true;
	ReleaseMutex( hMutex_enc );
}

void T_Omni::getEnc_Haptic(Vector3d& rEnc)
{
	WaitForSingleObject(hMutex_enc,  INFINITE);
	for( int i = 0 ; i < 3 ; i++)
	{
		if(_force_haptics[i] >= 5.0)
			_force_haptics[i] = 0.0;
		rEnc[i] = _enc_haptics[i];
	}
	_haptics_getenc = false;
	ReleaseMutex( hMutex_enc );
}
void T_Omni::RHmapper(Transform<double, 3, 2>& rHTranform_target, Vector3d& rVel_target, Vector3d rHome, 
		Vector3d rPos, Matrix3d rOri, Vector3d rVel)
{
	//VectorXd robot_home(3);
	Vector3d robot_home;
	///// 모든 조인트 30 degree 일 때 
	//robot_home(0) = (0.4465);
	//robot_home(1) = (0.379);
	//robot_home(2) = (0.6706);

	robot_home = rHome;

	//inkwell에 있을 때의 omni position
	//VectorXd haptics_inkwell(3);
	Vector3d haptics_inkwell;
	haptics_inkwell[0] = (0.0);
	haptics_inkwell[1] = (-65.51);
	haptics_inkwell[2] = (-88.11);

	//MatrixXd frame_mapping1(3, 3);
	Matrix3d frame_mapping1;
	frame_mapping1 = Rotate_with_Y(M_PI/2);

	//MatrixXd frame_mapping2(3, 3);
	Matrix3d frame_mapping2;
	frame_mapping2 = Rotate_with_Z(M_PI/2);

	Matrix3d frame_mapping3;
	frame_mapping3 = Rotate_with_X(M_PI);

	Matrix3d frame_mapping4;
	frame_mapping4 = Rotate_with_Z(-M_PI/2);

	//MatrixXd frame_mapping1_trans(3, 3);
	Matrix3d frame_mapping1_trans;
	frame_mapping1_trans = frame_mapping1.transpose();
	//MatrixXd frame_mapping2_trans(3, 3);
	Matrix3d frame_mapping2_trans;
	frame_mapping2_trans = frame_mapping2.transpose();


	/*사용하는 단위계 차이 보정*/
	double unit_compensate = 1.0;

	rHTranform_target.translation() = frame_mapping1*frame_mapping2*(rPos - haptics_inkwell)*unit_compensate + robot_home;


	// Orientation 좌표계 매칭.. {s^}{_omniE}R = {s^}{_omni}R * {omni^}{_omniE}R * {omniE^}{_staubliiE}R
	rHTranform_target.linear() = frame_mapping1*frame_mapping2 * rOri * frame_mapping3 * frame_mapping4;

	rVel_target = frame_mapping2_trans*frame_mapping1_trans*rVel/1000;
}

// 아래는 애초에 잘못된 방식인 것 같다. orientation은 절대값 mapping하면 되는 듯!
//void T_Omni::MakestaubliOri(Matrix3d& rHapticOri, Vector3d& rStaubliOri)
//{
//	//좌표계 보정부터 다시. z
//
//	//방법 2. Handbook of Robotics에서 찾은 것.. psi = alpha, theta = beta, phi = gamma
//	double psi = 0.0;
//	double theta = 0.0;
//	double phi = 0.0;
//	theta = atan2(-rHapticOri.coeff(2, 0), sqrt(rHapticOri.coeff(0, 0)*rHapticOri.coeff(0, 0) + 
//	rHapticOri.coeff(1, 0)*rHapticOri.coeff(1, 0)));
//	psi = atan2(rHapticOri.coeff(1, 0)/cos(theta), rHapticOri.coeff(0, 0)/cos(theta));
//	phi = atan2(rHapticOri.coeff(2, 1)/cos(theta), rHapticOri.coeff(2, 2)/cos(theta));
//
//	//방.법 X-Y-Z Euler angle (refernce manual 보면 staubli 는 이거 사용 하는 것 처럼 보임)
//	double alpha = 0.0;
//	double beta = 0.0;
//	double gamma = 0.0;
//
//	int sign = 1;
//	if( rHapticOri(0,2) < (-1.0 + 1e-5*1e-5/2.0) ){
//		alpha = 0;
//		beta = -M_PI/2;
//		gamma = atan2(rHapticOri(1,0), rHapticOri(3,1));
//	}else if( rHapticOri(0,2) > (1.0 + 1e-5*1e-5/2.0) ) {
//		alpha = 0;
//		beta = M_PI/2;
//		gamma = atan2(rHapticOri(1,0), rHapticOri(3,1));
//	}else {
//		beta = asin(rHapticOri(0,2));
//		if( rHapticOri(2,2)<0.0 && rHapticOri(0,0)<0.0 && ( abs(rHapticOri(0,2))>1e-10 || abs(rHapticOri(1,2))>1e-10  ) )
//		{
//			if(beta >= 0.0){
//				beta = M_PI-beta;
//			}else {
//				beta = -M_PI-beta;
//			}
//			sign = -1;
//		}
//		alpha = atan2(-sign*rHapticOri(1,2), sign*rHapticOri(2,2));
//		gamma = atan2(-sign*rHapticOri(0,1), sign*rHapticOri(0,0));
//	}
//
//	double pi = 3.141592;
//
//	Vector3d omni_orihome; //이건 뭘 기준으로 가져왔더라? omni stylus를 직각으로 세워두었을 때의 값인 듯.
//	omni_orihome[0] = 0 * M_PI/180;
//	omni_orihome[1] = 0 * M_PI/180;
//	omni_orihome[2] = -90 * M_PI/180;
//
//
//	//VectorXd staubli_orihome(3);
//	Vector3d staubli_orihome;
//	staubli_orihome[0] = 0.0 * M_PI/180;
//	staubli_orihome[1] = -180.0 * M_PI/180;
//	staubli_orihome[2] = 0.0 * M_PI/180;
//
//	Vector3d ori_omni;
//	ori_omni[0] = alpha;
//	ori_omni[1] = beta;
//	ori_omni[2] = gamma;
//
//	rStaubliOri[0] = (ori_omni[0] - omni_orihome[0]) + staubli_orihome[0];
//	rStaubliOri[1] = (ori_omni[1] - omni_orihome[1]) + staubli_orihome[1];
//	rStaubliOri[2] = (ori_omni[2] - omni_orihome[2]) + staubli_orihome[2]; // z축 방향만 반대..
//
//	/////////// 아니다~!~!~! 잘 생각해보면, Euler angle에서 +-로 초기위치 잡고 이러는 것은 아무 의미 없다!!
//	/////////// 그냥 E-E의 local frame만 맞춰주면 된다!!!
//	//rStaubliOri = ori_omni;
//
//	//for(int i=0; i<3; i++){
//	//	if(rStaubliOri[i] < -180)
//	//		rStaubliOri[i] = rStaubliOri[i]+360;
//	//	else if(rStaubliOri[i]>180)
//	//		rStaubliOri[i] = rStaubliOri[i]-360;
//	//}
//
//	//static int printer_count = 0;
//	//if (printer_count == 10){
//	//	std::cout<<"2."<<psi * 180/pi<<" "<<theta * 180/pi<<" "<<phi * 180/pi<<std::endl; //1번 방법과 같은 지 확인 OK, 다만 tan가 각도가 다름
//	//	std::cout<<"4."<<alpha * 180/pi<<" "<<beta * 180/pi<<" "<<gamma * 180/pi<<std::endl; // 이거는 omni의 값을 staubli 방식으로 해석한 것.
//	//	std::cout<<rStaubliOri[0]<<" "<<rStaubliOri[1]<<" "<<rStaubliOri[2]<<std::endl;
//	//	printer_count = 0;
//	//}
//	//printer_count++;
//	
//	//std::cout<<rHapticOri.coeff(0, 0)<<" h"<<rHapticOri(1, 1)<<" "<<rHapticOri(2, 2)<<std::endl;
//}

void T_Omni::MakestaubliTransform(Vector3d rStaubliPos, Vector3d rStaubliOri, Matrix4d &rStaubliTransform)
{
	rStaubliTransform.setIdentity();
	
	double alpha, beta, gamma;
	alpha = rStaubliOri(0), beta = rStaubliOri(1), gamma = rStaubliOri(2);

	rStaubliTransform(0, 3) = rStaubliPos(0);	// x
	rStaubliTransform(1, 3) = rStaubliPos(1);	// y
	rStaubliTransform(2, 3) = rStaubliPos(2);	// z

	// direction cos
	rStaubliTransform(0, 0) = cos(beta) * cos(gamma);
	rStaubliTransform(0, 1) = -cos(beta) * sin(gamma);
	rStaubliTransform(0, 2) = sin(beta);
	rStaubliTransform(1, 0) = sin(alpha) * sin(beta) * cos(gamma) + cos(alpha) * sin(gamma);
	rStaubliTransform(1, 1) = -sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
	rStaubliTransform(1, 2) = -sin(alpha) * cos(beta);
	rStaubliTransform(2, 0) = -cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
	rStaubliTransform(2, 1) = cos(alpha) * sin(beta) * sin(gamma) + sin(alpha) * cos(gamma);
	rStaubliTransform(2, 2) = cos(alpha) * cos(beta);
}

void T_Omni::RHForcemapper(Vector3d& rForce_target, Vector3d rForce)
{
	Matrix3d frame_mapping1;
	frame_mapping1 = Rotate_with_Z(-M_PI/2);

	//MatrixXd frame_mapping2(3, 3);
	Matrix3d frame_mapping2;
	frame_mapping2 = Rotate_with_Y(M_PI/2);

	/*사용하는 단위계 차이 보정*/
	double unit_compensate = 1;
	// FT 센서 값 좌표계 매칭
	rForce_target = unit_compensate*frame_mapping1*frame_mapping2*rForce;
}

void T_Omni::TeachingForcemapper(Vector3d& rForce_target, Vector3d rForce)
{
	Matrix3d frame_mapping1;
	frame_mapping1 = Rotate_with_Y(M_PI/2);

	//MatrixXd frame_mapping2(3, 3);
	Matrix3d frame_mapping2;
	frame_mapping2 = Rotate_with_Z(M_PI/2);

	/*사용하는 단위계 차이 보정*/
	double unit_compensate = 1;
	// FT 센서 값 좌표계 매칭
	rForce_target = unit_compensate*frame_mapping1*frame_mapping2*rForce;
}

void T_Omni::Low_Pass_Filter(MatrixXd& rDataset, int rDatacount, int rDatasize, VectorXd rData_pre, 
		VectorXd& rData)
{
	static int filter_counter = 0;
	//rDataset.resize(rDatacount, rDatasize);
	_FT_filter = false;
	if (filter_counter == 0)
	{
		rDataset.resize(rDatacount, rDatasize);
		rDataset.Zero(rDatacount, rDatasize);
	}
	else if(filter_counter == rDatacount)
	{
		VectorXd rDataSum(rDatasize);
		for (int i = 0; i < rDatasize; i++)
			for (int j = 0; j < rDatacount; j++)
				rDataSum[i] += rDataset(j, i);
		
		rData = rDataSum/rDatacount;

		_FT_filter = true;
		filter_counter = 0;
	}
	//rDataset.setrow(filter_counter, rData_pre); 
	for(int i = 0; i < rDatasize; i++)
		rDataset(filter_counter, i) = rData_pre(i);
	filter_counter++;
	//rDataset.print(_T("dataset"));
}

Matrix3d	T_Omni::Rotate_with_X(double rAngle)
{
	//MatrixXd	_Rotate_wth_X(3, 3);
	Matrix3d	_Rotate_wth_X;
	
	_Rotate_wth_X(0, 0) = 1.0;
	_Rotate_wth_X(1, 0) = 0.0;
	_Rotate_wth_X(2, 0) = 0.0;

	_Rotate_wth_X(0, 1) = 0.0;
	_Rotate_wth_X(1, 1) = cos(rAngle);
	_Rotate_wth_X(2, 1) = sin(rAngle);

	_Rotate_wth_X(0, 2) = 0.0;
	_Rotate_wth_X(1, 2) = -sin(rAngle);
	_Rotate_wth_X(2, 2) = cos(rAngle);

	return(_Rotate_wth_X);
}

Matrix3d	T_Omni::Rotate_with_Y(double rAngle)
{
	//MatrixXd	_Rotate_wth_Y(3, 3);
	Matrix3d	_Rotate_wth_Y;
	
	_Rotate_wth_Y(0, 0) = cos(rAngle);
	_Rotate_wth_Y(1, 0) = 0.0;
	_Rotate_wth_Y(2, 0) = -sin(rAngle);

	_Rotate_wth_Y(0, 1) = 0.0;
	_Rotate_wth_Y(1, 1) = 1.0;
	_Rotate_wth_Y(2, 1) = 0.0;

	_Rotate_wth_Y(0, 2) = sin(rAngle);
	_Rotate_wth_Y(1, 2) = 0.0;
	_Rotate_wth_Y(2, 2) = cos(rAngle);

	return(_Rotate_wth_Y);
}

Matrix3d	T_Omni::Rotate_with_Z(double rAngle)
{
	//MatrixXd	_Rotate_wth_Z(3, 3);
	Matrix3d	_Rotate_wth_Z;
	
	_Rotate_wth_Z(0, 0) = cos(rAngle);
	_Rotate_wth_Z(1, 0) = sin(rAngle);
	_Rotate_wth_Z(2, 0) = 0.0;

	_Rotate_wth_Z(0, 1) = -sin(rAngle);
	_Rotate_wth_Z(1, 1) = cos(rAngle);
	_Rotate_wth_Z(2, 1) = 0.0;

	_Rotate_wth_Z(0, 2) = 0.0;
	_Rotate_wth_Z(1, 2) = 0.0;
	_Rotate_wth_Z(2, 2) = 1.0;

	return(_Rotate_wth_Z);
}

//void Ori_DegreetoTransform(char mode, VectorXd rDegree, MatrixXd& rHapticOri)
//{
//	if (mode == 1)
//	{
//		rHapticOri.resize(3, 3);
//		rHapticOri.coeff(0, 0) = ;
//
//}

void T_Omni::PrintHelp()
{
    static const char help[] = {\
	"CommandJointTorque Help\n\
	---\n\
	T: Command Base & Gimbal Torques \n\
	   Torsional Springs at each Joint\n\
	   Torsion Spring Constant at Joints = 1000 mN.m/radian\n\
	   Torsion Spring Constant at Gimbals = 500 mN.m/radian\n\
	   Max Base Torque Commanded by this Demo = {200.0,350.0,200.0}mNm\n\
	   Max Gimbal Torque Commanded by this Demo = {188.0,188.0,48.0}mNm\n\
	F: Command Base force & Gimbal Torque\n\
	   Torsional Springs at Gimbals & a Extension Spring at Position(0,0,0)\n\
	   Extension Spring Constant at (0,0,0) = 0.075 N/mm\n\
	   Torsion Spring Constant at Gimbals = 500 mN.m/radian\n\
	L: Prints device state\n\
	C: Continuously prints device state\n\
	H: Prints help menu\n\
	Q: Quits the program\n\
	---"};
    
    printf("\n%s\n", help);
}

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
HDCallbackCode T_Omni::GetDeviceStateCallback(void *pUserData)
{
    DeviceStateStruct *pState = (DeviceStateStruct *) pUserData;

    hdGetIntegerv(HD_CURRENT_BUTTONS, pState->buttonValues);
	hdGetDoublev(HD_CURRENT_FORCE, pState->forceValues);
    hdGetDoublev(HD_CURRENT_JOINT_TORQUE, pState->jointTorqueValues);
    hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, pState->gimbalTorqueValues);

    return HD_CALLBACK_DONE;
}

///*****************************************************************************
// Callback that retrieves state.
//*****************************************************************************/
void T_Omni::PrintDeviceState(HDboolean bContinuous)
{
    int i;
    DeviceStateStruct state;

    memset(&state, 0, sizeof(DeviceStateStruct));

    do
    {
        hdScheduleSynchronous(GetDeviceStateCallback, &state,
            HD_DEFAULT_SCHEDULER_PRIORITY);

        printf("\n");

        if (TorqueMode)
        {
        printf("Current Base Torque Values (mNm):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.jointTorqueValues[i]);
        }
        printf("\n");

        printf("Current Gimbal Torque Values (mNm):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.gimbalTorqueValues[i]);
        }
        printf("\n");
        }
        else
        {
        printf("Current Base Force Values (N):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.forceValues[i]);
        }
        printf("\n");

        printf("Current Base Torque Values (mNm):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.jointTorqueValues[i]);
        }
        printf("\n");

        printf("Current Gimbal Torque Values (mNm):");
        for (i = 0; i < 3; i++)
        {
            printf(" %f", state.gimbalTorqueValues[i]);
        }
        printf("\n");
        }

        if (bContinuous)
        {
            Sleep(500);
        }

    } while (!_kbhit() && bContinuous);
}
///////////////////////////////////////////////////////////////////////


///******************************************************************************
// The main loop of execution.  Detects and interprets keypresses.  Monitors and 
// initiates error recovery if necessary.
//******************************************************************************/
void T_Omni::mainLoop()
{
	int keypress;
    int nMotorIndex = 0;

    while (TRUE)
    {
        if (_kbhit())
        {
            keypress = getch();
            keypress = toupper(keypress);
            
            switch (keypress)
            {
                case 'F': TorqueMode = false; break;
                case 'T': TorqueMode = true; break;
                case 'P': PrintDeviceState(FALSE); break;
                case 'C': PrintDeviceState(TRUE); break;
                case 'L': PrintHelp(); break;
                case 'Q': return;
                default: PrintHelp(); break;
            }
        }

        /* Check if the scheduled callback has stopped running */
        if (!hdWaitForCompletion(hGravityWell, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            return;
        }
    }
}
//
///*******************************************************************************
// Servo callback.  
// Called every servo loop tick.  Simulates a gravity well, which sucks the device 
// towards its center whenever the device is within a certain range.
//*******************************************************************************/
HDCallbackCode T_Omni::jointTorqueCallback(void *data)
{
    T_Omni* OMST = reinterpret_cast<T_Omni*>(data);
	
	const HDdouble kStiffness = 0.075; /* N/mm */
    const HDdouble kStylusTorqueConstant = 500; /* torque spring constant (mN.m/radian)*/
    const HDdouble kJointTorqueConstant = 1000; /* torque spring constant (mN.m/radian)*/
 
    const HDdouble kForceInfluence = 50; /* mm */
    const HDdouble kTorqueInfluence = 3.14; /* radians */

    /* This is the position of the gravity well in cartesian
       (i.e. x,y,z) space. */
    static hduVector3Dd wellPos(0,0,0);
    static const hduVector3Dd stylusVirtualFulcrum(0.0, 0.0, 0.0); // In radians
    static const hduVector3Dd jointVirtualFulcrum(0.0, 0.0, 0.0); // In radians
    
    /*자 well pose를 조절해서 한 번 만들어보자~~*/
	//double d_x = 0.0, d_y = 0.0, d_z = 0.0;
	//static int counter_t = 0;
	//if(counter_t<2*3.14*100000)
	//{
	//	d_x = 50.0*sin(counter_t/(2*3.14*100));
	//	d_y = 50.0*sin(counter_t/(2*3.14*100));
	//	wellPos[0] =d_x;
	//	wellPos[1] =d_y;
	//	wellPos[2] =d_z;
	//	//printf("%f\n", wellPos[0]);
	//	counter_t++;
	//}
	//else
	//{
	//	counter_t = 0;
	//	wellPos[0] =d_x;
	//	wellPos[1] =d_y;
	//	wellPos[2] =d_z;
	//}

	
	HDErrorInfo error;
    hduVector3Dd position;

	hduVector3Dd force;
	hduVector3Dd velocity;
    hduVector3Dd positionTwell;
    hduVector3Dd gimbalAngles;
    hduVector3Dd gimbalTorque;
    hduVector3Dd gimbalAngleOfTwist;
    hduVector3Dd jointAngles;
    hduVector3Dd jointTorque;
    hduVector3Dd jointAngleOfTwist;
	HDlong	encoders[6];

    HHD hHD = hdGetCurrentDevice();

    /* Begin haptics frame.  ( In general, all state-related haptics calls
       should be made within a frame. ) */
    hdBeginFrame(hHD);

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
	hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES,gimbalAngles );
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,jointAngles );   // omni라서 gimbal은 의미가 없음
	 

	HDdouble Haptics_transform[16];
	hdGetDoublev(HD_CURRENT_TRANSFORM, Haptics_transform); // 이 떄, column-major로 저장하기 때문에 변환이 필요함!!
	//printf("Haptics_transform : %f	%f	%f	%f\n", Haptics_transform[0], Haptics_transform[1], Haptics_transform[2], Haptics_transform[3]);
	//printf("Haptics_transform : %f	%f	%f	%f\n", Haptics_transform[4], Haptics_transform[5], Haptics_transform[6], Haptics_transform[7]);
	//printf("Haptics_transform : %f	%f	%f	%f\n", Haptics_transform[8], Haptics_transform[9], Haptics_transform[10], Haptics_transform[11]);
	//printf("Haptics_transform : %f	%f	%f	%f\n", Haptics_transform[12], Haptics_transform[13], Haptics_transform[14], Haptics_transform[15]);
	

    memset(force, 0, sizeof(hduVector3Dd));

	/* >  positionTwell = wellPos-position  < 
       Create a vector from the device position towards the gravity 
       well's center. F= kx 에서 x 가 될 녀석 */
    hduVecSubtract(positionTwell, wellPos, position);
	
	OMST->setHapticPos(position, Haptics_transform, velocity);
		
	//if(OMST->_haptics_getforce)
		OMST->setForce_Haptic(force);

		
	//if(_haptics_getforce)
	//	setForce_Haptic(force);
	////else
	////	force.set(rhctrl->_Cdetector(0), rhctrl->_Cdetector(1), rhctrl->_Cdetector(2));

	//printf("position : %f   %f   %f\n", position[0], position[1], position[2]);
	//printf("positionTwell : %f   %f   %f\n", positionTwell[0], positionTwell[1], positionTwell[2]);
    
	hdGetLongv(HD_CURRENT_ENCODER_VALUES, encoders ); 
	HDlong enc_gymbol[3];
	enc_gymbol[0] = encoders[3];
	enc_gymbol[1] = encoders[4];
	enc_gymbol[2] = encoders[5];
	OMST->setEnc_Haptic(enc_gymbol);
	//printf("encoders : %d	%d	%d	%d	%d	%d\n", encoders[0], encoders[1], encoders[2], encoders[3], encoders[4], encoders[5]);

    hduVecSubtract(gimbalAngleOfTwist, stylusVirtualFulcrum, gimbalAngles);

    hduVecSubtract(jointAngleOfTwist, jointVirtualFulcrum, jointAngles);

    /* If the device position is within some distance of the gravity well's 
       center, apply a spring force towards gravity well's center.  The force
       calculation differs from a traditional gravitational body in that the
       closer the device is to the center, the less force the well exerts;
       the device behaves as if a spring were connected between itself and
       the well's center. */
    if (hduVecMagnitude(positionTwell) < kForceInfluence)
    {
        /* >  F = k * x  < 
           F: Force in Newtons (N)
           k: Stiffness of the well (N/mm)
           x: Vector from the device endpoint position to the center 
           of the well. */
        //hduVecScale(force, positionTwell, kStiffness);
    }

    if (hduVecMagnitude(gimbalAngleOfTwist) < kTorqueInfluence)
    {
        /* >  T = k * r  < We calculate torque by assuming a torsional spring at each joint. 
           T: Torque in Milli Newton . meter (mN.m)
           k: Torque Spring Constant (mN.m / radian)
           r: Angle of twist from fulcrum point (radians)*/
        //hduVecScale(gimbalTorque, gimbalAngleOfTwist, kStylusTorqueConstant);
    }
    
    if (hduVecMagnitude(jointAngleOfTwist) < kTorqueInfluence)
    {
        /* >  T = k * r  < We calculate torque by assuming a torsional spring at each joint. 
           T: Torque in Milli Newton . meter (mN.m)
           k: Torque Spring Constant (mN.m / radian)
           r: Angle of twist from fulcrum point (radians)*/

        //hduVecScale(jointTorque, jointAngleOfTwist, kJointTorqueConstant);
    }


// Clamp the base torques to the nominal values. 
    for (int i=0; i<3; i++)
    {
        if (jointTorque[i] > nominalBaseTorque[i])
            jointTorque[i] = nominalBaseTorque[i];
        else if (jointTorque[i] < -nominalBaseTorque[i])
            jointTorque[i] = -nominalBaseTorque[i];
    }

// Clamp the gimbal torques to the max continous values. 
    for (int i=0; i<3; i++)
    {
        if (gimbalTorque[i] > maxGimbalTorque[i])
            gimbalTorque[i] = maxGimbalTorque[i];
        else if (gimbalTorque[i] < -maxGimbalTorque[i])
            gimbalTorque[i] = -maxGimbalTorque[i];
    }

	//force[0] = 1.0;
	//force[1] = 0.0;
	//force[2] = 0.0;
    /* Send the forces & torques to the device. */
    /*Switch back between sending forces & torques 
    to the base motors */
    //if (!TorqueMode)
        hdSetDoublev(HD_CURRENT_FORCE, force);
    //else
    //    hdSetDoublev(HD_CURRENT_JOINT_TORQUE, jointTorque);

    //hdSetDoublev(HD_CURRENT_GIMBAL_TORQUE, gimbalTorque);

    //printf("%f\t%f\t%f\n\n", jointTorque[0], jointTorque[1], jointTorque[2]);
    /* End haptics frame. */
    hdEndFrame(hHD);

    /* Check for errors and abort the callback if a scheduler error
       is detected. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, 
                      "Error detected while rendering gravity well\n");
        
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}

int T_Omni::initDemo(void)
{
    HDErrorInfo error;
    int calibrationStyle;
    //printf("Calibration\n");

    hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);
    if (calibrationStyle & HD_CALIBRATION_AUTO || calibrationStyle & HD_CALIBRATION_INKWELL)
    {
        printf("Please prepare for starting the demo by \n");
        printf("placing the device at its reset position.\n\n");
        //printf("Press any key to continue...\n");
        //getch();
        return 1;
    }
    if (calibrationStyle & HD_CALIBRATION_ENCODER_RESET )
    {
        printf("Please prepare for starting the demo by \n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");

        getch();

        hdUpdateCalibration(calibrationStyle);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            printf("Calibration complete.\n\n");
            return 1;
        }
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            return 0;           
        }
    }
}
/*****************************************************************************/

HDCallbackCode T_Omni::hm_buttonCallback(void *userdata)
{	
	//HDint *ButtonState = (HDint *) userdata;
	//HDint nButtons, nLastButtons;
	//hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);



	//if (nButtons == 1) // 아래쪽 버튼(펜 쪽)
	//{
	//	//printf("button status 1 !!\n");
	//	*ButtonState = nButtons;
	//	A->ButtonState = nButtons;
	//}
	//else if (nButtons == 2) // 위 쪽 버튼. (꼬리 쪽)
	//{
	//	//printf("button status 2 !!\n");
	//	*ButtonState = nButtons;
	//	A->ButtonState = nButtons;
	//}
	//else if (nButtons == 3) //omni 버튼 두 개 다 눌렀을 때임.
	//{
	//	//printf("button status 3 !!\n");
	//	*ButtonState = nButtons;
	//	A->ButtonState = nButtons;
	//}
	//else if (nButtons == 4) // 4까지 value가 있다고는 하는데 아직 파악이 안됨.
	//{
	//	printf("button status 4 !!\n");
	//	*ButtonState = nButtons;
	//	A->ButtonState = nButtons;
	//}
	//else{
	//	*ButtonState = nButtons;
	//	A->ButtonState = nButtons;
	//}

	
	////////////////////////////////////////////////////////////
	///////// button command를 위한 시간 계산
	ButtonStruct *A;
	A = (ButtonStruct*) userdata;
	
	hdGetIntegerv(HD_CURRENT_BUTTONS, &A->ButtonState);

	switch(A->button1_cnt){
		case 1:					// Button1이 1로 바뀌는 순간 (1단계)
			A->prsdclk = clock();	// Button1이 입력되는 순간 시간 저장
			A->button1_cnt = 2;	// 다음 단계로 변경
			break;
		case 2:					// 2단계
			if (A->ButtonState == 0){	// Button1이 떼질 때
				A->button1_cnt++;
			}
			break;
		case 3:
			A->endclk = clock();	// Button1에서 손을 떼는 순간 시간 저장
			A->button1_prsd_time = A->endclk - A->prsdclk;		// 버튼 눌린 시간 저장

			A->button1_prsd_time = (float)(A->endclk - A->prsdclk)/CLOCKS_PER_SEC; // 버튼 눌린 시간 저장

			if(A->button1_prsd_time < 1.0)
				cout<<"button 1 ="<<A->button1_prsd_time<<"  < 1.0"<<endl;
			else if (A->button1_prsd_time > 1.0)
				cout<<"button 1 ="<<A->button1_prsd_time<<"  > 1.0"<<endl;
			
			A->button1_cnt = 0;	// 초기 Default 상태로 변경
			break;

		default:
			if (A->ButtonState==1){		// Button1이 1로 바뀌면,
				A->button1_cnt++;	// count가 증가하여 case 1:에 들어가게 됨
			}
			//A->button1_prsd_time = 0;

			break;
	}
	////////////////////////////////////////
	/////////버튼 2 시간 카운트 ////////
	switch(A->button2_cnt){

		case 1:					// Button2가 1로 바뀌는 순간 (1단계)
			A->prsdclk = clock();	// Button2가 입력되는 순간 시간 저장
			A->button2_cnt = 2;	// 다음 단계로 변경
			break;

		case 2:					// 2단계
			if (A->ButtonState == 0){	// Button1이 떼질 때
				A->button2_cnt++;
			}
			break;

		case 3:
			A->endclk = clock();	// Button1에서 손을 떼는 순간 시간 저장
			A->button2_prsd_time = A->endclk - A->prsdclk;

			A->button2_prsd_time = (float)(A->endclk - A->prsdclk)/CLOCKS_PER_SEC; // Button1에서 손을 떼는 순간 시간 저장

			if(A->button2_prsd_time < 1.0)
				cout<<"button 2 < 1.0"<<endl;
			else if (A->button2_prsd_time > 1.0)
				cout<<"button 2 > 1.0"<<endl;

			A->button2_cnt = 0;	// 초기 Default 상태로 변경
			break;

		default:
			if (A->ButtonState==2){		// Button2가 1로 바뀌면,
				A->button2_cnt++;	// count가 증가하여 case 1:에 들어가게 됨
			}
			//A->button2_prsd_time = 0;

			break;
	}

	////////////////////////////////////////
	/////////버튼 3 시간 카운트 ////////
	switch(A->button3_cnt){

		case 1:					// Button3이 되는 순간 (1단계)
			A->prsdclk3 = clock();	// Button3가 입력되는 순간 시간 저장
			A->button3_cnt = 2;	// 다음 단계로 변경
			break;

		case 2:					// 2단계
			if (A->ButtonState == 0){	// Button3이 떼질 때
				A->button3_cnt++;
			}
			break;

		case 3:
			A->endclk3 = clock();	// Button3에서 손을 떼는 순간 시간 저장
			A->button3_prsd_time = A->endclk3 - A->prsdclk3;

			A->button3_prsd_time = (float)(A->endclk3 - A->prsdclk3)/CLOCKS_PER_SEC; // Button1에서 손을 떼는 순간 시간 저장

			if(A->button3_prsd_time < 1.0)
				cout<<"button 3 < 1.0"<<endl;
			else if (A->button3_prsd_time > 1.0)
				cout<<"button 3 > 1.0"<<endl;

			A->button3_cnt = 0;	// 초기 Default 상태로 변경
			break;

		default:
			if (A->ButtonState==3){		// Button이 3이 되면,
				A->button3_cnt++;	// count가 증가하여 case 1:에 들어가게 됨
			}
			//A->button3_prsd_time = 0;

			break;
	}
	///////////////////////////////////////////////////////////



	return HD_CALLBACK_CONTINUE;
}
/*
void T_Omni::StaubliDH(Robot *m_Staubli)
{
	VectorXd _a(6), _alpha(6), _d(6), _theta(6);
	_a(0) = 0.0,	_alpha(0) = -M_PI/2,		_d(0) = 0.0,			_theta(0) = 0.0;
	_a(1) = 290.0,	_alpha(1) = 0.0,			_d(1) = 0.0,			_theta(1) = -M_PI/2;
	_a(2) = 0.0,	_alpha(2) = M_PI/2,			_d(2) = 20.0,			_theta(2) = M_PI/2;
	_a(3) = 0.0,	_alpha(3) = -M_PI/2,		_d(3) = 310.0,			_theta(3) = 0.0;
	_a(4) = 0.0,	_alpha(4) = M_PI/2,			_d(4) = 0.0,			_theta(4) = 0.0;
	_a(5) = 0.0,	_alpha(5) = 0.0,			_d(5) = 70.0+150.0,		_theta(5) = 0.0;

	SetRobotKin(m_Staubli, 6, _a, _alpha, _d, _theta);
}

void T_Omni::StaubliFK(Robot *m_Staubli, VectorXd _des_q)
{
	//Matrix4d _T01, _T12, _T23, _T34, _T45, _T56, _T6E;
	//this->GetHTransform(0, -M_PI/2, 0, 0+_des_q(0), _T01);
	//this->GetHTransform(290, 0, 0, -M_PI/2+_des_q(1), _T12);
	//this->GetHTransform(0, M_PI/2, 20, M_PI/2+_des_q(2), _T23);
	//this->GetHTransform(0, -M_PI/2, 310, 0+_des_q(3), _T34);
	//this->GetHTransform(0, M_PI/2, 0, 0 + _des_q(4), _T45);
	//this->GetHTransform(0, 0, 70+150, 0+_des_q(5), _T56); // Tool +150 붙은 것을 합해줬음.

	//m_Staubli->T01 = _T01;
	//m_Staubli->T02 = _T01 * _T12;
	//m_Staubli->T03 = _T01 * _T12 * _T23;
	//m_Staubli->T04 = _T01 * _T12 * _T23 * _T34;
	//m_Staubli->T05 = _T01 * _T12 * _T23 * _T34 * _T45;
	//m_Staubli->T06 = _T01 * _T12 * _T23 * _T34 * _T45 * _T56;
	
	StaubliDH(m_Staubli);
	GetForwardKin(m_Staubli, _des_q);

	//for(int i=0; i<4; i++){
	//	for(int j=0; j<4; j++)
	//		cout<<m_Staubli->T0E(i, j)<<"	";
	//	cout<<endl;
	//}
}

void T_Omni::StaubliJacobian(Robot *m_Staubli)
{
	//Matrix3d temp_ori;
	//Vector3d temp_euler;
	//for(int i=0; i<3; i++)
	//	for(int j=0; j<3; j++)
	//		temp_ori(i, j) = m_Staubli->T06(i, j);
	//this->MakestaubliOri(temp_ori, temp_euler);
	//this->GetJacobian(m_Staubli, m_Staubli->basic_Jacobian, temp_euler, m_Staubli->Jacobian);

	//for(int i=0; i<6; i++){
	//	for(int j=0; j<6; j++)
	//		cout<<m_Staubli->Jacobian(i, j)<<"	";
	//	cout<<endl;
	//}

	//// 아래 자코비안은 mathmatica를 통해 symbolic하게 구한 것이고 mathmatica와 비교를 통해 맞는 것을 검증했다.
	m_Staubli->Jacobian.resize(6, 6);
	VectorXd _q(6);
	_q = m_Staubli->_q; //// stsaubli 값 반영.
	//_q(0) = 0, _q(1) = 55.7*H_DtoR, _q(2) = 113.06*H_DtoR, _q(3) = 0, _q(4) = 11.24*H_DtoR, _q(5) = 0;

	m_Staubli->Jacobian(0, 0) = -20*cos(_q(0)) - 290*sin(_q(0))*sin(_q(1)) + 310*(-1*cos(_q(2))*sin(_q(0))*sin(_q(1)) 
	- cos(_q(1))*sin(_q(0))*sin(_q(2))) + 220*(-1.0*cos(_q(4))*(cos(_q(2))*sin(_q(0))*sin(_q(1)) 
	+ cos(_q(1))*sin(_q(0))*sin(_q(2))) + (cos(_q(3))*(-cos(_q(1))*cos(_q(2))*sin(_q(0)) 
	+ sin(_q(0))*sin(_q(1))*sin(_q(2))) - cos(_q(0))*sin(_q(3)))*sin(_q(4)));
	
	m_Staubli->Jacobian(0, 1) = -sin(_q(0)); 

	m_Staubli->Jacobian(0, 2) =310*(cos(_q(0))*cos(_q(1))*cos(_q(2)) - cos(_q(0))*sin(_q(1))*sin(_q(2))) 
	+ 220*(-cos(_q(4))*(-cos(_q(0))*cos(_q(1))*cos(_q(2)) + cos(_q(0))*sin(_q(1))*sin(_q(2))) 
	+ cos(_q(3))*(-cos(_q(0))*cos(_q(2))*sin(_q(1)) - cos(_q(0))*cos(_q(1))*sin(_q(2)))*sin(_q(4))); 

	m_Staubli->Jacobian(0, 3) = 220*(-cos(_q(3))*sin(_q(0)) - (cos(_q(0))*cos(_q(1))*cos(_q(2)) - cos(_q(0))*sin(_q(1))*sin(_q(2)))*sin(_q(3)))*sin(_q(4)); 

	m_Staubli->Jacobian(0, 4) = 220*(cos(_q(4))*(cos(_q(3))*(cos(_q(0))*cos(_q(1))*cos(_q(2)) - cos(_q(0))*sin(_q(1))*sin(_q(2))) 
	- sin(_q(0))*sin(_q(3))) + (-cos(_q(0))*cos(_q(2))*sin(_q(1)) - cos(_q(0))*cos(_q(1))*sin(_q(2)))*sin(_q(4))); 

	m_Staubli->Jacobian(0, 5) = 0; 

	m_Staubli->Jacobian(1, 0) = -20*sin(_q(0)) + 290*cos(_q(0))*sin(_q(1)) + 310*(cos(_q(0))*cos(_q(2))*sin(_q(1)) + cos(_q(0))*cos(_q(1))*sin(_q(2))) 
	+ 220*(-cos(_q(4))*(-cos(_q(0))*cos(_q(2))*sin(_q(1)) - cos(_q(0))*cos(_q(1))*sin(_q(2))) 
	+ (cos(_q(3))*(cos(_q(0))*cos(_q(1))*cos(_q(2)) - cos(_q(0))*sin(_q(1))*sin(_q(2))) - sin(_q(0))*sin(_q(3)))*sin(_q(4))); 
	
	m_Staubli->Jacobian(1, 1) = cos(_q(0)); 
	
	m_Staubli->Jacobian(1, 2) = 310*(cos(_q(1))*cos(_q(2))*sin(_q(0)) - sin(_q(0))*sin(_q(1))*sin(_q(2))) 
	+ 220*(-cos(_q(4))*(-cos(_q(1))*cos(_q(2))*sin(_q(0)) + sin(_q(0))*sin(_q(1))*sin(_q(2))) 
	+ cos(_q(3))*(-cos(_q(2))*sin(_q(0))*sin(_q(1)) - cos(_q(1))*sin(_q(0))*sin(_q(2)))*sin(_q(4))); 
	
	m_Staubli->Jacobian(1, 3) = 220*(cos(_q(0))*cos(_q(3)) - (cos(_q(1))*cos(_q(2))*sin(_q(0)) 
	- sin(_q(0))*sin(_q(1))*sin(_q(2)))*sin(_q(3)))*sin(_q(4)); 
	
	m_Staubli->Jacobian(1, 4) = 220*(cos(_q(4))*(cos(_q(3))*(cos(_q(1))*cos(_q(2))*sin(_q(0)) - sin(_q(0))*sin(_q(1))*sin(_q(2))) 
	+ cos(_q(0))*sin(_q(3))) + (-cos(_q(2))*sin(_q(0))*sin(_q(1)) - cos(_q(1))*sin(_q(0))*sin(_q(2)))*sin(_q(4))); 
	
	m_Staubli->Jacobian(1, 5) = 0; 

	m_Staubli->Jacobian(2, 0) = 0; 
	
	m_Staubli->Jacobian(2, 1) = 0; 
	
	m_Staubli->Jacobian(2, 2) = 310*(-cos(_q(2))*sin(_q(1)) - cos(_q(1))*sin(_q(2))) + 220*(-cos(_q(4))*(cos(_q(2))*sin(_q(1)) 
	+ cos(_q(1))*sin(_q(2))) + cos(_q(3))*(-cos(_q(1))*cos(_q(2)) + sin(_q(1))*sin(_q(2)))*sin(_q(4))); 
	
	m_Staubli->Jacobian(2, 3) = -220*(-cos(_q(2))*sin(_q(1)) - cos(_q(1))*sin(_q(2)))*sin(_q(3))*sin(_q(4)); 
	
	m_Staubli->Jacobian(2, 4) = 220*(cos(_q(3))*cos(_q(4))*(-cos(_q(2))*sin(_q(1)) - cos(_q(1))*sin(_q(2))) 
	+ (-cos(_q(1))*cos(_q(2)) + sin(_q(1))*sin(_q(2)))*sin(_q(4))); 
	
	m_Staubli->Jacobian(2, 5) = 0; 

	m_Staubli->Jacobian(3, 0) = -sin(_q(0)); 
	
	m_Staubli->Jacobian(3, 1) = -sin(_q(0)); 

	m_Staubli->Jacobian(3, 2) = cos(_q(0))*cos(_q(2))*sin(_q(1)) + cos(_q(0))*cos(_q(1))*sin(_q(2)); 
	
	m_Staubli->Jacobian(3, 3) = -cos(_q(3))*sin(_q(0)) - (cos(_q(0))*cos(_q(1))*cos(_q(2)) - cos(_q(0))*sin(_q(1))*sin(_q(2)))*sin(_q(3)); 
	  
	m_Staubli->Jacobian(3, 4) = -cos(_q(4))*(-cos(_q(0))*cos(_q(2))*sin(_q(1)) - cos(_q(0))*cos(_q(1))*sin(_q(2))) 
	+ (cos(_q(3))*(cos(_q(0))*cos(_q(1))*cos(_q(2)) - cos(_q(0))*sin(_q(1))*sin(_q(2))) 
	- sin(_q(0))*sin(_q(3)))*sin(_q(4)); 
	  
	m_Staubli->Jacobian(3, 5) = -cos(_q(4))*(-cos(_q(0))*cos(_q(2))*sin(_q(1)) - cos(_q(0))*cos(_q(1))*sin(_q(2))) 
	+ (cos(_q(3))*(cos(_q(0))*cos(_q(1))*cos(_q(2)) - cos(_q(0))*sin(_q(1))*sin(_q(2))) 
	- sin(_q(0))*sin(_q(3)))*sin(_q(4)); 

	m_Staubli->Jacobian(4, 0) = cos(_q(0));

	m_Staubli->Jacobian(4, 1) = cos(_q(0)); 

    m_Staubli->Jacobian(4, 2) = cos(_q(2))*sin(_q(0))*sin(_q(1)) + cos(_q(1))*sin(_q(0))*sin(_q(2)); 

    m_Staubli->Jacobian(4, 3) = cos(_q(0))*cos(_q(3)) - (cos(_q(1))*cos(_q(2))*sin(_q(0)) - sin(_q(0))*sin(_q(1))*sin(_q(2)))*sin(_q(3)); 
	  
    m_Staubli->Jacobian(4, 4) = -cos(_q(4))*(-cos(_q(2))*sin(_q(0))*sin(_q(1)) - cos(_q(1))*sin(_q(0))*sin(_q(2))) 
                                + (cos(_q(3))*(cos(_q(1))*cos(_q(2))*sin(_q(0)) - sin(_q(0))*sin(_q(1))*sin(_q(2))) 
                                + cos(_q(0))*sin(_q(3)))*sin(_q(4)); 

	m_Staubli->Jacobian(4, 5) = -cos(_q(4))*(-cos(_q(2))*sin(_q(0))*sin(_q(1)) - cos(_q(1))*sin(_q(0))*sin(_q(2))) 
	+ (cos(_q(3))*(cos(_q(1))*cos(_q(2))*sin(_q(0)) - sin(_q(0))*sin(_q(1))*sin(_q(2))) 
	+ cos(_q(0))*sin(_q(3)))*sin(_q(4)); 
	  
	m_Staubli->Jacobian(5, 0) = 0; 

	m_Staubli->Jacobian(5, 1) = 0;

	m_Staubli->Jacobian(5, 2) = cos(_q(1))*cos(_q(2)) - sin(_q(1))*sin(_q(2)); 

    m_Staubli->Jacobian(5, 3) = -(-cos(_q(2))*sin(_q(1)) - cos(_q(1))*sin(_q(2)))*sin(_q(3));

    m_Staubli->Jacobian(5, 4) = -cos(_q(4))*(-cos(_q(1))*cos(_q(2)) + sin(_q(1))*sin(_q(2))) 
	  + cos(_q(3))*(-cos(_q(2))*sin(_q(1)) - cos(_q(1))*sin(_q(2)))*sin(_q(4)); 

	m_Staubli->Jacobian(5, 5) = -cos(_q(4))*(-cos(_q(1))*cos(_q(2)) + sin(_q(1))*sin(_q(2))) 
	  + cos(_q(3))*(-cos(_q(2))*sin(_q(1)) - cos(_q(1))*sin(_q(2)))*sin(_q(4));
}

void T_Omni::StaubliIK(Robot *m_Staubli, Matrix4d &_des_T, VectorXd &_dq, double _damp_param)
{
	while ( (_des_T - m_Staubli->T06).col(3).segment(0, 3).norm() > 1 ){ // 1mm resolution의 IK임. ㅋ
		// IK 계산
		GetInverseKin(m_Staubli, _des_T, _dq, 0.05*(_des_T - m_Staubli->T06).col(3).segment(0, 3).norm());		// dq 계산해내기.
		_dq = _dq * H_DtoR;										// H_DtoR하지 않으면 너무 값이 커져버린다.

		// 로봇 update
		m_Staubli->_q = m_Staubli->_q + _dq;				// q 값 갱신
		StaubliFK(m_Staubli, m_Staubli->_q);		// q 값에 의한 FK 값 갱신 => 현재의 transform을 구해오기 위해서.
		StaubliJacobian(m_Staubli);				// 자코비안 갱신
				
		m_Staubli->_q = LPF1(0.99, m_Staubli->_q + _dq, m_Staubli->_q);

		//cout<<"dq in degree: "<<dq*H_RtoD<<endl;
		//cout<<"current theta : "<<initq * H_RtoD<<endl;
		Matrix3d testori;
		Vector3d tetetete;
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++)
				testori(i, j) = m_Staubli->T06(i, j);
		MakestaubliOri(testori, tetetete);
		//MakestuabliOri에서 (0, 0, -90)을 빼주니 다시 더해주고 (0, -180, 0) 을 더해주므로 다시 빼주자.
		//cout<<"current ori "<<tetetete(0)<<" "<<tetetete(1) + 180<<" "<<tetetete(2) - 90<<endl;
		//cout<<"current x : "<<omst.Staubli.T06(0, 3)<<" "<<omst.Staubli.T06(1, 3)<<" "<<omst.Staubli.T06(2, 3)<<endl;
	}


	//////////////////////////////////////////////////////////////////////

	////// _x가 몇 자유도로 들어오는 지 잘 생각해 보아야 한다. 
	//VectorXd _dist_x(3);
	//
	//_dist_x = _des_T.col(3).segment(0, 3) - m_Staubli->T06.col(3).segment(0, 3);
	////cout<<_dist_x<<endl;

	//VectorXd _dx(6);
	//
	////while(_dist_x.norm() > 0.01){

	//	Vector3d _s1, _s2, _s3, _s1d, _s2d, _s3d;
	//	_s1 = m_Staubli->T06.col(0).segment(0, 3), _s2 = m_Staubli->T06.col(1).segment(0, 3), _s3 = m_Staubli->T06.col(2).segment(0, 3);
	//	_s1d = _des_T.col(0).segment(0, 3), _s2d = _des_T.col(1).segment(0, 3), _s3d = _des_T.col(2).segment(0, 3);
	//	//// Generate dx.
	//	for(int i=0; i<3; i++)
	//		_dx(i) = m_Staubli->T06.col(i).segment(0, 3).dot(_dist_x);
	//	_dx(0) = _s1.dot(_dist_x);
	//	_dx(1) = _s2.dot(_dist_x);
	//	_dx(2) = _s3.dot(_dist_x);
	//	//// 일단 orientation은 잘 수렴이 안된다.
	//	_dx(3) = 0.5*(_s3.dot(_s2d) - _s3d.dot(_s2));//(m_Staubli->T06.col(2).segment(0, 3).dot(_des_T.col(1).segment(0, 3)) - _des_T.col(2).segment(0, 3).dot(m_Staubli->T06.col(1).segment(0, 3)));
	//	_dx(4) = 0.5*(_s1.dot(_s3d) - _s1d.dot(_s3));//(m_Staubli->T06.col(0).segment(0, 3).dot(_des_T.col(2).segment(0, 3)) - _des_T.col(0).segment(0, 3).dot(m_Staubli->T06.col(2).segment(0, 3)));
	//	_dx(5) = 0.5*(_s2.dot(_s1d) - _s2d.dot(_s1));//(m_Staubli->T06.col(1).segment(0, 3).dot(_des_T.col(0).segment(0, 3)) - _des_T.col(1).segment(0, 3).dot(m_Staubli->T06.col(0).segment(0, 3)));

	//	//// Change Coordinate.
	//	MatrixXd Rot(6, 6);
	//	Rot.setZero();
	//	for(int i=0; i<3; i++){
	//		Rot.col(i).segment(0, 3) = m_Staubli->T06.col(i).segment(0, 3);
	//		Rot.col(i+3).segment(3, 3) = m_Staubli->T06.col(i).segment(0, 3);
	//	}

	//	_dx = Rot * _dx;

	//	JacobiSVD<MatrixXd> svd(m_Staubli->Jacobian, ComputeFullU | ComputeFullV); //matrix가 square가 아니면 ComputeThinU
	//	//cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(_dx) << endl;


	//	MatrixXd _Jacobian_pinv(6, 6);
	//	MatrixXd singularvals(6, 6);
	//	singularvals = svd.singularValues().asDiagonal();

	//	double pinvtoler = max(m_Staubli->Jacobian.rows(), m_Staubli->Jacobian.cols()) * m_Staubli->Jacobian.norm() * 2.22*exp(-16.0); ///tolerence 없으면 발산하는 부분이 발생한다. 
	//	MatrixXd singularvals_inv(6, 6);
	//	singularvals_inv.setZero();
	//	for(int i=0; i<6; i++){
	//		if(singularvals(i, i) > pinvtoler)
	//			singularvals_inv(i, i) = 1/singularvals(i, i);
	//	}

	//	//cout<<"Its singular values are : "<<endl<<svd.singularValues()<<endl;
	//	//for(int i=0; i<6; i++){
	//	//	for(int j=0; j<6; j++)
	//	//		cout<<singularvals(i, j)<<"	";
	//	//	cout<<endl;
	//	//}
	//	//cout<<"Its singular values inverse are : "<<endl<<svd.singularValues()<<endl;
	//	//for(int i=0; i<6; i++){
	//	//	for(int j=0; j<6; j++)
	//	//		cout<<singularvals_inv(i, j)<<"	";
	//	//	cout<<endl;
	//	//}

	//	//// 아래 jacobian pinv는 matlab과 비교하여 옳다는 것을 검증했음.
	//	_Jacobian_pinv = svd.matrixV() * singularvals_inv * svd.matrixU().transpose();

	//	//// 이거는 보통 사용하는 것들..
	//	//MatrixXd _jacobian_square(6, 6);
	//	//_jacobian_square = m_Staubli->Jacobian * m_Staubli->Jacobian.transpose();
	//	//_Jacobian_pinv = m_Staubli->Jacobian.transpose() * _jacobian_square.inverse();

	//	_dq.resize(6);
	//	_dq = _Jacobian_pinv * _dx;

	//	//// use damped least square method. 
	//	//// It can be easily seen that the joint speeds are only zero if e has become zero. 
	//	//// A problem arises, however, when the end-effector has to go through a singularity to get to its goal. 
	//	//// Then, the solution to J^+ “explodes” and joint speeds go to infinity. 
	//	//// In order to work around this, we can introduce damping to the controller.
	//	//// 출처 : 에이치티티피:://correll.cs.colorado.edu/?p=1958
	//	MatrixXd _mat_Identity(6, 6);
	//	_mat_Identity.setIdentity();
	//	//// damping 튜닝하는 방법에 대한 스터디도 많다고 함.. 그래도 대부분 휴리스틱 하다고 함.
	//	//// 해보니깐 5.0은 약간 왔다리갔다리 하는 편이고 10.0은 지나치게 damped 되는 느낌이 있음. 느림.
	//	//// adaptive하게 튜닝해 주는 것이 좋겠다. 초반에는 damping을 크게 하다가 나중에는 적게 하는 방식으로!
	//	//// 글구 이건 논문 거리가 될 지는 모르겠는데, damping이 작으면 부드러운 곡선으로 다가가질 않고 이쪽저쪽 튀는 느낌이다.
	//	//// damping이 지나치게 작으면 오히려 값에 수렴하는 데 너무 오래 걸		
	//	//// damping에 0을 넣으면 jacobian의 inverse와 똑같아져 버리는데, 이건 존재하지 않을 경우가 많다. (pseudo inverse가 아니게 됨)
	//	//// 목표지점과의 거리차 * 0.01을 하면 적당한 속도로 도착하는 것 같고, 0.1을 하면 오히려 안좋다. 기준이 뭘까?
	//	//// 0.05일 때에도 도달 못하네.. 숫자의 차이에서 오는 차이는 이해를 못 하겠지만, 일단 짐작으로는 거의 도달했을 때 너무 damping이 
	//	//// 작아지면 이쪽저쪽 발산하느라 안되는 것으로 생각된다. 일정 값 이하일 때에는 미니멈 리밋 값을 줘야겠다.
	//	//// _dq = (m_Staubli->Jacobian.transpose() * m_Staubli->Jacobian + _damp_param*_damp_param*_mat_Identity).inverse() * _Jacobian_pinv * _dx;

	//	if(_damp_param < 2)
	//		_damp_param = 2;
	//	// 아래 식은 출처논문 : Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods
	//	_dq = m_Staubli->Jacobian.transpose() * (m_Staubli->Jacobian * m_Staubli->Jacobian.transpose() + _damp_param*_damp_param*_mat_Identity).inverse() * _dx;

	//	//cout<<"_dx : "<<_dx<<endl;
	//	//cout<<"_dq : "<<_dq<<endl;

	////}

	////MatrixXd Jacobian_inv(6, 6);
	//
	////if(m_Staubli->_Jacobian.determinant() < 0.1){
	////	Jacobian_inv = m_Staubli->_Jacobian.transpose();
	////	cout<<"transpose"<<endl;
	////}
	////else{
	////	Jacobian_inv = m_Staubli->_Jacobian.inverse() * m_Staubli->_Jacobian.transpose();
	////	cout<<"pseudo inverse"<<endl;
	////}

	////JacobiSVD<MatrixXd> svd(m_Staubli->_Jacobian, ComputeFullU | ComputeFullV); //matrix가 square가 아니면 ComputeThinU
	////cout<<"Its singular values are : "<<endl<<svd.singularValues()<<endl;
	////cout<<"Its left singular vectors are the columns of the U matrix : " <<endl<<svd.matrixU()<<endl;
	////cout<<"Its right singular vectors are the columns of the V matrix : " <<endl<<svd.matrixV()<<endl;

	////cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(_dx) << endl;
}
*/

