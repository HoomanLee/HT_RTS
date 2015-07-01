#include "Precompiled.h"  
#include "TeachingSW_HM.h"

TeachingSW_HM::TeachingSW_HM()
{
	pb_stepper = 0;
	hash_track = 0;

	pb_stepper_R = 0;
	pb_stepper_L = 0;
	hash_track_R = 0;
	hash_track_L = 0;
	event_time_R = 0.0;
	event_time_L = 0.0;

	bool_Teleoperation = false;
	bool_Playback = false;
	bool_vision = false;
	bool_load = false;
	event_time = 0.0;

	des_x.resize(6); //일단 cartesian 좌표계는 size가 6으로 fixed.
	cur_x.resize(6);

	des_q.resize(6); //일단 joint 좌표계는 size가 6으로 fixed. 필요시 다른 곳에서 리사이즈하면 될 것 같다. 
	cur_q.resize(6);

	event_x.resize(6);
	event_q.resize(6);
	home_x.resize(6);
	home_q.resize(6);

}
TeachingSW_HM::~TeachingSW_HM()
{
	if(m_robot)
		delete m_robot;
	if(m_vp)
		delete m_vp;
	if(nominal_robot)
		delete nominal_robot;
}


/////////////////////////////
/////// TeachingSW 기본 기능
bool TeachingSW_HM::save_taught_data_p(const char* file_name, VectorXd cartesian_data, MatrixXd& T_data)
{
	T_data.conservativeResize(T_data.rows() + 1, cartesian_data.size());
	for(int i=0; i<cartesian_data.size(); i++)
		T_data(T_data.rows()-1, i) = cartesian_data(i); //마지막 행의 index는 matrix의 row의 개수-1

	cout<<file_name<<"T_data_p saved "<<T_data.rows()<<endl;
	for(int i=0; i<cartesian_data.size(); i++)		
		cout<<"  "<<cartesian_data(i);
	cout<<endl;

	// save as file
	ofstream myfile(file_name, ios::out);
	//myfile_R<<"Right Arm Teaching data"<<endl; // file descriptor로 넣었으나.. 일단은 제외..

	for (int i =0; i<T_data.rows(); i++){
		for (int j=0; j<T_data.cols(); j++)
			myfile<<T_data(i,j)<<" ";
		myfile<<endl;
	}

	if (myfile.rdstate() == ios::failbit || myfile.rdstate() == ios::badbit)
		return false;
	else
		return true;

	myfile.close();
}

bool TeachingSW_HM::save_taught_data_q(const char* file_name, VectorXd joint_data, MatrixXd& T_data)
{
	T_data.conservativeResize(T_data.rows() + 1, joint_data.size());
	for(int i=0; i<joint_data.size(); i++)
		T_data(T_data.rows()-1, i) = joint_data(i); //마지막 행의 index는 matrix의 row의 개수-1

	cout<<file_name<<"T_data_q saved "<<T_data.rows()<<endl;
	for(int i=0; i<joint_data.size(); i++)		
		cout<<"  "<<joint_data(i);
	cout<<endl;

	// save as file
	ofstream myfile(file_name, ios::out);
	//myfile_R<<"Right Arm Teaching data"<<endl; // file descriptor로 넣었으나.. 일단은 제외..

	for (int i =0; i<T_data.rows(); i++){
		for (int j=0; j<T_data.cols(); j++)
			myfile<<T_data(i,j)<<" ";
		myfile<<endl;
	}

	if (myfile.rdstate() == ios::failbit || myfile.rdstate() == ios::badbit)
		return false;
	else
		return true;

	myfile.close();
}

bool TeachingSW_HM::save_taught_data_T(const char* file_name, MatrixXd transform_data, MatrixXd& T_data)
{
	T_data.conservativeResize(T_data.rows() + 1, 12);

	for(int i=0; i<3; i++){
		T_data(T_data.rows()-1, i) = transform_data(i, 0); //마지막 행의 index는 matrix의 row의 개수-1
		T_data(T_data.rows()-1, i+3) = transform_data(i, 1);
		T_data(T_data.rows()-1, i+6) = transform_data(i, 2);
		T_data(T_data.rows()-1, i+9) = transform_data(i, 3);
	}

	cout<<file_name<<"T_data_Tr saved "<<T_data.rows()<<endl;
	cout<<T_data(T_data.rows())<<endl;
	
	// save as file
	ofstream myfile(file_name, ios::out);
	//myfile_R<<"Right Arm Teaching data"<<endl; // file descriptor로 넣었으나.. 일단은 제외..

	for (int i =0; i<T_data.rows(); i++){
		for (int j=0; j<T_data.cols(); j++)
			myfile<<T_data(i,j)<<" ";
		myfile<<endl;
	}

	if (myfile.rdstate() == ios::failbit || myfile.rdstate() == ios::badbit)
		return false;
	else
		return true;

	myfile.close();
}

bool TeachingSW_HM::save_countinuous_path(bool mkfile, const char* file_name, VectorXd current_data, MatrixXd& T_data) // 어느 정도 촘촘한 데이터를 할 것인지 구현도 필요한가..
{
	T_data.conservativeResize(T_data.rows() + 1, current_data.size());
	for(int i=0; i<current_data.size(); i++)
		T_data(T_data.rows()-1, i) = current_data(i); //마지막 행의 index는 matrix의 row의 개수-1

	if(mkfile == true){
		// save as file
		ofstream myfile(file_name, ios::out);
		//myfile_R<<"Right Arm Teaching data"<<endl; // file descriptor로 넣었으나.. 일단은 제외..

		for (int i =0; i<T_data.rows(); i++){
			for (int j=0; j<T_data.cols(); j++)
				myfile<<T_data(i,j)<<" ";
			myfile<<endl;
		}

		if (myfile.rdstate() == ios::failbit || myfile.rdstate() == ios::badbit)
			return false;
		else
			return true;
		myfile.close();
	}
	else	
		return true;
}

void TeachingSW_HM::Addheader(const char* dirSrc, const char* dirDst)
{
	/// ----------------------------------------------------
	/// 함수 설명: 
	/// 원본 파일을 읽어서 가장 윗줄에 해당 파일의 row데이터 
	/// 개수와 column 데이터 개수를 nCol, nRow 의 순으로
	/// 추가하여 새 파일을 만든다. 
	///
	/// ex) Addheader(dirSrc, dirDst);
	/// dirSrc: 원본 파일 경로
	/// dirDst: 헤더를 추가하여 생성할 파일 경로
	///
	/// 필요한 헤더파일은 stdio.h, string.h 두가지..
	/// ----------------------------------------------------


	FILE *src, *dst;
	
	char buf[256];

	int nRow = 0;
	int nCol = 0;
	int chkRflag = 0;
	
	src = fopen(dirSrc, "r");
	if(src!=NULL) {
		for(;;) {
			if(fgets(buf,256,src) == NULL) {
				break;
			}
			nCol++;

			// 첫 줄만 검사
			if(chkRflag == 0) {
				for(int i=0; i<256; i++)
				{
					if(buf[i] == 32)	// ASCii code space
						nRow++;
				}
				chkRflag = 1;
			}	
			//printf("%s", buf);
		}
	}
	else {
		printf("Failed to open file \n");
	}
	//printf("nCol: %d, nRow: %d \n", nCol, nRow);


	// 함수 테스트 용으로 새로운 파일 생성
	dst = fopen(dirDst, "wb");

	// row and col buffer
	char header[100] = {0};
	
	if(dst!=NULL)
	{
		fseek(src, 0L, SEEK_SET);	// src 파일의 처음 위치로
		sprintf(header, "%d %d\n", nCol, nRow);	// format대로 문자열 만들기 
		
		fputs(header, dst);
		while(!feof(src) && fgets(buf, 256, src) != NULL) {
			fputs(buf, dst);
		}
		
		fclose(src);
		fclose(dst);
	}
}

bool TeachingSW_HM::save_hash_table(const char* _path_hash, string _label)
{
	// save as file
	ofstream myfile(_path_hash, ios::app);

	myfile<<_label<</*" "*/endl;

	if (myfile.rdstate() == ios::failbit || myfile.rdstate() == ios::badbit)
		return false;
	else
		return true;

	myfile.close();
}

bool TeachingSW_HM::decode_hash_table(const char* _path_hash, int &_numindx, int &_gripindx, int &_releaseindx, int &_gripindxV, int &_releaseindxV)
{
	// To do... _gripindx와 _releaseindx의 위치만 찾아낸다?
	int indx = 0;
	ifstream myfile(_path_hash);
	if(myfile.is_open())
	{
		string line;
		while( getline(myfile, line) ){
			indx++;
			cout<<line<<endl;
			if(line == "Open"){
				cout<<"Find Open!!!"<<endl;
				_releaseindx = indx;
			}
			if(line == "Close"){
				cout<<"Find Close!!!"<<endl;
				_gripindx = indx;
			}
			if(line == "OpenV"){
				cout<<"Find Open with Vison!!!"<<endl;
				_releaseindxV = indx;
			}
			if(line == "CloseV"){
				cout<<"Find Close with Vison!!!"<<endl;
				_gripindxV = indx;
			}
			_numindx = indx;
		}
		cout<<"Hash table decoded!"<<endl;
		return true;
	}
	else{
		cout<<"Hash table.txt not founded!"<<endl;
		return false;
	}

}

bool TeachingSW_HM::load_taught_data(const char* file_name, MatrixXd& taught_data)
{
	cout<<"Playback : "<<file_name<<endl;

	ifstream myfile(file_name);
	if(myfile.is_open())
	{
		string line;
		while( getline(myfile, line) )
			cout<<line<<endl;

		vector <double> t_vec;
		int rows = 1, cols = 1;
		import_matrix_from_txt_file(file_name, t_vec, rows, cols);
		taught_data.resize(rows, cols); 

		for (int i=0; i<rows; i++)
			for (int j=0; j<cols; j++)
				taught_data(i, j) = t_vec[i*cols+j];

		myfile.close();
		
		return true;
	}
	else{
		cout << "unable to open file" <<endl;
		return false;
	}	
}
void TeachingSW_HM::import_matrix_from_txt_file(const char* filename_X, vector <double>& v, int& rows, int& cols){
    
    ifstream file_X;
    string line;
    
    file_X.open(filename_X);
    if (file_X.is_open())
    {
        int i=0;
        getline(file_X, line);
        
		istringstream is( line );
		double n;
		while( is >> n ) {
			v.push_back( n );
		}
		cols = v.size();

        cout << "cols:" << cols << endl;
        
		/// 아마도 32767이 max일겨.. 2^15.. 
        for ( i=1;i<32767;i++){
            if ( getline(file_X, line) == 0 ) 
				break;
			istringstream is( line );
			double n;
			while( is >> n ) {
				v.push_back( n );
			}            
        }
        
        rows=i;
        cout << "rows :" << rows << endl;
        if(rows >32766) cout<< "N must be smaller than MAX_INT";
        
        file_X.close();
    }
    else{
        cout << "file open failed";
    }
    
    cout << "v:" << endl;
    for (int i=0;i<rows;i++){
        for (int j=0;j<cols;j++){
            cout << v[i*cols+j] <<"\t";
        }
        cout << endl;
    }
}

void TeachingSW_HM::clear_jobfile(const char* file_name)
{
	// save as file
	ofstream myfile(file_name, ios::out);
	myfile.close();
}

/////// Playback algorithms
/// 아래는 hash table 따로 decoding없이 바로 가도록 하는 것..
void TeachingSW_HM::playback_demo(ViaPoint _vp, Robot *_robot, int &_hash_track, int _subhash_GR, 
						int _jdof, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback)
{
	double via_time = _via_time;
	
	_Desired_q.resize(_jdof);

	if (_vp.GetNum() == 0){
		cout<<"There's no teaching data"<<endl;
		_bool_playback = false;
	}
	else{
		if (_hash_track == 0)
		{
			for (int i =0; i < _jdof; i++){
				_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + (_hash_track+1) * via_time, _event_q_0(i), _q_home(i), 0.0, 0.0);
			}
			//JointSpaceController(m_M, _gravity_torque, 400.0, 40.0, _q_sensed, _q_des, _qdot_sensed, _qdot_des, _torque_des);
			//cout<<"_event_time_0 : "<<_event_time_0<<" _end_time : "<<_event_time_0 + (_pb_stepper+1) * via_time<<
			//	" event_x_0 : "<<_event_x_0(4)<<" x_home(4) : "<<_x_home(4)<<" path : "<<_Desired_path(4)<<endl;			
		
			// 일단 joint space에서의 값만 체크하자. task space는 나중에..
			if(Is_reached(_q_home, _current_q, _resolution)){
				cout<<"Get home to play back! Now start!"<<endl;
				_event_q_0 = _current_q;
				_event_time_0 = _time;        // 플레이백 시작할 때의 초기 시간 저장
				_hash_track++;
				cout<<"event_time"<<_hash_track<<" : "<<_event_time_0<<endl;
			}
		}

		/////////
		else
		{
			if(!strcmp(_vp.GetHash(_hash_track-1),"Open")){
				if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
					cout<<"tcp connected.. gripper Close!"<<endl;
					_tcpip.SendData_TBtnCMD(BTN_2_LONG);
					_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
				}
				else{
					cout<<"tcp not connected.. gripper Close!"<<endl;
					_event_time_0 = _time;
				}
				_hash_track++;
			}
			else if(!strcmp(_vp.GetHash(_hash_track-1),"Close")){
				if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
					cout<<"tcp connected.. gripper Close!"<<endl;
					_tcpip.SendData_TBtnCMD(BTN_2_LONG);
					_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
				}
				else{
					cout<<"tcp not connected.. gripper Close!"<<endl;
					_event_time_0 = _time;
				}
				_hash_track++;
			}
			else if(!strcmp(_vp.GetHash(_hash_track-1),"OpenV")){
				
				//// vision 에서 움직이라는 신호 줄 때까지 대기
				//while()
				//	cout<<"waiting move command.."<<endl;	
								
				Matrix4d _goalT_Greach;
				
				/// Vision 에서 object ID까지 데이터를 받아와서 GoalT 생성.
				for(int r=0; r<3; r++){
					for(int c=0; c<3; c++){
						////// Vision 있을 때 없을 때.
						if(bool_vision) //vision 연결되어 있으면 playback 호출 순간에 snapshot 찍어야 함. 개선해야 할 부분..
							if(m_objV.ID == _vp.GetObjV_ID(_hash_track-1) && m_objV.ID != -1)
								_goalT_Greach(r, c) = GetRfrom_eXYZ(0.0, -180.0 * H_DtoR, m_objV.Rz)(r, c); // vision data에서는 yaw만 있음
						else
							_goalT_Greach(r, c) = GetRfrom_eXYZ(0.0, 158.10 * H_DtoR, 0.0)(r, c);
					}
					//_goal_T(r, 3) = snpshot(r); //snapshot에는 RGBY순서로 저장되어 있으니 그 값을 맞게 쓰자. 약속된 상자순서 ㅠㅠ
					////// Vision 있을 때 없을 때.
					if(bool_vision) //vision 연결되어 있으면 playback 호출 순간에 snapshot 찍어야 함. 개선해야 할 부분..
						// redbox x y z 
						_goalT_Greach(0, 3) = snpshot(0), _goalT_Greach(1, 3) = snpshot(1), _goalT_Greach(2, 3) = snpshot(2);
						// yellowbox x y z 
					else
						_goalT_Greach(0, 3) = 476.61, _goalT_Greach(1, 3) = 20.00, _goalT_Greach(2, 3) = -234.12;
				}
				//// GoalT 기준으로 4단계 IK 계산.
				VectorXd approachG_q(6), reachG_q(6), leave_q(6);
				IKLoop_grasp(_robot, _goalT_Greach, 1, 0.1, approachG_q, reachG_q, leave_q);

				//// 이제 아래에서 4단계 루프를 돌도록 만들어야 하는데 그걸 아직 잘 모르겠다.

				if(_subhash_GR == 1){
					//// 보간 알고리즘
					for(int i = 0; i < _robot->GetDOF(); i++){
						_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
							_event_q_0(i), approachG_q(i), 0.0, 0.0);
					}
					// 일단 joint space에서의 값만 체크하자. task space는 나중에..
					if(Is_reached(approachG_q, _current_q, _resolution)){
						cout<<"approachG_q passed!!"<<endl;
						_subhash_GR++;
						_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
						_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
						cout<<"event_time"<<_subhash_GR<<" : "<<_event_time_0<<endl;
					}
				}
				else if(_subhash_GR == 2){
					//// 보간 알고리즘
					for(int i = 0; i < _robot->GetDOF(); i++){
						_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
							_event_q_0(i), reachG_q(i), 0.0, 0.0);
					}
					// 일단 joint space에서의 값만 체크하자. task space는 나중에..
					if(Is_reached(reachG_q, _current_q, _resolution)){
						cout<<"reachG_q passed!!"<<endl;
						_subhash_GR++;
						_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
						_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
						cout<<"event_time"<<_subhash_GR<<" : "<<_event_time_0<<endl;
					}
				}
				else if(_subhash_GR == 3){
					if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
						cout<<"tcp connected.. gripper Close!"<<endl;
						_tcpip.SendData_TBtnCMD(BTN_2_LONG);
						_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
						m_objV.ID = -1;
					}
					else{
						cout<<"tcp not connected.. gripper Close!"<<endl;
						_event_time_0 = _time;
						m_objV.ID = -1;
					}
				}
				else if(_subhash_GR == 4){
					//// 보간 알고리즘
					for(int i = 0; i < _robot->GetDOF(); i++){
						_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
							_event_q_0(i), leave_q(i), 0.0, 0.0);
					}
					// 일단 joint space에서의 값만 체크하자. task space는 나중에..
					if(Is_reached(leave_q, _current_q, _resolution)){
						cout<<"leave_q passed!!"<<endl;
						_subhash_GR = 0;
						_hash_track++;
						_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
						_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
						cout<<"event_time"<<_subhash_GR<<" : "<<_event_time_0<<endl;
					}
				}
			}
			else if(!strcmp(_vp.GetHash(_hash_track-1),"CloseV")){
				if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
					cout<<"tcp connected.. gripper Close!"<<endl;
					_tcpip.SendData_TBtnCMD(BTN_2_LONG);
					_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
				}
				else{
					cout<<"tcp not connected.. gripper Close!"<<endl;
					_event_time_0 = _time;
				}
				_hash_track++;
			}
			else{
				//// 보간 알고리즘
				for(int i = 0; i < _Teaching_Data_q.cols(); i++){
					_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
						_event_q_0(i), _vp.GetJoint(_hash_track-1)(i), 0.0, 0.0);
				}

				// 일단 joint space에서의 값만 체크하자. task space는 나중에..
				if(Is_reached(_vp.GetJoint(_hash_track-1), _current_q, _resolution)){
					cout<<"Via point" << _hash_track <<"passed!!"<<endl;
					_hash_track++;
					_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
					_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
					cout<<"event_time"<<_hash_track<<" : "<<_event_time_0<<endl;
				}
			}
		}

		if ( _hash_track-1  == _vp.hash_numindx ){
			//_pb_stepper 초기화
			_hash_track = 0;
			cout<<"Play back finished!!"<<endl;
					
			_bool_playback = false;
		}

		/////////
	}

}

//// 아래 알고리즘은 Vision Boolean 없이 통합 플레이백 알고리즘을 만들기 위해서 개발한 코드이다.
void TeachingSW_HM::playback_finalist(ViaPoint _vp, Robot *_robot, int &_hash_track, 
						int _jdof, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback)
{
	double via_time = _via_time;
	
	_Desired_q.resize(_jdof);

	if (_vp.GetNum() == 0){
		cout<<"There's no teaching data"<<endl;
		_bool_playback = false;
	}
	else{
		if (_hash_track == 0)
		{
			for (int i =0; i < _jdof; i++){
				_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + (_hash_track+1) * via_time, _event_q_0(i), _q_home(i), 0.0, 0.0);
			}
			//JointSpaceController(m_M, _gravity_torque, 400.0, 40.0, _q_sensed, _q_des, _qdot_sensed, _qdot_des, _torque_des);
			//cout<<"_event_time_0 : "<<_event_time_0<<" _end_time : "<<_event_time_0 + (_pb_stepper+1) * via_time<<
			//	" event_x_0 : "<<_event_x_0(4)<<" x_home(4) : "<<_x_home(4)<<" path : "<<_Desired_path(4)<<endl;			
		
			// 일단 joint space에서의 값만 체크하자. task space는 나중에..
			if(Is_reached(_q_home, _current_q, _resolution)){
				cout<<"Get home to play back! Now start!"<<endl;
				_event_q_0 = _current_q;
				_event_time_0 = _time;        // 플레이백 시작할 때의 초기 시간 저장
				_hash_track++;
				cout<<"event_time"<<_hash_track<<" : "<<_event_time_0<<endl;
			}
		}
		else if (_hash_track >= 1)
		{
			/// gripper 동작이 있을 경우에만 아래 코드.. 없을 때에는 그냥 보간만.
			if(_vp.nToolcnt != 0){
				//// Tool 동작일 경우 (close, open)
				if(_hash_track == _vp.hash_gripindx && _vp.hash_gripindx != 0){
					if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
						cout<<"tcp connected.. gripper Close!"<<endl;
						_tcpip.SendData_TBtnCMD(BTN_2_LONG);
						_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
					}
					else{
						cout<<"tcp not connected.. gripper Close!"<<endl;
						_event_time_0 = _time;
					}
					_hash_track++;
				}
				else if(_hash_track == _vp.hash_releaseindx && _vp.hash_releaseindx != 0){
					if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
						cout<<"tcp connected.. gripper Open!"<<endl;
						_tcpip.SendData_TBtnCMD(BTN_2_SHORT);
						_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
					}
					else{
						cout<<"tcp not connected.. gripper Open!"<<endl;
						_event_time_0 = _time;
					}
					_hash_track++;
				}
				//// Tool 동작이 아닐 경우.
				else{ 
					//// approach grip
					if(_hash_track == _vp.hash_gripindxV - 2 && _vp.hash_gripindxV != 0){
				
						cout<<"tcp connected.. gripper approaching!"<<endl;
				
						for(int i=0; i<_robot->GetDOF(); i++)
							_Teaching_Data_q(_hash_track-1, i) = _vp.vp_Gapproach_q(i);
					}
					//// reach grip
					else if(_hash_track == _vp.hash_gripindxV - 1 && _vp.hash_gripindxV != 0){
				
						cout<<"tcp connected.. gripper reaching!"<<endl;

						for(int i=0; i<_robot->GetDOF(); i++)
							_Teaching_Data_q(_hash_track-1, i) = _vp.vp_Greach_q(i);
					}
					//// reach grip
					else if(_hash_track == _vp.hash_gripindxV + 1 && _vp.hash_gripindxV != 0){
				
						cout<<"tcp connected.. gripper leaving!"<<endl;

						for(int i=0; i<_robot->GetDOF(); i++)
							_Teaching_Data_q(_hash_track-1, i) = _vp.vp_Gleave_q(i);
					}
					//// approach release
					else if(_hash_track == _vp.hash_releaseindxV - 2 && _vp.hash_releaseindxV != 0){
				
						cout<<"tcp connected.. gripper approaching!"<<endl;
				
						for(int i=0; i<_robot->GetDOF(); i++)
							_Teaching_Data_q(_hash_track-1, i) = _vp.vp_Rapproach_q(i);
					}
					//// reach release
					else if(_hash_track == _vp.hash_releaseindxV - 1 && _vp.hash_releaseindxV != 0){
				
						cout<<"tcp connected.. gripper reaching!"<<endl;

						for(int i=0; i<_robot->GetDOF(); i++)
							_Teaching_Data_q(_hash_track-1, i) = _vp.vp_Rreach_q(i);
					}
					//// reach release
					else if(_hash_track == _vp.hash_releaseindxV + 1 && _vp.hash_releaseindxV != 0){
				
						cout<<"tcp connected.. gripper leaving!"<<endl;

						for(int i=0; i<_robot->GetDOF(); i++)
							_Teaching_Data_q(_hash_track-1, i) = _vp.vp_Rleave_q(i);
					}


					//// 보간 알고리즘
					for(int i = 0; i < _Teaching_Data_q.cols(); i++){
						_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
													_event_q_0(i), _Teaching_Data_q(_hash_track-1, i), 0.0, 0.0);
					}

					// 일단 joint space에서의 값만 체크하자. task space는 나중에..
					if(Is_reached(_Teaching_Data_q.row(_hash_track-1), _current_q, _resolution)){
						cout<<"Via point" << _hash_track <<"passed!!"<<endl;
						_hash_track++;
						_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
						_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
						cout<<"event_time"<<_hash_track<<" : "<<_event_time_0<<endl;
					}
				}
			}
			else {
				//// 보간 알고리즘
				for(int i = 0; i < _Teaching_Data_q.cols(); i++){
					_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
												_event_q_0(i), _Teaching_Data_q(_hash_track-1, i), 0.0, 0.0);
				}

				// 일단 joint space에서의 값만 체크하자. task space는 나중에..
				if(Is_reached(_Teaching_Data_q.row(_hash_track-1), _current_q, _resolution)){
					cout<<"Via point" << _hash_track <<"passed!!"<<endl;
					_hash_track++;
					_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
					_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
					cout<<"event_time"<<_hash_track<<" : "<<_event_time_0<<endl;
				}
			}
		
		
			if ( _hash_track-1 == _vp.hash_numindx ){
					//_pb_stepper 초기화
					_hash_track = 0;
					cout<<"Play back finished!!"<<endl;
					
					_bool_playback = false;
			}
		}
	}
}


void TeachingSW_HM::playback_woVsn2(ViaPoint _vp, Robot *_robot, int &_hash_track, int _hash_numindx, int _hash_gripindx, int _hash_releaseindx,
						int _jdof, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback)
{
	double via_time = _via_time;
	
	_Desired_q.resize(_jdof);

	if (_hash_track == 0)
	{
		for (int i =0; i < _jdof; i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + (_hash_track+1) * via_time, _event_q_0(i), _q_home(i), 0.0, 0.0);
		}
		//JointSpaceController(m_M, _gravity_torque, 400.0, 40.0, _q_sensed, _q_des, _qdot_sensed, _qdot_des, _torque_des);
		//cout<<"_event_time_0 : "<<_event_time_0<<" _end_time : "<<_event_time_0 + (_pb_stepper+1) * via_time<<
		//	" event_x_0 : "<<_event_x_0(4)<<" x_home(4) : "<<_x_home(4)<<" path : "<<_Desired_path(4)<<endl;			
		
		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		if(Is_reached(_q_home, _current_q, _resolution)){
			cout<<"Get home to play back! Now start!"<<endl;
			_event_q_0 = _current_q;
			_event_time_0 = _time;        // 플레이백 시작할 때의 초기 시간 저장
			_hash_track++;
			cout<<"event_time"<<_hash_track<<" : "<<_event_time_0<<endl;
		}
	}
	else if (_hash_track >= 1)
	{
		//// Tool 동작일 경우 (close, open)
		if(_hash_track == _vp.hash_gripindx){
			if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				cout<<"tcp connected.. gripper Close!"<<endl;
				_tcpip.SendData_TBtnCMD(BTN_2_LONG);
				_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
			}
			else{
				cout<<"tcp not connected.. gripper Close!"<<endl;
				_event_time_0 = _time;
			}
			_hash_track++;
		}
		else if(_hash_track == _vp.hash_releaseindx){
			if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				cout<<"tcp connected.. gripper Open!"<<endl;
				_tcpip.SendData_TBtnCMD(BTN_2_SHORT);
				_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
			}
			else{
				cout<<"tcp not connected.. gripper Open!"<<endl;
				_event_time_0 = _time;
			}
			_hash_track++;
		}
		//// 보간 알고리즘
		for(int i = 0; i < _Teaching_Data_q.cols(); i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
										_event_q_0(i), _Teaching_Data_q(_hash_track-1, i), 0.0, 0.0);
		}

		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		if(Is_reached(_Teaching_Data_q.row(_hash_track-1), _current_q, _resolution)){
			cout<<"Via point" << _hash_track <<"passed!!"<<endl;
			_hash_track++;
			_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
			_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
			cout<<"event_time"<<_hash_track<<" : "<<_event_time_0<<endl;
		}
		
		if ( _hash_track-1 == _vp.hash_numindx ){
				//_pb_stepper 초기화
				_hash_track = 0;
				cout<<"Play back finished!!"<<endl;
					
				_bool_playback = false;
		}
	}
}

void TeachingSW_HM::playback_woVsn(ViaPoint _vp, Robot *_robot, int &_hash_track, int _hash_numindx, int _hash_gripindx, int _hash_releaseindx,
						int _jdof, int& _pb_stepper, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback)
{
	double via_time = _via_time;
	
	_Desired_q.resize(_jdof);

	if (_pb_stepper == 0)
	{
		for (int i =0; i < _jdof; i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + (_pb_stepper+1) * via_time, _event_q_0(i), _q_home(i), 0.0, 0.0);
		}
		//JointSpaceController(m_M, _gravity_torque, 400.0, 40.0, _q_sensed, _q_des, _qdot_sensed, _qdot_des, _torque_des);
		//cout<<"_event_time_0 : "<<_event_time_0<<" _end_time : "<<_event_time_0 + (_pb_stepper+1) * via_time<<
		//	" event_x_0 : "<<_event_x_0(4)<<" x_home(4) : "<<_x_home(4)<<" path : "<<_Desired_path(4)<<endl;			
		
		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		if(Is_reached(_q_home, _current_q, _resolution)){
			cout<<"Get home to play back! Now start!"<<endl;
			_event_q_0 = _current_q;
			_event_time_0 = _time;        // 플레이백 시작할 때의 초기 시간 저장
			_pb_stepper++;
			_hash_track++;
			cout<<"event_time"<<_pb_stepper<<" : "<<_event_time_0<<endl;
		}
	}
	else if (_pb_stepper >= 1)
	{
		//// Tool 동작일 경우 (close, open)
		if(_hash_track == _vp.hash_gripindx){
			if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				cout<<"tcp connected.. gripper Close!"<<endl;
				_tcpip.SendData_TBtnCMD(BTN_2_LONG);
				_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
			}
			else
				cout<<"tcp not connected.. gripper Close!"<<endl;
			_hash_track++;
		}
		else if(_hash_track == _vp.hash_releaseindx){
			if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				cout<<"tcp connected.. gripper Open!"<<endl;
				_tcpip.SendData_TBtnCMD(BTN_2_SHORT);
				_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
			}
			else
				cout<<"tcp not connected.. gripper Open!"<<endl;
			_hash_track++;

		}
		//// 보간 알고리즘
		for(int i = 0; i < _Teaching_Data_q.cols(); i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
										_event_q_0(i), _Teaching_Data_q(_pb_stepper-1, i), 0.0, 0.0);
		}

		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		if(Is_reached(_Teaching_Data_q.row(_pb_stepper-1), _current_q, _resolution)){
			cout<<"Via point" << _pb_stepper <<"passed!!"<<endl;
			_pb_stepper++;
			_hash_track++;
			_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
			_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
			cout<<"event_time"<<_pb_stepper<<" : "<<_event_time_0<<endl;
		}
		
		if ( _hash_track-1 == _vp.hash_numindx ){
				//_pb_stepper 초기화
				_pb_stepper = 0;
				_hash_track = 0;
				cout<<"Play back finished!!"<<endl;
					
				_bool_playback = false;
		}
	}
}

void TeachingSW_HM::playback_wVsn(ViaPoint _vp, Robot *_robot, int &_hash_track, int _hash_numindx, int _hash_gripindx, int _hash_releaseindx,
						int _jdof, int& _pb_stepper, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback)
{
	double via_time = _via_time;
	
	_Desired_q.resize(_jdof);

	if (_pb_stepper == 0)
	{
		for (int i =0; i < _jdof; i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + (_pb_stepper+1) * via_time, _event_q_0(i), _q_home(i), 0.0, 0.0);
		}
		//JointSpaceController(m_M, _gravity_torque, 400.0, 40.0, _q_sensed, _q_des, _qdot_sensed, _qdot_des, _torque_des);
		//cout<<"_event_time_0 : "<<_event_time_0<<" _end_time : "<<_event_time_0 + (_pb_stepper+1) * via_time<<
		//	" event_x_0 : "<<_event_x_0(4)<<" x_home(4) : "<<_x_home(4)<<" path : "<<_Desired_path(4)<<endl;			
		
		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		if(Is_reached(_q_home, _current_q, _resolution)){
			cout<<"Get home to play back! Now start!"<<endl;
			_event_q_0 = _current_q;
			_event_time_0 = _time;        // 플레이백 시작할 때의 초기 시간 저장
			_pb_stepper++;
			_hash_track++;
			cout<<"event_time"<<_pb_stepper<<" : "<<_event_time_0<<endl;
		}
	}
	else if (_pb_stepper >= 1)
	{
		//// Tool 동작일 경우 (close, open)
		if(_hash_track == _vp.hash_gripindx){
			if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				cout<<"tcp connected.. gripper Close!"<<endl;
				_tcpip.SendData_TBtnCMD(BTN_2_LONG);
				_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
			}
			else
				cout<<"tcp not connected.. gripper Close!"<<endl;
			_hash_track++;
		}
		else if(_hash_track == _vp.hash_releaseindx){
			if(_tcpip.bConnected){ //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				cout<<"tcp connected.. gripper Open!"<<endl;
				_tcpip.SendData_TBtnCMD(BTN_2_SHORT);
				_event_time_0 = _time;        // staubli가 otime ctime 하는 동안 쉬어주고 다시 초기 시간 저장
			}
			else
				cout<<"tcp not connected.. gripper Open!"<<endl;
			_hash_track++;
		}
		//// Tool 동작이 아닐 경우.
		else{ 
			//// approach grip
			if(_hash_track == _vp.hash_gripindx - 2){
				
				cout<<"tcp connected.. gripper approaching!"<<endl;
				
				for(int i=0; i<_robot->GetDOF(); i++)
					_Teaching_Data_q(_pb_stepper-1, i) = _vp.vp_Gapproach_q(i);
			}
			//// reach grip
			else if(_hash_track == _vp.hash_gripindx - 1){
				
				cout<<"tcp connected.. gripper reaching!"<<endl;

				for(int i=0; i<_robot->GetDOF(); i++)
					_Teaching_Data_q(_pb_stepper-1, i) = _vp.vp_Greach_q(i);
			}
			//// reach grip
			else if(_hash_track == _vp.hash_gripindx + 1){
				
				cout<<"tcp connected.. gripper leaving!"<<endl;

				for(int i=0; i<_robot->GetDOF(); i++)
					_Teaching_Data_q(_pb_stepper-1, i) = _vp.vp_Gleave_q(i);
			}
			//// approach release
			else if(_hash_track == _vp.hash_releaseindx - 2){
				
				cout<<"tcp connected.. gripper approaching!"<<endl;
				
				for(int i=0; i<_robot->GetDOF(); i++)
					_Teaching_Data_q(_pb_stepper-1, i) = _vp.vp_Rapproach_q(i);
			}
			//// reach release
			else if(_hash_track == _vp.hash_releaseindx - 1){
				
				cout<<"tcp connected.. gripper reaching!"<<endl;

				for(int i=0; i<_robot->GetDOF(); i++)
					_Teaching_Data_q(_pb_stepper-1, i) = _vp.vp_Rreach_q(i);
			}
			//// reach release
			else if(_hash_track == _vp.hash_releaseindx + 1){
				
				cout<<"tcp connected.. gripper leaving!"<<endl;

				for(int i=0; i<_robot->GetDOF(); i++)
					_Teaching_Data_q(_pb_stepper-1, i) = _vp.vp_Rleave_q(i);
			}


			//// 보간 알고리즘
			for(int i = 0; i < _Teaching_Data_q.cols(); i++){
				_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
											_event_q_0(i), _Teaching_Data_q(_pb_stepper-1, i), 0.0, 0.0);
			}

			// 일단 joint space에서의 값만 체크하자. task space는 나중에..
			if(Is_reached(_Teaching_Data_q.row(_pb_stepper-1), _current_q, _resolution)){
				cout<<"Via point" << _pb_stepper <<"passed!!"<<endl;
				_pb_stepper++;
				_hash_track++;
				_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
				_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
				cout<<"event_time"<<_pb_stepper<<" : "<<_event_time_0<<endl;
			}
		}
		
		if ( _hash_track-1 == _vp.hash_numindx ){
				//_pb_stepper 초기화
				_pb_stepper = 0;
				_hash_track = 0;
				cout<<"Play back finished!!"<<endl;
					
				_bool_playback = false;
		}
	}
}

/////// Playback controller
void TeachingSW_HM::playback_controller(int& _pb_stepper, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home, VectorXd& _x_home, 
						VectorXd& _event_q_0, VectorXd& _event_x_0, VectorXd& _current_q, VectorXd& _current_x, MatrixXd _Teaching_Data, MatrixXd _Teaching_Data_q, 
						VectorXd& _Desired_x, VectorXd& _Desired_q, float _resolution, bool& _bool_playback)
{
	double via_time = _via_time;
	//static int _pb_stepper = 0;

	//_q_home.resize(_Teaching_Data.cols());
	//_current_q.resize(_Teaching_Data.cols());
	//_Desired_path.resize(_Teaching_Data.cols());

	if (_pb_stepper == 0)
	{
		//for (int i =0; i < _Teaching_Data.cols(); i++) {//for문을 티칭한 자유도만큼 돌리려면 컬럼을 보자.
		//	_Desired_x(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + (_pb_stepper+1) * via_time, _event_x_0(i), _x_home(i), 0.0, 0.0);
		//}
		for (int i =0; i < _Teaching_Data_q.cols(); i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + (_pb_stepper+1) * via_time, _event_q_0(i), _q_home(i), 0.0, 0.0);
		}
		//JointSpaceController(m_M, _gravity_torque, 400.0, 40.0, _q_sensed, _q_des, _qdot_sensed, _qdot_des, _torque_des);
		//cout<<"_event_time_0 : "<<_event_time_0<<" _end_time : "<<_event_time_0 + (_pb_stepper+1) * via_time<<
		//	" event_x_0 : "<<_event_x_0(4)<<" x_home(4) : "<<_x_home(4)<<" path : "<<_Desired_path(4)<<endl;			
		
		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		//if(Is_reached(_x_home, _current_x, _resolution) || Is_reached(_q_home, _current_q, _resolution)){
		if(Is_reached(_q_home, _current_q, _resolution)){
			cout<<"Get home to play back! Now start!"<<endl;
			_event_x_0 = _current_x;   // 플레이백 시작할 때의 초기 위치 저장
			_event_q_0 = _current_q;
			_event_time_0 = _time;        // 플레이백 시작할 때의 초기 시간 저장
			_pb_stepper++;
			cout<<"event_time"<<_pb_stepper<<" : "<<_event_time_0<<endl;
		}
	}
	else if (_pb_stepper >= 1)
	{
		//for(int i = 0; i < 3; i++){
			//_Desired_x(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, _event_x_0(i), _Teaching_Data(_pb_stepper-1, i), 0.0, 0.0);
			//_s1d(i) = _Teaching_Data(_pb_stepper-1, i+3);
			//_s2d(i) = _Teaching_Data(_pb_stepper-1, i+6);
			//_s3d(i) = _Teaching_Data(_pb_stepper-1, i+9);
		//}
		for(int i = 0; i < _Teaching_Data_q.cols(); i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
										_event_q_0(i), _Teaching_Data_q(_pb_stepper-1, i), 0.0, 0.0);
		}
		//GetPhi(T_wrist.R, _s1d, _s2d, _s3d, _phi);

		//for(int i=0; i<3; i++){
		//	_x_error(i) = _desired_x(i) - _x(i);
		//	_x_error(i+3) = -_phi(i);   //phi의 부호 때문에 position 에러를 위와 같이 했다는 것을 명심!!
		//	_Kp_task(i, i) = 400.0;
		//	_Kv_task(i, i) = 40.0;
		//	_Kp_task(i+3, i+3) = 400.0;
		//	_Kv_task(i+3, i+3) = 40.0;	
		//}
		//TaskSpaceController(7, 6, J_end, m_M, _gravity_torque, _Kp_task, _Kv_task, _x, _qdot_sensed, _x_error, _desired_xdot, _torque_des);

		//////////////////////////////////////
		////// joint control rather than task
		//for (int i =0; i < JOINTNUMBERS; i++)
		//	_q_des(i) = Cubic_path2(_time, event_time_0, event_time_0 + via_time,
		//								event_q_0(i), Q_teached_R(_pb_stepper-1, i), 0.0, 0.0);
		//JointSpaceController(m_M, _gravity_torque, 400.0, 40.0, _q_sensed, _q_des, _qdot_sensed, _qdot_des, _torque_des);

		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		//if(Is_reached(_Teaching_Data.row(_pb_stepper-1), _current_x, _resolution) || Is_reached(_Teaching_Data_q.row(_pb_stepper-1), _current_q, _resolution)){
		if(Is_reached(_Teaching_Data_q.row(_pb_stepper-1), _current_q, _resolution)){
			cout<<"Via point" << _pb_stepper <<"passed!!"<<endl;
			_pb_stepper++;			
			_event_x_0 = _current_x;
			_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
			_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
			cout<<"event_time"<<_pb_stepper<<" : "<<_event_time_0<<endl;

			if ( _pb_stepper-1 == _Teaching_Data_q.rows()){ // _pb_stepper-1 == _Teaching_Data.rows()
				//_pb_stepper 초기화
				_pb_stepper = 0;
				cout<<"Play back finished!!"<<endl;
					
				_bool_playback = false;

				//temp_x = _x;
				//_s1d_temp = T_wrist.R.column(0);
				//_s2d_temp = T_wrist.R.column(1);
				//_s3d_temp = T_wrist.R.column(2);
			}
		}
	}
}

/////// Playback controller
void TeachingSW_HM::playback_controller_q(int _jdof, int& _pb_stepper, double _via_time, double _time, double& _event_time_0, VectorXd& _q_home,  
						VectorXd& _event_q_0, VectorXd& _current_q, MatrixXd _Teaching_Data_q, VectorXd& _Desired_q, float _resolution, bool& _bool_playback)
{
	double via_time = _via_time;
	//static int _pb_stepper = 0;

	//size error check
	if(_q_home.size() != _jdof){
		cout<<"size of _q_home is wrong!"<<endl;
		return;
	}
	if(_event_q_0.size() != _jdof){
		cout<<"size of _event_q_0 is wrong!"<<endl;
		return;
	}
	if(_Teaching_Data_q.cols() != _jdof){
		cout<<"size of _Teaching_Data_q is wrong!"<<endl;
		return;
	}
	if(_current_q.size() != _jdof){
		cout<<"size of _current_q is wrong!"<<endl;
		return;
	}
	if(_Desired_q.size() != _jdof){
		cout<<"size of _Desired_q is wrong!"<<endl;
		return;
	}

	_Desired_q.resize(_jdof);

	if (_pb_stepper == 0)
	{
		for (int i =0; i < _jdof; i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + (_pb_stepper+1) * via_time, _event_q_0(i), _q_home(i), 0.0, 0.0);
		}
		//JointSpaceController(m_M, _gravity_torque, 400.0, 40.0, _q_sensed, _q_des, _qdot_sensed, _qdot_des, _torque_des);
		//cout<<"_event_time_0 : "<<_event_time_0<<" _end_time : "<<_event_time_0 + (_pb_stepper+1) * via_time<<
		//	" event_x_0 : "<<_event_x_0(4)<<" x_home(4) : "<<_x_home(4)<<" path : "<<_Desired_path(4)<<endl;			
		
		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		if(Is_reached(_q_home, _current_q, _resolution)){
			cout<<"Get home to play back! Now start!"<<endl;
			_event_q_0 = _current_q;
			_event_time_0 = _time;        // 플레이백 시작할 때의 초기 시간 저장
			_pb_stepper++;
			cout<<"event_time"<<_pb_stepper<<" : "<<_event_time_0<<endl;
		}
	}
	else if (_pb_stepper >= 1)
	{
		for(int i = 0; i < _Teaching_Data_q.cols(); i++){
			_Desired_q(i) = Cubic_path2(_time, _event_time_0, _event_time_0 + via_time, 
										_event_q_0(i), _Teaching_Data_q(_pb_stepper-1, i), 0.0, 0.0);
		}

		// 일단 joint space에서의 값만 체크하자. task space는 나중에..
		if(Is_reached(_Teaching_Data_q.row(_pb_stepper-1), _current_q, _resolution)){
			cout<<"Via point" << _pb_stepper <<"passed!!"<<endl;
			_pb_stepper++;			
			_event_q_0 = _current_q;	// via 포인트 지나갈 다시 경로 계획을 위한 위치 저장
			_event_time_0 = _time;       // via 포인트 지나갈 다시 경로 계획을 위한 시간 저장
			cout<<"event_time"<<_pb_stepper<<" : "<<_event_time_0<<endl;

			if ( _pb_stepper-1 == _Teaching_Data_q.rows() ){
				//_pb_stepper 초기화
				_pb_stepper = 0;
				cout<<"Play back finished!!"<<endl;
					
				_bool_playback = false;
			}
		}
	}
}


void TeachingSW_HM::IKLoop_grasp(T_Robot *_robot, Matrix4d &_grasp, double pos_res, double ori_res, VectorXd& _approachG_q, VectorXd& _reachG_q, VectorXd& _leaveG_q)
{
	bool bIKfin = false;
	Robot robot_dummy;
	robot_dummy = *_robot;

	_approachG_q.resize(_robot->GetDOF());
	_reachG_q.resize(_robot->GetDOF());
	_leaveG_q.resize(_robot->GetDOF());

	Matrix4d _approachG;
	_approachG = _grasp;
	_approachG(2, 3) = _grasp(2, 3) + 30.0;
	
	while(!bIKfin){
		double pos_error = (_approachG.col(3).segment(0,3) - robot_dummy.GetTool().col(3).segment(0,3)).norm();
		Vector3d ori_error;
		ori_error(0) = 1.0 - _approachG.col(0).segment(0,3).dot(robot_dummy.GetTool().col(0).segment(0,3));
		ori_error(1) = 1.0 - _approachG.col(1).segment(0,3).dot(robot_dummy.GetTool().col(1).segment(0,3));
		ori_error(2) = 1.0 - _approachG.col(2).segment(0,3).dot(robot_dummy.GetTool().col(2).segment(0,3));
		VectorXd IKdq(robot_dummy.GetDOF());
		IKdq.setZero();
		IKdq = IKLoop(&robot_dummy, _approachG, pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);
		for(int i=0; i<robot_dummy.GetDOF(); i++)
			robot_dummy.SetQ(i, robot_dummy.GetQ(i) + IKdq(i)); 
		if(bIKfin == true){
			//// approach 변수에 via point 저장.. 우선 joint space 값만..
			_approachG_q = robot_dummy.GetQ(); /// IK 다 풀고난 값을 목적지로 적용
			_leaveG_q = _approachG_q;
			cout<<"IK for vision data _approachG_q to grip finished"<<_approachG_q<<endl;
			cout<<"**********************"<<endl;
		}
	}

	while(!bIKfin){
		double pos_error = (_grasp.col(3).segment(0,3) - robot_dummy.GetTool().col(3).segment(0,3)).norm();
		Vector3d ori_error;
		ori_error(0) = 1.0 - _grasp.col(0).segment(0,3).dot(robot_dummy.GetTool().col(0).segment(0,3));
		ori_error(1) = 1.0 - _grasp.col(1).segment(0,3).dot(robot_dummy.GetTool().col(1).segment(0,3));
		ori_error(2) = 1.0 - _grasp.col(2).segment(0,3).dot(robot_dummy.GetTool().col(2).segment(0,3));
		VectorXd IKdq(robot_dummy.GetDOF());
		IKdq.setZero();
		IKdq = IKLoop(&robot_dummy, _grasp, pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);
		for(int i=0; i<robot_dummy.GetDOF(); i++)
			robot_dummy.SetQ(i, robot_dummy.GetQ(i) + IKdq(i)); 
		if(bIKfin == true){
			//// approach 변수에 via point 저장.. 우선 joint space 값만..
			_reachG_q = robot_dummy.GetQ(); /// IK 다 풀고난 값을 목적지로 적용
			cout<<"IK for vision data _reachG_q to grip finished"<<_reachG_q<<endl;
			cout<<"**********************"<<endl;
		}
	}
}

void TeachingSW_HM::IKLoop_release(T_Robot *_robot, Matrix4d &_release, double pos_res, double ori_res, VectorXd& _approachR_q, VectorXd& _reachR_q, VectorXd& _leaveR_q)
{
	bool bIKfin = false;
	Robot robot_dummy;
	robot_dummy = *_robot;

	_approachR_q.resize(_robot->GetDOF());
	_reachR_q.resize(_robot->GetDOF());
	_leaveR_q.resize(_robot->GetDOF());

	Matrix4d _approachR;
	_approachR = _release;
	_approachR(2, 3) = _release(2, 3) + 30.0;
	
	while(!bIKfin){
		double pos_error = (_approachR.col(3).segment(0,3) - robot_dummy.GetTool().col(3).segment(0,3)).norm();
		Vector3d ori_error;
		ori_error(0) = 1.0 - _approachR.col(0).segment(0,3).dot(robot_dummy.GetTool().col(0).segment(0,3));
		ori_error(1) = 1.0 - _approachR.col(1).segment(0,3).dot(robot_dummy.GetTool().col(1).segment(0,3));
		ori_error(2) = 1.0 - _approachR.col(2).segment(0,3).dot(robot_dummy.GetTool().col(2).segment(0,3));
		VectorXd IKdq(robot_dummy.GetDOF());
		IKdq.setZero();
		IKdq = IKLoop(&robot_dummy, _approachR, pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);
		for(int i=0; i<robot_dummy.GetDOF(); i++)
			robot_dummy.SetQ(i, robot_dummy.GetQ(i) + IKdq(i)); 
		if(bIKfin == true){
			//// approach 변수에 via point 저장.. 우선 joint space 값만..
			_approachR_q = robot_dummy.GetQ(); /// IK 다 풀고난 값을 목적지로 적용
			_leaveR_q = _approachR_q;
			cout<<"IK for vision data _approachR_q to release finished"<<_approachR_q<<endl;
			cout<<"**********************"<<endl;
		}
	}

	while(!bIKfin){
		double pos_error = (_release.col(3).segment(0,3) - robot_dummy.GetTool().col(3).segment(0,3)).norm();
		Vector3d ori_error;
		ori_error(0) = 1.0 - _release.col(0).segment(0,3).dot(robot_dummy.GetTool().col(0).segment(0,3));
		ori_error(1) = 1.0 - _release.col(1).segment(0,3).dot(robot_dummy.GetTool().col(1).segment(0,3));
		ori_error(2) = 1.0 - _release.col(2).segment(0,3).dot(robot_dummy.GetTool().col(2).segment(0,3));
		VectorXd IKdq(robot_dummy.GetDOF());
		IKdq.setZero();
		IKdq = IKLoop(&robot_dummy, _release, pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);
		for(int i=0; i<robot_dummy.GetDOF(); i++)
			robot_dummy.SetQ(i, robot_dummy.GetQ(i) + IKdq(i)); 
		if(bIKfin == true){
			//// approach 변수에 via point 저장.. 우선 joint space 값만..
			_reachR_q = robot_dummy.GetQ(); /// IK 다 풀고난 값을 목적지로 적용
			cout<<"IK for vision data _reachR_q to release finished"<<_reachR_q<<endl;
			cout<<"**********************"<<endl;
		}
	}
}

///////////////////////////////////
/////////////// Robotics Algorithm
VectorXd TeachingSW_HM::IKLoop(Robot *_Robot, Matrix4d &_goal_T, double pos_res, double ori_res, double pos_error, double ori_error, bool &_bfinished)
{
	VectorXd dq;
	dq.resize(_Robot->GetDOF());
	dq.setZero();

	if( pos_error > pos_res/*1.0f*//*0.2f*/ 
		|| ori_error > ori_res/*1e-2*//*1e-10*/	)
	{
		_bfinished = false;

		_Robot->CalcFK();
		_Robot->CalcJacobian();

		// IK 계산, damping parameter 설정
		JacobiSVD<MatrixXd> svd(_Robot->GetJacobian(), ComputeThinU | ComputeThinV);
		double minval_singular = svd.singularValues()(0); // 기본적인 min값 소팅 방법..
		for (int i = 1; i < svd.singularValues().size(); i++)
		{
			if (svd.singularValues()(i) < minval_singular) {
				minval_singular = svd.singularValues()(i);
			}
		}
		double damp = 0.0, singular_region = 0.2, max_damp = abs(sqrt(0.15)); // singular region 와 max_damp 어떻게 설정할 지 잘 생각해보자.
		if(minval_singular >= singular_region) // min sigular val이 singular region보다 크거나 같으면 damping 0 
			damp =0.0;
		else
			damp =(1 - (minval_singular/singular_region)*(minval_singular/singular_region)) * max_damp * max_damp;///*pos_error;//*/1.0*(_goal_T.col(3).segment(0,3) - _Amiro.GetTool().col(3).segment(0,3)).norm();

		//// Desired position과 Current position사이의 Error값
		VectorXd err(3);
		err = _goal_T.col(3).segment(0,3) - _Robot->GetTool().col(3).segment(0,3);	// x, y, z값에 대한 것만

		VectorXd _dx(6);

		// orientation (direction cosine) direction cosine vector (x, y, z축별)
		Vector3d vRxc, vRyc, vRzc,    vRxd, vRyd, vRzd;	// c: current, d: desired
		vRxc = _Robot->GetTool().col(0).segment(0, 3),	vRyc = _Robot->GetTool().col(1).segment(0, 3),	vRzc = _Robot->GetTool().col(2).segment(0, 3);	// 현재 로봇의 direction cosine
		vRxd = _goal_T.col(0).segment(0, 3),			vRyd = _goal_T.col(1).segment(0, 3),			vRzd = _goal_T.col(2).segment(0, 3);	// goal_T 의 direction cosine
			
		_dx(0) = err(0);
		_dx(1) = err(1);
		_dx(2) = err(2);
	
		Vector3d ori;
		ori = 0.5*(vRxc.cross(vRxd) + vRyc.cross(vRyd) + vRzc.cross(vRzd));

		_dx(3) = ori(0);
		_dx(4) = ori(1);
		_dx(5) = ori(2);
		
		MatrixXd _mat_Identity(6, 6);
		_mat_Identity.setIdentity();
	
		//if(_damp_param < 1) _damp_param = 1;
		// dq = J'inv(J*J'+damp*I)*dx
		dq.resize(_Robot->GetDOF());
	
		// TODO -------------------------------------------------
		//ColPivHouseholderQR<MatrixXd> qr(GetJacobian());
		//_dq = qr.solve(_dx);
		// TODO -------------------------------------------------
	
		dq = _Robot->GetJacobian().transpose() * (_Robot->GetJacobian() * _Robot->GetJacobian().transpose() + damp*damp*_mat_Identity).inverse() * _dx;

		//double max_dq = 5 * H_DtoR; //upper bound on how far we attempt to move joints in a single update step
		//for(int i=0; i<_Robot->GetDOF(); i++)
		//	if (dq(i) > max_dq){
		//		cout<<"dq '" <<i<<" ' exceeds"<<dq * H_RtoD<<endl;
		//		dq = max_dq * dq/dq.norm();
		//	}
	}
	else
		_bfinished = true; // IK가 끝났는 지 보여주기 위해.

	return dq;
}

VectorXd TeachingSW_HM::IKLoop_selective(Robot *_Robot, Matrix4d &_goal_T, double pos_res, double ori_res, double pos_error, double ori_error, MatrixXd _selectionMatrix, bool &_bfinished)
{
	VectorXd dq;
	dq.resize(_Robot->GetDOF());
	dq.setZero();

	if (_selectionMatrix.size() != 36){
		_selectionMatrix.resize(6, 6);
		_selectionMatrix.setIdentity();
	}

	if( pos_error > pos_res/*1.0f*//*0.2f*/ 
		|| ori_error > ori_res/*1e-2*//*1e-10*/	)
	{
		_bfinished = false;

		_Robot->CalcFK();
		_Robot->CalcJacobian();

		// IK 계산, damping parameter 설정
		JacobiSVD<MatrixXd> svd(_Robot->GetJacobian(), ComputeThinU | ComputeThinV);
		double minval_singular = svd.singularValues()(0); // 기본적인 min값 소팅 방법..
		for (int i = 1; i < svd.singularValues().size(); i++)
		{
			if (svd.singularValues()(i) < minval_singular) {
				minval_singular = svd.singularValues()(i);
			}
		}
		double damp = 0.0, singular_region = 0.2, max_damp = abs(sqrt(0.15)); // singular region 와 max_damp 어떻게 설정할 지 잘 생각해보자.
		if(minval_singular >= singular_region) // min sigular val이 singular region보다 크거나 같으면 damping 0 
			damp =0.0;
		else
			damp =(1 - (minval_singular/singular_region)*(minval_singular/singular_region)) * max_damp * max_damp;///*pos_error;//*/1.0*(_goal_T.col(3).segment(0,3) - _Amiro.GetTool().col(3).segment(0,3)).norm();

		//// Desired position과 Current position사이의 Error값
		VectorXd err(3);
		err = _goal_T.col(3).segment(0,3) - _Robot->GetTool().col(3).segment(0,3);	// x, y, z값에 대한 것만

		VectorXd _dx(6);

		// orientation (direction cosine) direction cosine vector (x, y, z축별)
		Vector3d vRxc, vRyc, vRzc,    vRxd, vRyd, vRzd;	// c: current, d: desired
		vRxc = _Robot->GetTool().col(0).segment(0, 3),	vRyc = _Robot->GetTool().col(1).segment(0, 3),	vRzc = _Robot->GetTool().col(2).segment(0, 3);	// 현재 로봇의 direction cosine
		vRxd = _goal_T.col(0).segment(0, 3),			vRyd = _goal_T.col(1).segment(0, 3),			vRzd = _goal_T.col(2).segment(0, 3);	// goal_T 의 direction cosine
			
		_dx(0) = err(0);
		_dx(1) = err(1);
		_dx(2) = err(2);
	
		Vector3d ori;
		ori = 0.5*(vRxc.cross(vRxd) + vRyc.cross(vRyd) + vRzc.cross(vRzd));

		_dx(3) = ori(0);
		_dx(4) = ori(1);
		_dx(5) = ori(2);
		
		MatrixXd _mat_Identity(6, 6);
		_mat_Identity.setIdentity();
	
		//if(_damp_param < 1) _damp_param = 1;
		// dq = J'inv(J*J'+damp*I)*dx
		dq.resize(_Robot->GetDOF());
	
		// TODO -------------------------------------------------
		//ColPivHouseholderQR<MatrixXd> qr(GetJacobian());
		//_dq = qr.solve(_dx);
		// TODO -------------------------------------------------
	
		dq = _Robot->GetJacobian().transpose() * (_Robot->GetJacobian() * _Robot->GetJacobian().transpose() + damp*damp*_mat_Identity).inverse() * _selectionMatrix * _dx;

		//double max_dq = 5 * H_DtoR; //upper bound on how far we attempt to move joints in a single update step
		//for(int i=0; i<_Robot->GetDOF(); i++)
		//	if (dq(i) > max_dq){
		//		cout<<"dq '" <<i<<" ' exceeds"<<dq * H_RtoD<<endl;
		//		dq = max_dq * dq/dq.norm();
		//	}
	}
	else
		_bfinished = true; // IK가 끝났는 지 보여주기 위해.

	return dq;
}

VectorXd TeachingSW_HM::IKLoop_with_Null(Robot *_Robot, Matrix4d &_goal_T, VectorXd _Midjoint, double pos_res, double ori_res, double pos_error, double ori_error)
{
	VectorXd dq;
	dq.resize(_Robot->GetDOF());
	dq.setZero();

	if( pos_error > pos_res/*1.0f*//*0.2f*/ 
		|| ori_error > ori_res/*1e-2*//*1e-10*/	)
	{
		_Robot->CalcFK();
		_Robot->CalcJacobian();

		// IK 계산, damping parameter 설정
		JacobiSVD<MatrixXd> svd(_Robot->GetJacobian(), ComputeThinU | ComputeThinV);
		double minval_singular = svd.singularValues()(0); // 기본적인 min값 소팅 방법..
		for (int i = 1; i < svd.singularValues().size(); i++)
		{
			if (svd.singularValues()(i) < minval_singular) {
				minval_singular = svd.singularValues()(i);
			}
		}
		double damp = 0.0, singular_region = 0.2, max_damp = abs(sqrt(0.15));
		
		if(minval_singular < singular_region) 
			damp =(1 - (minval_singular/singular_region)*(minval_singular/singular_region)) * max_damp * max_damp;///*pos_error;//*/1.0*(_goal_T.col(3).segment(0,3) - _Amiro.GetTool().col(3).segment(0,3)).norm();
		if(damp > max_damp) // min sigular val이 singular region보다 작거나 같으면 damping 0 
			damp = 0.0;

		_Robot->CalcIK(_goal_T, dq, damp);

		// null space velocity 더해주기
		MatrixXd _Ident;
		_Ident.setIdentity(_Robot->GetDOF(), _Robot->GetDOF());
		//ColPivHouseholderQR<MatrixXd> _qr(_Robot->GetJacobian());
		MatrixXd _pseudo_inv_J;
		_pseudo_inv_J.resize(_Robot->GetJacobian().cols(), _Robot->GetJacobian().rows());
		_pseudo_inv_J = _Robot->GetJacobian().transpose() * (_Robot->GetJacobian() * _Robot->GetJacobian().transpose() + damp*damp*_Ident).inverse();

		// Null Velocity 정의
		VectorXd Nullvel(_Robot->GetDOF());
		Nullvel.setZero();

		VectorXd weight(_Robot->GetDOF());
		weight.setConstant(0.1);
		Nullvel = AvoidJointlimitCtrl(_Robot, _Midjoint, weight);

		// Null Velocity 적용
		dq += (_Ident - _pseudo_inv_J * _Robot->GetJacobian())*Nullvel;
                             

		// 로봇 update(q값 갱신) // 자체 시뮬레이터에서만 필요한 듯.
		//VectorXd _q = _Amiro.GetQ();
		//_Amiro.SetQ(_Amiro.LPF1(0.99, _q+dq, _q));
		
		//cout<<"dq!!"<<dq * H_RtoD<<endl;
		//cout<<"current Q!!"<<_Amiro.GetQ() * H_RtoD<<endl;
		
		///*	Robot과 교신할 때에는 로봇에 지령치를 주어야 함 */	
		//_SHM->traj.joint_traj[0].duration = duration;
		//for(int i=0; i<16; i++){
		//	if(i < _Amiro.GetDOF() )
		//		_SHM->traj.joint_traj[0].joint_rad[i] = _Amiro.LPF1(0.7, _Amiro.GetQ()+dq, _Amiro.GetQ())(i) * H_RtoD;// Amiro 값 갱신 (오른팔)
		//	else
		//		_SHM->traj.joint_traj[0].joint_rad[i] = 0.0 * H_RtoD;// Amiro 값 갱신 (오른팔)
		//}

		///*	Robot과 교신할 때에는 아래의 코드로 로봇의 값으로 갱신해야 함 */	
		//for(int i=0; i<_Amiro.GetDOF(); i++)
		//	_Amiro.SetQ(i, _SHM->state.slave_state[i].position * H_DtoR); // Amiro 값을 받아와 갱신 (오른팔)
	}
	//else{
	//	_SHM->traj.joint_traj[0].duration = 0.001;
	//	for(int i=0; i<16; i++)
	//		_SHM->traj.joint_traj[0].joint_rad[i] = _SHM->state.slave_state[i].position;
	//}
	return dq;
}

VectorXd TeachingSW_HM::IKLoop_dualArm_with_torso(Robot *_AmiroR, Robot *_AmiroL, Matrix4d &_goal_TR, Matrix4d &_goal_TL, double pos_res, double ori_res, double pos_error, double ori_error, bool &_bfinished)
{
	VectorXd dq_total;
	dq_total.resize(16);
	dq_total.setZero();
	
	if( pos_error > pos_res/*1.0f*//*0.2f*/ 
		|| ori_error > ori_res/*1e-2*//*1e-10*/	)
	{
		_bfinished = false;

		// Amiro 상태 업데이트                                                                  
		_AmiroR->CalcFK();
		_AmiroR->CalcJacobian();
		_AmiroL->CalcFK();
		_AmiroL->CalcJacobian();

		MatrixXd _Jacobian_total(12, 16); // torso-rightArm-leftArm 순서로.
		for(int i=0; i<6; i++){
			for(int j=0; j<2; j++){
				_Jacobian_total(i, j) = _AmiroR->GetJacobian()(i, j);
				_Jacobian_total(i+6, j) = _AmiroL->GetJacobian()(i, j);
			}
		}//torso 
		for(int i=0; i<6; i++){
			for(int j=0; j<7; j++){
				//RightArm
				_Jacobian_total(i, j+2) = _AmiroR->GetJacobian()(i, j+2);
				_Jacobian_total(i, j+9) = 0.0;
				//LeftArm
				_Jacobian_total(i+6, j+2) = 0.0;
				_Jacobian_total(i+6, j+9) = _AmiroL->GetJacobian()(i, j+2);
			}
		}

		// Amiro IK 계산
		// IK 계산, damping parameter 설정
		JacobiSVD<MatrixXd> svd(_Jacobian_total, ComputeThinU | ComputeThinV);
		double minval_singular = svd.singularValues()(0); // 기본적인 min값 소팅 방법..
		for (int i = 1; i < svd.singularValues().size(); i++)
		{
			if (svd.singularValues()(i) < minval_singular) {
				minval_singular = svd.singularValues()(i);
			}
		}
		double damp = 0.0, singular_region = 0.2, max_damp = abs(sqrt(0.15)); // singular region 와 max_damp 어떻게 설정할 지 잘 생각해보자.
		
		if(minval_singular >= singular_region) // min sigular val이 singular region보다 크거나 같으면 damping 0 
			damp =0.0;
		else
			damp =(1 - (minval_singular/singular_region)*(minval_singular/singular_region)) * max_damp * max_damp;///*pos_error;//*/1.0*(_goal_T.col(3).segment(0,3) - _Amiro.GetTool().col(3).segment(0,3)).norm();

		
		//// Desired position과 Current position사이의 Error값
		VectorXd _dx_total(12);
		//// Right Arm
		VectorXd errR(3);
		errR = _goal_TR.col(3).segment(0,3) - _AmiroR->GetTool().col(3).segment(0,3);	// x, y, z값에 대한 것만
		
		// orientation (direction cosine) direction cosine vector (x, y, z축별)
		Vector3d vRxcR, vRycR, vRzcR,    vRxdR, vRydR, vRzdR;	// c: current, d: desired
		vRxcR = _AmiroR->GetTool().col(0).segment(0, 3),		vRycR = _AmiroR->GetTool().col(1).segment(0, 3),		vRzcR = _AmiroR->GetTool().col(2).segment(0, 3);	// 현재 로봇의 direction cosine
		vRxdR = _goal_TR.col(0).segment(0, 3),				vRydR = _goal_TR.col(1).segment(0, 3),				vRzdR = _goal_TR.col(2).segment(0, 3);	// goal_T 의 direction cosine
		
		Vector3d ori_errR;
		ori_errR = 0.5*(vRxcR.cross(vRxdR) + vRycR.cross(vRydR) + vRzcR.cross(vRzdR));

		// calculate qdot
		static VectorXd pre_q_R(_AmiroR->GetDOF());
		static bool flag_qdotR = true;
		if(flag_qdotR){
			pre_q_R.setZero();
			flag_qdotR = false;
		}
		VectorXd _qdot_R(_AmiroR->GetDOF());
		_qdot_R = _AmiroR->GetQ() - pre_q_R;
		pre_q_R = _AmiroR->GetQ();
		
		VectorXd _xdot_R(6);
		_xdot_R = _AmiroR->GetJacobian() * _qdot_R;
		
		_dx_total(0) = errR(0);
		_dx_total(1) = errR(1);
		_dx_total(2) = errR(2);
		_dx_total(3) = ori_errR(0);
		_dx_total(4) = ori_errR(1);
		_dx_total(5) = ori_errR(2);	

		
		//// Left Arm
		VectorXd errL(3);
		errL = _goal_TL.col(3).segment(0,3) - _AmiroL->GetTool().col(3).segment(0,3);	// x, y, z값에 대한 것만
		// orientation (direction cosine) direction cosine vector (x, y, z축별)
		Vector3d vRxcL, vRycL, vRzcL,    vRxdL, vRydL, vRzdL;	// c: current, d: desired
		vRxcL = _AmiroL->GetTool().col(0).segment(0, 3),		vRycL = _AmiroL->GetTool().col(1).segment(0, 3),		vRzcL = _AmiroL->GetTool().col(2).segment(0, 3);	// 현재 로봇의 direction cosine
		vRxdL = _goal_TL.col(0).segment(0, 3),				vRydL = _goal_TL.col(1).segment(0, 3),				vRzdL = _goal_TL.col(2).segment(0, 3);	// goal_T 의 direction cosine

		Vector3d ori_errL;
		ori_errL = 0.5*(vRxcL.cross(vRxdL) + vRycL.cross(vRydL) + vRzcL.cross(vRzdL));

		_dx_total(6) = errL(0);
		_dx_total(7) = errL(1);
		_dx_total(8) = errL(2);
		_dx_total(9) = ori_errL(0);
		_dx_total(10) = ori_errL(1);
		_dx_total(11) = ori_errL(2);		

		MatrixXd _mat_Identity(12, 12);
		_mat_Identity.setIdentity();

		dq_total = _Jacobian_total.transpose() * (_Jacobian_total * _Jacobian_total.transpose() + damp * damp * _mat_Identity).inverse() * _dx_total;

		double max_dq = 40.0 * H_DtoR; //upper bound on how far we attempt to move joints in a single update step
		if (dq_total.norm() > max_dq){
			cout<<"dq exceeds"<<dq_total * H_RtoD<<endl;
			dq_total = max_dq * dq_total/dq_total.norm();
		}

		////////////      null space velocity 더해주기       //////////////////////
		MatrixXd _Ident;
		_Ident.setIdentity(_Jacobian_total.rows(), _Jacobian_total.rows());
		//ColPivHouseholderQR<MatrixXd> _qr(_Robot.GetJacobian());
		MatrixXd _pseudo_inv_J_total;
		_pseudo_inv_J_total.resize(_Jacobian_total.cols(), _Jacobian_total.rows());
		_pseudo_inv_J_total = _Jacobian_total.transpose() * (_Jacobian_total * _Jacobian_total.transpose() + 0.0*damp*damp*_Ident).inverse();

		/* Joint limit control을 위한 설정 */
		VectorXd MidjointR(_AmiroR->GetDOF());
		VectorXd MidjointL(_AmiroL->GetDOF());
		MidjointR(0) = 0.0 * H_DtoR;
		MidjointR(1) = 0.0 * H_DtoR;
		MidjointR(2) = 0.0 * H_DtoR;
		MidjointR(3) = -50.0 * H_DtoR;
		MidjointR(4) = 0.0 * H_DtoR;
		MidjointR(5) = 30.0 * H_DtoR;
		MidjointR(6) = 0.0 * H_DtoR;
		MidjointR(7) = 0.0 * H_DtoR;
		MidjointR(8) = -5.0 * H_DtoR;
		MidjointL = -1.0 * MidjointR;
		// Null Velocity 정의
		VectorXd Nullvel(16);
		Nullvel.setZero();
		for(int i=0; i<2; i++)
			Nullvel(i) = 10.0/*_weight*/ * (MidjointR(i) - _AmiroR->GetQ()(i));
		for(int i=0; i<7; i++){
			Nullvel(i+2) = 10.0/*_weight*/ * (MidjointR(i+2) - _AmiroR->GetQ()(i+2));
			Nullvel(i+9) = 10.0/*_weight*/ * (MidjointL(i+2) - _AmiroL->GetQ()(i+2));
		}

		// Null Velocity 적용
		MatrixXd _Ident2;
		_Ident2.setIdentity(16, 16);
		//dq_total = dq_total + (_Ident2 - _pseudo_inv_J_total * _Jacobian_total)*Nullvel;
		
		//cout<<"null dq"<<_Jacobian_total*(_Ident2 - _pseudo_inv_J_total * _Jacobian_total)*Nullvel<<endl;
		//max_dq = 40.0 * H_DtoR; //upper bound on how far we attempt to move joints in a single update step
		//if (dq_total.norm() > max_dq){
		//	cout<<"dq exceeds"<<dq_total * H_RtoD<<endl;
		//	dq_total = max_dq * dq_total/dq_total.norm();
		//}
	}
	else
		_bfinished = true; // IK가 끝났는 지 보여주기 위해.

	return dq_total;
}

MatrixXd TeachingSW_HM::Skew(Vector3d _Skew)
{
	MatrixXd		_Skew_Symmetric(3, 3);

	_Skew_Symmetric.setZero();
	_Skew_Symmetric(0, 1) = - (_Skew(2));
	_Skew_Symmetric(0, 2) = _Skew(1);
	_Skew_Symmetric(1, 0) = _Skew(2);
	_Skew_Symmetric(1, 2) = - (_Skew(0));
	_Skew_Symmetric(2, 0) = - (_Skew(1));
	_Skew_Symmetric(2, 1) = _Skew(0);

	return(_Skew_Symmetric);
}

void TeachingSW_HM::GetPhi(Matrix3d RotationMtx, Vector3d s1d, Vector3d s2d, Vector3d s3d, Vector3d& phi)
{
	// Get SkewSymmetric
	MatrixXd s1_skew(3, 3);
	MatrixXd s2_skew(3, 3);
	MatrixXd s3_skew(3, 3);
	
	s1_skew = Skew(RotationMtx.col(0));
	s2_skew = Skew(RotationMtx.col(1));
	s3_skew = Skew(RotationMtx.col(2));
	/////////////////////////////////////////////////////////////

	VectorXd s1f(3);
	VectorXd s2f(3);
	VectorXd s3f(3);

	s1f = s1_skew * s1d;
	s2f = s2_skew * s2d;
	s3f = s3_skew * s3d;
	/////////////////////////////////////////////////////////////
	phi = (s1f + s2f + s3f) * (-1.0/2.0);
}

double TeachingSW_HM::Cubic_path2(double rT, double rT_0, double rT_f, double rTheta_0, double rTheta_f,
												 double rTheta_dot_0, double rTheta_dot_f)
{
	MatrixXd	timedata = Matrix<double, 4, 4>::Identity();

	timedata(0, 1) = rT_0;
	timedata(0, 2) = rT_0*rT_0;
	timedata(0, 3) = rT_0*rT_0*rT_0;
	timedata(1, 2) = 2*rT_0;
	timedata(1, 3) = 3*rT_0*rT_0;
	timedata(2, 1) = 1.0;
	timedata(2, 2) = 2*rT_f;
	timedata(2, 3) = 3*rT_f*rT_f;
	timedata(3, 0) = 1.0;
	timedata(3, 1) = rT_f;
	timedata(3, 2) = rT_f*rT_f;
	timedata(3, 3) = rT_f*rT_f*rT_f;

	MatrixXd timedata_inv;
	timedata_inv.resize(4, 4);
	timedata_inv = timedata.inverse();

	VectorXd	coefficient;
	coefficient.resize(4);
	VectorXd	constant;
	constant.resize(4);
	constant(0) = rTheta_0;
	constant(1) = rTheta_dot_0;
	constant(2) = rTheta_dot_f;
	constant(3) = rTheta_f;

	coefficient = timedata_inv * constant;

	double rTheta_t;

	rTheta_t = coefficient(0) + coefficient(1)*rT + coefficient(2)*rT*rT + coefficient(3)*rT*rT*rT;

	if(rT > rT_f)
		rTheta_t = rTheta_f;

	return(rTheta_t);
}

bool TeachingSW_HM::Is_reached(VectorXd destination, VectorXd _current, float threshold)
{
	if((destination - _current).norm() < threshold)
		return true;
	else
		return false;
}

void TeachingSW_HM::JointSpaceController(MatrixXd inertia, VectorXd gravity_torque, float Kp, float Kv,
						  VectorXd _q, VectorXd desired_q, VectorXd _qdot, VectorXd desired_qdot, VectorXd& torque)
{
	torque = inertia*(Kp*(desired_q - _q) + Kv*(desired_qdot - _qdot)) + gravity_torque - 40.0*inertia*_qdot;
}

void TeachingSW_HM::TaskSpaceController(int jointDOF, int taskDOF, MatrixXd task_jacobian, MatrixXd inertia, VectorXd gravity_torque, 
		                  MatrixXd Kp, MatrixXd Kv, VectorXd _x, VectorXd _qdot, VectorXd _x_error, VectorXd desired_xdot, VectorXd& torque)
{
	MatrixXd inertia_inv(jointDOF, jointDOF);
	MatrixXd task_inertia_temp(taskDOF, taskDOF);
	MatrixXd task_inertia(taskDOF, taskDOF);
	MatrixXd task_jacobian_trans(jointDOF, taskDOF);

	inertia_inv = inertia.inverse(); // pseudo 로 할 필요가 있을까? 아닐듯 항상 positive definite 아닌가?	
	task_jacobian_trans = task_jacobian.transpose();
	task_inertia_temp = task_jacobian*inertia_inv*task_jacobian_trans;
	task_inertia = task_inertia_temp.inverse(); //그럼 이건 pseudo로 할 필요가 있나? 아무래도 jacobian이 rank 감소할 떄도 있고.
		
	VectorXd xdot(taskDOF);
	xdot = task_jacobian * _qdot;
	
	torque = task_jacobian_trans*task_inertia*(Kp*_x_error + Kv*(desired_xdot - xdot)) + gravity_torque - 40.0*inertia*_qdot;
}

VectorXd TeachingSW_HM::AvoidJointlimitCtrl(Robot *_Robot, VectorXd _Midjoint, VectorXd _weight)
{
	VectorXd _Nullvel(_Robot->GetDOF());
	
	if(_Midjoint.size() != _Robot->GetDOF()){
		cout<<"you put the wrong Midjoint size."<<endl;
		return _weight.setZero();
	}

	if(_weight.size() != _Robot->GetDOF()){
		cout<<"you put the wrong weight size."<<endl;
		return _weight.setZero();
	}
	
	/* weight factor 설정 */
	MatrixXd weight_factor(_Robot->GetDOF(), _Robot->GetDOF());
	weight_factor.setIdentity(); // 일단 weighting factor는 identity로 한다. 
	                             // 이 값의 설정이 몹시 중요한 것처럼 논문에 나와있음. 아마 이걸 intelligent하게 튜닝하는 법이 있지 않을까?

	for(int i=0; i<_Robot->GetDOF(); i++)
		weight_factor(i, i) = _weight(i);
	
	// 이런 방식의 튜닝은 어떨까? 각 차이가 너무 크다 싶으면 weight를 줄이는 것이지..
	if ( (_Robot->GetQ() - _Midjoint).norm() > 0.1 ) // rad 사용
		weight_factor = 0.1 * weight_factor;

	/* getting gradient */
	F<double> cost_midjoint;
	F<double> *_q, *_midq;
	_q = new F<double>[_Robot->GetDOF()];
	_midq = new F<double>[_Robot->GetDOF()];

	for(int i=0; i<_Robot->GetDOF(); i++)
		_q[i].diff(i, _Robot->GetDOF());	//q를 i번째 미분변수로 지정
	for(int i=0; i<_Robot->GetDOF(); i++)
		_q[i] = _Robot->GetQ(i);			//q의 numeric value 입력
	for(int i=0; i<_Robot->GetDOF(); i++)
		_midq[i] = _Midjoint(i);	//midq의 numeric value 입력

	cost_midjoint = 0.5 * (_Robot->GetQ() - _Midjoint).transpose() * weight_factor * (_Robot->GetQ() - _Midjoint);
	//위 식은 numerical 변수들로 표현이 안되니.. 일일이 풀어서 써줘어 하나?
	//일단 다양한 검색을 해 보았으나, eigen과 완벽히 혼용되는 AD(automatic differentiation) tool이 잘 없는 것 같다.
	//https://justindomke.wordpress.com/2009/02/17/automatic-differentiation-the-most-criminally-underused-tool-in-the-potential-machine-learning-toolbox/


	//일단, joint limit avoidance는 굳이 gradient를 구하지 않아도 되는 것 같다. (원래는 위의 cost_midjoint를 grad해야할 것 같지만..
	//desired value를 midjoint로 갖는 P control을 한다고 생각하고 구현하는 논문이 많은듯.
	//그래서 gain tuning이 중요해 지는 것으로 보인다.
	_Nullvel = weight_factor * (_Midjoint - _Robot->GetQ());

	delete _q;
	delete _midq;

	return _Nullvel;
}



F<double> TeachingSW_HM::GetManipulability(Robot *_Robot)
{
	MatrixXd _rJacobian_trans(_Robot->GetJacobian().cols(), _Robot->GetJacobian().rows());
	MatrixXd _rJacobian_square(_Robot->GetJacobian().rows(), _Robot->GetJacobian().rows());
	MatrixXd _rJacobian_square_inv(_Robot->GetJacobian().rows(), _Robot->GetJacobian().rows());
	_rJacobian_trans = _Robot->GetJacobian().transpose();
	_rJacobian_square = _Robot->GetJacobian() * _rJacobian_trans;
	
	double _determinant = 0.0;
	double _manipulability = 0.0;

	_rJacobian_square_inv = _rJacobian_square.inverse();
	
	//// 그런데 jacobian에 determinant가 없을 경우에는 어떻게하지? eigen에서 알아서 해주나?
	//// eigen 문서 찾아보면 4*4 matrix까지만 유효한 것 같기도 하니... 다른 방법도 생각해보자.
	_determinant = _rJacobian_square.determinant();

	_manipulability = sqrt(fabs(_determinant));
	
	return(_manipulability);
}

void TeachingSW_HM::GetMnPlBltyGrad(Robot *_Robot, VectorXd &_MnPlBltyGrad)
{
	F<double> _manipulability;
	_manipulability = GetManipulability(_Robot);

	F<double> *_q;
	_q = new F<double>[_Robot->GetDOF()];

	_MnPlBltyGrad.resize(_Robot->GetDOF());
	
	for(int i=0; i<_Robot->GetDOF(); i++)
		_q[i].diff(i, _Robot->GetDOF());
	for(int i=0; i<_Robot->GetDOF(); i++)
		_q[i] = _Robot->GetQ(i);
	for(int i=0; i<_Robot->GetDOF(); i++)
		_MnPlBltyGrad(i) = _manipulability.d(i);

	delete _q;
}



/////////////////////////////////////////
////////// Orientation Representations
//// get orientation as rx, ry, rz according to the x-y-z fixed angle theory
void TeachingSW_HM::GetOrientation_fXYZ(vector <float> rExoData, vector <float>& rxryrz_R, vector <float>& rxryrz_L)
{
	rxryrz_R.resize(3);
	rxryrz_L.resize(3);
		
	/// introduction to robotics page. 43
	rxryrz_R[1] = atan2(-rExoData[8], sqrt(rExoData[0] * rExoData[0] + rExoData[4] * rExoData[4]));
	rxryrz_R[2] = atan2(rExoData[4]/cos(rxryrz_R[1]), rExoData[0]/cos(rxryrz_R[1]));
	rxryrz_R[0] = atan2(rExoData[9]/cos(rxryrz_R[1]), rExoData[10]/cos(rxryrz_R[1]));

	//// staubli uses degrees
	rxryrz_R[1] = 180 * rxryrz_R[1]/H_PI;
	rxryrz_R[2] = 180 * rxryrz_R[2]/H_PI;
	rxryrz_R[0] = 180 * rxryrz_R[0]/H_PI;

	/// introduction to robotics page. 43
	rxryrz_L[1] = atan2(-rExoData[23], sqrt(rExoData[15] * rExoData[15] + rExoData[19] * rExoData[19]));
	rxryrz_L[2] = atan2(rExoData[19]/cos(rxryrz_L[1]), rExoData[15]/cos(rxryrz_L[1]));
	rxryrz_L[0] = atan2(rExoData[24]/cos(rxryrz_L[1]), rExoData[25]/cos(rxryrz_L[1]));

	//// staubli uses degrees
	rxryrz_L[1] = 180 * rxryrz_L[1]/H_PI;
	rxryrz_L[2] = 180 * rxryrz_L[2]/H_PI;
	rxryrz_L[0] = 180 * rxryrz_L[0]/H_PI;

	//if(rxryrz[0] >= 180.0 && rxryrz[0] <=360)
	//	rxryrz[0] = rxryrz[0]-360;
	//else if(rxryrz[0] <=0 && rxryrz[0] >= -180.0)
	//	rxryrz[0] = -rxryrz[0];
	//else if(rxryrz[0] <= -180.0 && rxryrz[0] >= -360)
	//	we;
}

void TeachingSW_HM::GetOrientation_eXYZ(Matrix3d& rotMat, Vector3d& eAngle)
{
	//// 아래는 STaubli LLI 코드에서 직접 찾아본 것.
	double alpha = 0.0;
	double beta = 0.0;
	double gamma = 0.0;
	int sign = 1;
	if( rotMat(0,2) < (-1.0 + 1e-5*1e-5/2.0) ){
		alpha = 0;
		beta = -M_PI/2;
		gamma = atan2(rotMat(1,0), rotMat(3,1));
	}else if( rotMat(0,2) > (1.0 + 1e-5*1e-5/2.0) ) {
		alpha = 0;
		beta = M_PI/2;
		gamma = atan2(rotMat(1,0), rotMat(3,1));
	}else {
		beta = asin(rotMat(0,2));
		if( rotMat(2,2)<0.0 && rotMat(0,0)<0.0 && ( abs(rotMat(0,2))>1e-10 || abs(rotMat(1,2))>1e-10  ) )
		{
			if(beta >= 0.0){
				beta = M_PI-beta;
			}else {
				beta = -M_PI-beta;
			}
			sign = -1;
		}
		alpha = atan2(-sign*rotMat(1,2), sign*rotMat(2,2));
		gamma = atan2(-sign*rotMat(0,1), sign*rotMat(0,0));
	}

	eAngle[0] = alpha;
	eAngle[1] = beta;
	eAngle[2] = gamma;
}

//// get orientation as rx, ry, rz according to the z-y-z euler angle theory
void TeachingSW_HM::GetOrientation_eXYZ(Matrix3d OriR, Matrix3d OriL, Vector3d& rxryrz_R, Vector3d& rxryrz_L)
{
	rxryrz_R.resize(3);
	rxryrz_L.resize(3);
	
	/*
	/// introduction to robotics page. 43
	rxryrz_R[1] = atan2(sqrt(ExoOriR(2, 0) * ExoOriR(2, 0) + ExoOriR(2, 1) * ExoOriR(2, 1)), ExoOriR(2, 2));
	rxryrz_R[2] = atan2(-ExoOriR(2, 1)/sin(rxryrz_R[1]), ExoOriR(2, 0)/sin(rxryrz_R[1]));
	rxryrz_R[0] = atan2(ExoOriR(1, 2)/sin(rxryrz_R[1]), ExoOriR(0, 2)/sin(rxryrz_R[1]));

	//// staubli uses degrees
	rxryrz_R[1] = 180 * rxryrz_R[1]/3.141592;
	rxryrz_R[2] = 180 * rxryrz_R[2]/3.141592;
	rxryrz_R[0] = 180 * rxryrz_R[0]/3.141592;

	/// introduction to robotics page. 43
	rxryrz_L[1] = atan2(sqrt(ExoOriL(2, 0) * ExoOriL(2, 0) + ExoOriL(2, 1) * ExoOriL(2, 1)), ExoOriL(2, 2));
	rxryrz_L[2] = atan2(-ExoOriL(2, 1)/sin(rxryrz_L[1]), ExoOriL(2, 0)/sin(rxryrz_L[1]));
	rxryrz_L[0] = atan2(ExoOriL(1, 2)/sin(rxryrz_L[1]), ExoOriL(0, 2)/sin(rxryrz_L[1]));

	//// staubli uses degrees
	rxryrz_L[1] = 180 * rxryrz_L[1]/3.141592;
	rxryrz_L[2] = 180 * rxryrz_L[2]/3.141592;
	rxryrz_L[0] = 180 * rxryrz_L[0]/3.141592;

	//if(rxryrz[0] >= 180.0 && rxryrz[0] <=360)
	//	rxryrz[0] = rxryrz[0]-360;
	//else if(rxryrz[0] <=0 && rxryrz[0] >= -180.0)
	//	rxryrz[0] = -rxryrz[0];
	//else if(rxryrz[0] <= -180.0 && rxryrz[0] >= -360)
	//	we;
	*/ // 기존 방식의 문제를 해결한 것이 아래 방식이다. 기존 방식에서는 특히 경계조건에 도달했을 때 문제가 발생한다.
	
	if ( OriR(0, 2) < 1){
		if ( OriR(0, 2) > -1) // -1< ... <1
		{
			rxryrz_R[1] = asin(OriR(0, 2));
			rxryrz_R[0] = atan2(-OriR(1, 2), OriR(2, 2));
			rxryrz_R[2] = atan2(-OriR(0, 1), -OriR(0, 0));
		}
		else // <= -1 .. numerical problem으로 -1보다 작게 나올 수도 있음
		{
			// Not a unique solution : rxryrz_R[2] - rxryrz_R[0] = atan2 (r10 ,r11)
			rxryrz_R[1] = -H_PI/2;
			rxryrz_R[0] = -atan2(OriR(1, 0), OriR(1, 1));
			rxryrz_R[2] = 0;
		}
	}
	else // 1 <= .. numerical problem으로 1보다 크게 나올 수도 있음
	{
		// Not a unique solution : rxryrz_R[2] + rxryrz_R[0] = atan2 (r10 ,r11)
		rxryrz_R[1] = H_PI/2;
		rxryrz_R[0] = atan2(OriR(1, 0), OriR(1, 1));
		rxryrz_R[2] = 0;
	}

	if ( OriL(0, 2) < 1){
		if ( OriL(0, 2) > -1) // -1< ... <1
		{
			rxryrz_L[1] = asin(OriL(0, 2));
			rxryrz_L[0] = atan2(-OriL(1, 2), OriL(2, 2));
			rxryrz_L[2] = atan2(-OriL(0, 1), -OriL(0, 0));
		}
		else // <= -1 .. numerical problem으로 -1보다 작게 나올 수도 있음
		{
			// Not a unique solution : rxryrz_L[2] - rxryrz_L[0] = atan2 (r10 ,r11)
			rxryrz_L[1] = -H_PI/2;
			rxryrz_L[0] = -atan2(OriL(1, 0), OriL(1, 1));
			rxryrz_L[2] = 0;
		}
	}
	else // 1 <= .. numerical problem으로 1보다 크게 나올 수도 있음
	{
		// Not a unique solution : rxryrz_L[2] + rxryrz_L[0] = atan2 (r10 ,r11)
		rxryrz_L[1] = H_PI/2;
		rxryrz_L[0] = atan2(OriL(1, 0), OriL(1, 1));
		rxryrz_L[2] = 0;
	}

	rxryrz_R = rxryrz_R * H_RtoD;
	rxryrz_L = rxryrz_L * H_RtoD;
}

//// get orientation as rx, ry, rz according to the z-y-z euler angle theory
void TeachingSW_HM::GetOrientation_eZYZ(Matrix3d ExoOriR, Matrix3d ExoOriL, Vector3d& rxryrz_R, Vector3d& rxryrz_L)
{
	rxryrz_R.resize(3);
	rxryrz_L.resize(3);
	
	/*
	/// introduction to robotics page. 43
	rxryrz_R[1] = atan2(sqrt(ExoOriR(2, 0) * ExoOriR(2, 0) + ExoOriR(2, 1) * ExoOriR(2, 1)), ExoOriR(2, 2));
	rxryrz_R[2] = atan2(-ExoOriR(2, 1)/sin(rxryrz_R[1]), ExoOriR(2, 0)/sin(rxryrz_R[1]));
	rxryrz_R[0] = atan2(ExoOriR(1, 2)/sin(rxryrz_R[1]), ExoOriR(0, 2)/sin(rxryrz_R[1]));

	//// staubli uses degrees
	rxryrz_R[1] = 180 * rxryrz_R[1]/3.141592;
	rxryrz_R[2] = 180 * rxryrz_R[2]/3.141592;
	rxryrz_R[0] = 180 * rxryrz_R[0]/3.141592;

	/// introduction to robotics page. 43
	rxryrz_L[1] = atan2(sqrt(ExoOriL(2, 0) * ExoOriL(2, 0) + ExoOriL(2, 1) * ExoOriL(2, 1)), ExoOriL(2, 2));
	rxryrz_L[2] = atan2(-ExoOriL(2, 1)/sin(rxryrz_L[1]), ExoOriL(2, 0)/sin(rxryrz_L[1]));
	rxryrz_L[0] = atan2(ExoOriL(1, 2)/sin(rxryrz_L[1]), ExoOriL(0, 2)/sin(rxryrz_L[1]));

	//// staubli uses degrees
	rxryrz_L[1] = 180 * rxryrz_L[1]/3.141592;
	rxryrz_L[2] = 180 * rxryrz_L[2]/3.141592;
	rxryrz_L[0] = 180 * rxryrz_L[0]/3.141592;

	//if(rxryrz[0] >= 180.0 && rxryrz[0] <=360)
	//	rxryrz[0] = rxryrz[0]-360;
	//else if(rxryrz[0] <=0 && rxryrz[0] >= -180.0)
	//	rxryrz[0] = -rxryrz[0];
	//else if(rxryrz[0] <= -180.0 && rxryrz[0] >= -360)
	//	we;
	*/ // 기존 방식의 문제를 해결한 것이 아래 방식이다. 기존 방식에서는 특히 경계조건에 도달했을 때 문제가 발생한다.

	if ( ExoOriR(2, 2) < 1){
		if ( ExoOriR(2, 2) > -1) // -1< ... <1
		{
			rxryrz_R[1] = acos(ExoOriR(2, 2));
			rxryrz_R[0] = atan2(ExoOriR(1, 2), ExoOriR(0, 2));
			rxryrz_R[2] = atan2(ExoOriR(2, 1), -ExoOriR(2, 0));
		}
		else // <= -1 .. numerical problem으로 -1보다 작게 나올 수도 있음
		{
			// Not a unique solution : rxryrz_R[2] - rxryrz_R[0] = atan2 (r10 ,r11)
			rxryrz_R[1] = H_PI;
			rxryrz_R[0] = -atan2(ExoOriR(1, 0), ExoOriR(1, 1));
			rxryrz_R[2] = 0;
		}
	}
	else // 1 <= .. numerical problem으로 1보다 크게 나올 수도 있음
	{
		// Not a unique solution : rxryrz_R[2] + rxryrz_R[0] = atan2 (r10 ,r11)
		rxryrz_R[1] = 0;
		rxryrz_R[0] = atan2(ExoOriR(1, 0), ExoOriR(1, 1));
		rxryrz_R[2] = 0;
	}

	if ( ExoOriL(2, 2) < 1){
		if ( ExoOriL(2, 2) > -1) // -1< ... <1
		{
			rxryrz_L[1] = acos(ExoOriL(2, 2));
			rxryrz_L[0] = atan2(ExoOriL(1, 2), ExoOriL(0, 2));
			rxryrz_L[2] = atan2(ExoOriL(2, 1), -ExoOriL(2, 0));
		}
		else // <= -1 .. numerical problem으로 -1보다 작게 나올 수도 있음
		{
			// Not a unique solution : rxryrz_L[2] - rxryrz_L[0] = atan2 (r10 ,r11)
			rxryrz_L[1] = H_PI;
			rxryrz_L[0] = -atan2(ExoOriL(1, 0), ExoOriL(1, 1));
			rxryrz_L[2] = 0;
		}
	}
	else // 1 <= .. numerical problem으로 1보다 크게 나올 수도 있음
	{
		// Not a unique solution : rxryrz_L[2] + rxryrz_L[0] = atan2 (r10 ,r11)
		rxryrz_L[1] = 0;
		rxryrz_L[0] = atan2(ExoOriL(1, 0), ExoOriL(1, 1));
		rxryrz_L[2] = 0;
	}

	rxryrz_R = rxryrz_R * H_RtoD;
	rxryrz_L = rxryrz_L * H_RtoD;
}

Matrix3d TeachingSW_HM::GetRfrom_eXYZ(double alpha, double beta, double gamma)
{
	Matrix3d _Rmatrix;

	_Rmatrix(0, 0) = cos(beta) * cos(gamma);
	_Rmatrix(0, 1) = -cos(beta) * sin(gamma);
	_Rmatrix(0, 2) = sin(beta);
	_Rmatrix(1, 0) = sin(alpha) * sin(beta) * cos(gamma) + cos(alpha) * sin(gamma);
	_Rmatrix(1, 1) = -sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
	_Rmatrix(1, 2) = -sin(alpha) * cos(beta);
	_Rmatrix(2, 0) = -cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);
	_Rmatrix(2, 1) = cos(alpha) * sin(beta) * sin(gamma) + sin(alpha) * cos(gamma);
	_Rmatrix(2, 2) = cos(alpha) * cos(beta);

	return _Rmatrix;
}

Matrix3d TeachingSW_HM::GetRfrom_eZYZ(double alpha, double beta, double gamma)
{
	Matrix3d _Rmatrix;

	_Rmatrix(0, 0) = cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma);
	_Rmatrix(0, 1) = -cos(alpha)*cos(beta)*sin(gamma) - sin(alpha)*cos(gamma);
	_Rmatrix(0, 2) = cos(alpha)*sin(beta);
	_Rmatrix(1, 0) = sin(alpha)*cos(beta)*cos(gamma) + cos(alpha)*sin(gamma);
	_Rmatrix(1, 1) = -sin(alpha)*cos(beta)*sin(gamma) + cos(alpha)*cos(gamma);
	_Rmatrix(1, 2) = sin(alpha)*sin(beta);
	_Rmatrix(2, 0) = -sin(beta)*cos(gamma);
	_Rmatrix(2, 1) = sin(beta)*sin(gamma);
	_Rmatrix(2, 2) = cos(beta);

	return _Rmatrix;
}



Matrix3d TeachingSW_HM::GetRfrom_fXYZ(double alpha, double beta, double gamma)
{
	Matrix3d _Rmatrix;

	_Rmatrix(0, 0) = cos(alpha)*cos(beta);
	_Rmatrix(0, 1) = cos(alpha)*sin(beta)*sin(gamma); - sin(alpha)*cos(gamma);
	_Rmatrix(0, 2) = cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma);
	_Rmatrix(1, 0) = sin(alpha)*cos(beta);
	_Rmatrix(1, 1) = sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma);
	_Rmatrix(1, 2) = sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma);
	_Rmatrix(2, 0) = -sin(beta);
	_Rmatrix(2, 1) = cos(beta)*sin(gamma);
	_Rmatrix(2, 2) = cos(beta)*cos(gamma);

	return _Rmatrix;
}

Matrix3d TeachingSW_HM::GetRfrom_fZYZ(double alpha, double beta, double gamma)
{
	Matrix3d _Rmatrix;

	_Rmatrix(0, 0) = cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma);
	_Rmatrix(0, 1) = -cos(alpha)*cos(beta)*sin(gamma) - sin(alpha)*cos(gamma);
	_Rmatrix(0, 2) = cos(alpha)*sin(beta);
	_Rmatrix(1, 0) = sin(alpha)*cos(beta)*cos(gamma) + cos(alpha)*sin(gamma);
	_Rmatrix(1, 1) = -sin(alpha)*cos(beta)*sin(gamma) + cos(alpha)*cos(gamma);
	_Rmatrix(1, 2) = sin(alpha)*sin(beta);
	_Rmatrix(2, 0) = -sin(beta)*cos(gamma);
	_Rmatrix(2, 1) = sin(beta)*sin(gamma);
	_Rmatrix(2, 2) = cos(beta);

	return _Rmatrix;
}




void TeachingSW_HM::EulerParameters(Matrix3d _rInitialorientation, Matrix3d _rPresentorientation, Vector3d& _rRotationalaxis, double& _rRotationalangle)
{
	double e1 = 0, e2 = 0, e3 = 0, e4 = 0;
	Matrix3d Orientation_offset;
	Matrix3d _rInitialorientation_inv;

	_rInitialorientation_inv = _rInitialorientation.inverse(); 
	Orientation_offset = _rPresentorientation * _rInitialorientation_inv;
	e4 = 0.5 * sqrt(1 + Orientation_offset(0, 0) + Orientation_offset(1, 1) + Orientation_offset(2, 2));
	_rRotationalangle = 2 * acos(e4);
	//printf("%lf", e4); orientation resolution이 그렇게 작은데 e4가 0이 될리가 없지.. 1에 거의 가깝다.

	_rRotationalaxis(0) = (Orientation_offset(2, 1) - Orientation_offset(1, 2))/(2*sin(_rRotationalangle));
	_rRotationalaxis(1) = (Orientation_offset(0, 2) - Orientation_offset(2, 0))/(2*sin(_rRotationalangle));
	_rRotationalaxis(2) = (Orientation_offset(1, 0) - Orientation_offset(0, 1))/(2*sin(_rRotationalangle));
}




/////////////////////////////
//////// Transformation
Matrix3d	TeachingSW_HM::Rotate_with_X(double rAngle)
{
	//Eigen::MatrixXd	_Rotate_wth_X(3, 3);
	Eigen::Matrix3d	_Rotate_wth_X;
	
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

Matrix3d	TeachingSW_HM::Rotate_with_Y(double rAngle)
{
	//Eigen::MatrixXd	_Rotate_wth_Y(3, 3);
	Eigen::Matrix3d	_Rotate_wth_Y;
	
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

Matrix3d	TeachingSW_HM::Rotate_with_Z(double rAngle)
{
	//Eigen::MatrixXd	_Rotate_wth_Z(3, 3);
	Eigen::Matrix3d	_Rotate_wth_Z;
	
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

Matrix3d TeachingSW_HM::Rotate_with_Axis(Eigen::Vector3d rAxis, double rAngle)
{
	Eigen::Vector3d _Axis;
	Eigen::Matrix3d	_Rotate_wth_Axis;

	_Axis = rAxis;

	_Rotate_wth_Axis(0, 0) = cos(rAngle) + (1-cos(rAngle))*_Axis(0)*_Axis(0);
	_Rotate_wth_Axis(1, 0) = sin(rAngle)*_Axis(2) + (1-cos(rAngle))*_Axis(0)*_Axis(1);
	_Rotate_wth_Axis(2, 0) = -sin(rAngle)*_Axis(1) + (1-cos(rAngle))*_Axis(0)*_Axis(2);

	_Rotate_wth_Axis(0, 1) = -sin(rAngle)*_Axis(2) + (1-cos(rAngle))*_Axis(0)*_Axis(1);
	_Rotate_wth_Axis(1, 1) = cos(rAngle) + (1-cos(rAngle))*_Axis(1)*_Axis(1);
	_Rotate_wth_Axis(2, 1) = sin(rAngle)*_Axis(0) + (1-cos(rAngle))*_Axis(1)*_Axis(2);

	_Rotate_wth_Axis(0, 2) = sin(rAngle)*_Axis(1) + (1-cos(rAngle))*_Axis(0)*_Axis(2);
	_Rotate_wth_Axis(1, 2) = -sin(rAngle)*_Axis(0) + (1-cos(rAngle))*_Axis(1)*_Axis(2);
	_Rotate_wth_Axis(2, 2) = cos(rAngle) + (1-cos(rAngle))*_Axis(2)*_Axis(2);

	return(_Rotate_wth_Axis);
}

Matrix3d	TeachingSW_HM::Rotate_with_X_traj(double rT, double rAngle, double rPeriod, double rTranslation)
{
	Matrix3d	_Rotate_wth_X;
	
	_Rotate_wth_X(0, 0) = 1.0;
	_Rotate_wth_X(1, 0) = 0.0;
	_Rotate_wth_X(2, 0) = 0.0;

	_Rotate_wth_X(0, 1) = 0.0;
	_Rotate_wth_X(1, 1) = cos(rAngle*sin((rT-rTranslation)*rPeriod));
	_Rotate_wth_X(2, 1) = sin(rAngle*sin((rT-rTranslation)*rPeriod));

	_Rotate_wth_X(0, 2) = 0.0;
	_Rotate_wth_X(1, 2) = -sin(rAngle*sin((rT-rTranslation)*rPeriod));
	_Rotate_wth_X(2, 2) = cos(rAngle*sin((rT-rTranslation)*rPeriod));

	return(_Rotate_wth_X);
}

Matrix3d	TeachingSW_HM::Rotate_with_Y_traj(double rT, double rAngle, double rPeriod, double rTranslation)
{
	Matrix3d	_Rotate_wth_Y;
	
	_Rotate_wth_Y(0, 0) = cos(rAngle*sin((rT-rTranslation)*rPeriod));
	_Rotate_wth_Y(1, 0) = 0.0;
	_Rotate_wth_Y(2, 0) = -sin(rAngle*sin((rT-rTranslation)*rPeriod));

	_Rotate_wth_Y(0, 1) = 0.0;
	_Rotate_wth_Y(1, 1) = 1.0;
	_Rotate_wth_Y(2, 1) = 0.0;

	_Rotate_wth_Y(0, 2) = sin(rAngle*sin((rT - rTranslation)*rPeriod));
	_Rotate_wth_Y(1, 2) = 0.0;
	_Rotate_wth_Y(2, 2) = cos(rAngle*sin((rT - rTranslation)*rPeriod));

	return(_Rotate_wth_Y);
}

Matrix3d	TeachingSW_HM::Rotate_with_Z_traj(double rT, double rAngle, double rPeriod, double rTranslation)
{
	Matrix3d	_Rotate_wth_Z;
	
	_Rotate_wth_Z(0, 0) = cos(rAngle*sin((rT - rTranslation)*rPeriod));
	_Rotate_wth_Z(1, 0) = sin(rAngle*sin((rT - rTranslation)*rPeriod));
	_Rotate_wth_Z(2, 0) = 0.0;

	_Rotate_wth_Z(0, 1) = -sin(rAngle*sin((rT - rTranslation)*rPeriod));
	_Rotate_wth_Z(1, 1) = cos(rAngle*sin((rT - rTranslation)*rPeriod));
	_Rotate_wth_Z(2, 1) = 0.0;

	_Rotate_wth_Z(0, 2) = 0.0;
	_Rotate_wth_Z(1, 2) = 0.0;
	_Rotate_wth_Z(2, 2) = 1.0;

	return(_Rotate_wth_Z);
}
Matrix3d	TeachingSW_HM::Rotate_with_Axis_traj(Vector3d rAxis, double rT, double rAngle, double rPeriod, double rTranslation)
{
	Vector3d	_Axis;
	Matrix3d	_Rotate_wth_Axis;

	_Axis = rAxis;

	_Rotate_wth_Axis(0, 0) = cos(rAngle*sin((rT - rTranslation)*rPeriod)) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(0)*_Axis(0);
	_Rotate_wth_Axis(1, 0) = sin(rAngle*sin((rT - rTranslation)*rPeriod))*_Axis(2) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(0)*_Axis(1);
	_Rotate_wth_Axis(2, 0) = -sin(rAngle*sin((rT - rTranslation)*rPeriod))*_Axis(1) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(0)*_Axis(2);

	_Rotate_wth_Axis(0, 1) = -sin(rAngle*sin((rT - rTranslation)*rPeriod))*_Axis(2) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(0)*_Axis(1);
	_Rotate_wth_Axis(1, 1) = cos(rAngle*sin((rT - rTranslation)*rPeriod)) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(1)*_Axis(1);
	_Rotate_wth_Axis(2, 1) = sin(rAngle*sin((rT - rTranslation)*rPeriod))*_Axis(0) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(1)*_Axis(2);

	_Rotate_wth_Axis(0, 2) = sin(rAngle*sin((rT - rTranslation)*rPeriod))*_Axis(1) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(0)*_Axis(2);
	_Rotate_wth_Axis(1, 2) = -sin(rAngle*sin((rT - rTranslation)*rPeriod))*_Axis(0) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(1)*_Axis(2);
	_Rotate_wth_Axis(2, 2) = cos(rAngle*sin((rT - rTranslation)*rPeriod)) + (1-cos(rAngle*sin((rT - rTranslation)*rPeriod)))*_Axis(2)*_Axis(2);

	return(_Rotate_wth_Axis);
}



/////////////////////////////
///////////// 필터류
VectorXd TeachingSW_HM::LPF1(double alpha, VectorXd input, VectorXd output)
{
	for ( int i=0; i<input.size(); i++)
		output(i) = output(i) + alpha * (input(i) - output(i));
	return output;
}

VectorXd TeachingSW_HM::LPF2(double tau, double Ts, VectorXd input, VectorXd output)
{
	for ( int i=0; i<input.size(); i++)
		output(i) = (tau/(tau + Ts)) * output(i) + (Ts/(tau + Ts)) * input(i);
	return output;
}	



///////////////////////////////////////
/////////// Vision 데이터 활용 시 스냅샷 기능. 이것도 RGBY 순서겠지?RGBY 박스 순서 각각 x, y, z, rz 4DOF 정보
void TeachingSW_HM::Snapshot(VectorXd& snapshot)
{
	snapshot.resize(sizeof(T_ND.recv_vision)/8);
	snapshot.setZero();

	for(int i=0; i< sizeof(T_ND.recv_vision)/8; i++){
		snapshot(i) = T_ND.recv_vision[i];
		cout<<"vision data : "<<T_ND.recv_vision[i]<<endl;
	}

	cout<<"Get Vision Snapshot! "<<snapshot<<endl;
}

//// Vision 데이터 활용 시 Grip이 발생했을 때, ID를 부여하기 위해 배열에 상자 순서 넣기
void TeachingSW_HM::SetBoxID(Vector3d robotpos, VectorXd snpshot, VectorXd& _order, string& boxID)
{
	_order.resize(snpshot.size());

	//// 그리퍼가 동작했을 때, 어떤 상자에 대해 작동하는 것인지 알기 위한 알고리즘
	double dist_to_Rbox = 0, dist_to_Gbox = 0, dist_to_Bbox = 0, dist_to_Ybox = 0;
	for (int j = 0; j <3; j++){
		dist_to_Rbox += pow((double)(robotpos[j] - snpshot[j]), 2);
		dist_to_Gbox += pow((double)(robotpos[j] - snpshot[j+3]), 2);
		dist_to_Bbox += pow((double)(robotpos[j] - snpshot[j+6]), 2);
		dist_to_Ybox += pow((double)(robotpos[j] - snpshot[j+9]), 2);
	}
	
	if (dist_to_Rbox <= 50)
		boxID = "Red";
	else if (dist_to_Gbox <= 50)
		boxID = "Green";
	else if (dist_to_Bbox <= 50)
		boxID = "Green";
	else if (dist_to_Ybox <= 50)
		boxID = "Green";
	else
		boxID = "";
}





///////////////////////////////
//////// 그 외 기능 
// 문자열을 비교하는 함수
int TeachingSW_HM::StringComp(char msg1[], char msg2[])
{
	int nMsgLen1, nMsgLen2, i;
	nMsgLen1 = strlen(msg1);
	nMsgLen2 = strlen(msg2);
	if (msg1 && nMsgLen1 == nMsgLen2) // 일단 글자수가 같아야 같은 문자열이겠지요 
	{
		i=0;
		while(msg1[i] != NULL)   // 문자열 끝까지 
		{
			if (msg1[i] != msg2[i])  // 두 문자열의 글자가 하나라도 다르면;
			return 1;                       // 문자열 일치 하지 않음
			i++;
		}
		return 1;      // 문자열 길이가 같고 문자열 길이가 0보다 크며, 모두 동일하다면
	}                  // 두 문자열은 동일하다고 할 수 있으므로 1을 리턴합니다.
	else 
		return 0;
}