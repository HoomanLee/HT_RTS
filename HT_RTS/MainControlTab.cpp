// MainControlTab.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "MainControlTab.h"
#include "afxdialogex.h"

#include "HT_RTSDlg.h"	// 부모 윈도우 클래스 추가

#define MAXJOINT 9

#define RES_FONTSIZE	100
#define VALUE_FONTSIZE	100

CWinThread* pEXO_Thread = NULL;
CWinThread* pOMNI_Thread = NULL;
CWinThread* pCompute_Thread = NULL;


// CMainControlTab 대화 상자입니다.

UINT CMainControlTab::T_Compute(LPVOID _lParam)
{
	COpenGLControl *pParam = (COpenGLControl*) _lParam;

	bool bIKfin = false;

	while(1)
	{
		/* 시간 업데이트.. 나중에 update 기능 만들면 거기에 넣든지.. */
		theApp.T_SW->temp_time = clock()/(double)CLOCKS_PER_SEC;

		if (pParam->onIKstart){
			//// omni에서 값 얻어오기.
			if(theApp.T_SW->m_omni.connect_stats){
				//HT_target에 omni thread와 함께 접근하면 메모리 충돌 날 수도 있으니, omni thread에서 아예 값을 저장해버리자.
				//for(int r=0; r<3; r++){
				//	for(int c=0; c<3; c++)
				//		goal_T[0](r, c) = theApp.T_SW->m_omni.HT_target.linear()(r, c);
				//	goal_T[0](r, 3) = theApp.T_SW->m_omni.HT_target.translation()(r);
				//}
				/*if(theApp.T_SW->bool_Playback)
					theApp.T_SW->m_robot[0].SetQ(theApp.T_SW->des_q); */
			}
			//// exo에서 값 얻어오기.
			if(theApp.T_SW->m_exo.connect_stats){
				/* 원래 아래 코드에서 했는데, 메모리 충돌 날 수도 있으니 exo thread에서 아예 값을 바꿔버리자. */
				//vector <float> exo_data; //exo raw 데이터
				//Vector3d rxryrz_R, rxryrz_L, rxryrz_R_f, rxryrz_L_f, exo_pos_R_f, exo_pos_L_f;
				//theApp.T_SW->m_exo.DAQ_Exo(exo_data, theApp.T_SW->m_exo.exoHT_ee_R, theApp.T_SW->m_exo.exoHT_ee_L, theApp.T_SW->m_exo.exo_pos_elboR, theApp.T_SW->m_exo.exo_pos_elboL, theApp.T_SW->m_exo.button);
				//for(int i=0; i<3; i++){
				//	for(int j=0; j<3; j++){
				//		theApp.T_SW->m_exo.exo_ori_R(i, j) = theApp.T_SW->m_exo.exoHT_ee_R(i, j);//exo_data[i*4 + j];
				//		theApp.T_SW->m_exo.exo_ori_L(i, j) = theApp.T_SW->m_exo.exoHT_ee_L(i, j);//exo_data[((i+3)*4+3) + j];
				//	}
				//	theApp.T_SW->m_exo.exo_pos_R(i) = theApp.T_SW->m_exo.exoHT_ee_R(i, 3);//exo_data[i*4 + 3];
				//	theApp.T_SW->m_exo.exo_pos_L(i) = theApp.T_SW->m_exo.exoHT_ee_L(i, 3);//exo_data[((i+3)*4+3) + 3];
				//}
				//theApp.T_SW->GetOrientation_eZYZ(theApp.T_SW->m_exo.exo_ori_R, theApp.T_SW->m_exo.exo_ori_L*theApp.T_SW->Rotate_with_Z(M_PI), rxryrz_R, rxryrz_L);
				//Vector3d pos_forIK, ori_forIK;
				//pos_forIK(0) = theApp.T_SW->m_exo.exo_pos_R(0);// +300.0;
				//pos_forIK(1) = theApp.T_SW->m_exo.exo_pos_R(1);// +0.0;
				//pos_forIK(2) = theApp.T_SW->m_exo.exo_pos_R(2);// +300.0;
				//ori_forIK = rxryrz_R * H_DtoR;
				//theApp.T_SW->m_robot[0].MakeRobotTransform(pos_forIK/*robot_pos_R*/, ori_forIK/*robot_ori_R*/, pParam->goal_T[0], "Ezyz");
				//pos_forIK(0) = theApp.T_SW->m_exo.exo_pos_L(0);// +300.0;
				//pos_forIK(1) = theApp.T_SW->m_exo.exo_pos_L(1);// +0.0;
				//pos_forIK(2) = theApp.T_SW->m_exo.exo_pos_L(2);// +300.0;
				//ori_forIK = rxryrz_L * H_DtoR;
				//theApp.T_SW->m_robot[1].MakeRobotTransform(pos_forIK/*robot_pos_R*/, ori_forIK/*robot_ori_R*/, pParam->goal_T[1], "Ezyz");
			}

			//// TODO !
			// 양팔을 위한 루프. 한팔을 위한 루프 추가 요망! 
			if(theApp.GetNumRobot()==1) {	// 한팔일때

				// TODO, 궤적 저장을 위한 임시 변수들... -----------------------
				VectorXd jVec;
				jVec.resize(theApp.T_SW->m_robot[0].GetDOF());
				// -------------------------------------------------------------

				if(!theApp.T_SW->bool_Playback){
					double pos_res = 1.0;
					double ori_res = 0.01;
					double pos_error = (pParam->goal_T[0].col(3).segment(0,3) - theApp.T_SW->m_robot[0].GetTool().col(3).segment(0,3)).norm();
					Vector3d ori_error;
					ori_error(0) = 1.0 - pParam->goal_T[0].col(0).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(0).segment(0,3));
					ori_error(1) = 1.0 - pParam->goal_T[0].col(1).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(1).segment(0,3));
					ori_error(2) = 1.0 - pParam->goal_T[0].col(2).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(2).segment(0,3));
					VectorXd IKdq(6);
					IKdq = theApp.T_SW->IKLoop(&theApp.T_SW->m_robot[0], pParam->goal_T[0], pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);

					/* set upper bound on how far we attempt to move joints in a single update step */
					double max_dq = 0.1 * H_DtoR; // cout을 하면 1 * H_DtoR이 적당함
					for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++){
						if (IKdq(i) > max_dq){
							//cout<<"dq '" <<i<<" ' exceeds"<<IKdq * H_RtoD<<endl;
							IKdq = max_dq * IKdq/IKdq.norm();
						}
					}

					/**************** write value to robots ***************************/
					//// Staubli와 연결되어 있을 때에는 staubli에서 데이터를 얻어와야 sim과 real이 동기화가 되겠지?
					//WaitForSingleObject(theApp.T_SW->m_omni.event_sequence, 4/*INFINITE*/); // event가 signaled 상태가 되기를 기다린다.

					if(theApp.T_SW->_tcpip.bConnected){ 
						/* send */
						theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
						/* receive */
						theApp.T_SW->_tcpip.RecvData(CONTAINER, theApp.T_SW->T_ND.recv_container); // 조금 더 많은 내용을 포함한 데이터를 받자.		
						/* set Q for animate */
						for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++) {
							theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->T_ND.recv_container[i+6] * H_DtoR);
							// joint 데이터 임시 저장
							jVec(i) = theApp.T_SW->T_ND.recv_container[i+6];	// Degree
						}
					}
					else{
						if(!theApp.T_SW->bool_load)
							for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
								theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->m_robot[0].GetQ(i) + IKdq(i)); 
					}
					//ResetEvent(theApp.T_SW->m_omni.event_sequence);
					/********************************************************************/
				}
				else{
					////// Playback 시 자체 보간 알고리즘으로 돌리기
					//theApp.T_SW->playback_controller_q(theApp.T_SW->m_robot[0].GetDOF(), theApp.T_SW->pb_stepper, 2.0, theApp.T_SW->temp_time
					//, theApp.T_SW->event_time, theApp.T_SW->home_q, theApp.T_SW->event_q, theApp.T_SW->m_robot[0].GetQ(), 
					//theApp.T_SW->Teaching_data_j, theApp.T_SW->des_q, 0.01 * H_DtoR, theApp.T_SW->bool_Playback);
					//// Playback은 Staubli에 맡기고 sim을 Staubli data로 update
					//for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
					//	theApp.T_SW->des_q(i) = theApp.T_SW->T_ND.recv_container[i];

					//if(theApp.T_SW->bool_vision)
					//	theApp.T_SW->playback_wVsn(theApp.T_SW->m_vp[0], &theApp.T_SW->m_robot[0], theApp.T_SW->hash_track, theApp.T_SW->m_vp[0].hash_numindx, theApp.T_SW->m_vp[0].hash_gripindx, theApp.T_SW->m_vp[0].hash_releaseindx,
					//		theApp.T_SW->m_robot[0].GetDOF(), theApp.T_SW->pb_stepper, 2.0, theApp.T_SW->temp_time, 
					//		theApp.T_SW->event_time, theApp.T_SW->home_q, theApp.T_SW->event_q, theApp.T_SW->m_robot[0].GetQ(), 
					//	theApp.T_SW->Teaching_data_j, theApp.T_SW->des_q, 0.01 * H_DtoR, theApp.T_SW->bool_Playback);
					//else{
					//	if(theApp.T_SW->m_vp[0].nToolcnt == 0)
					//		theApp.T_SW->playback_controller_q(theApp.T_SW->m_robot[0].GetDOF(), theApp.T_SW->pb_stepper, 2.0, theApp.T_SW->temp_time
					//			, theApp.T_SW->event_time, theApp.T_SW->home_q, theApp.T_SW->event_q, theApp.T_SW->m_robot[0].GetQ(), 
					//			theApp.T_SW->Teaching_data_j, theApp.T_SW->des_q, 0.01 * H_DtoR, theApp.T_SW->bool_Playback);
					//	else
					//		/*theApp.T_SW->playback_woVsn(theApp.T_SW->m_vp[0], &theApp.T_SW->m_robot[0], theApp.T_SW->hash_track, theApp.T_SW->m_vp[0].hash_numindx, theApp.T_SW->m_vp[0].hash_gripindx, theApp.T_SW->m_vp[0].hash_releaseindx,
					//			theApp.T_SW->m_robot[0].GetDOF(), theApp.T_SW->pb_stepper, 2.0, theApp.T_SW->temp_time, 
					//			theApp.T_SW->event_time, theApp.T_SW->home_q, theApp.T_SW->event_q, theApp.T_SW->m_robot[0].GetQ(), 
					//			theApp.T_SW->Teaching_data_j, theApp.T_SW->des_q, 0.01 * H_DtoR, theApp.T_SW->bool_Playback);*/
					//		theApp.T_SW->playback_woVsn2(theApp.T_SW->m_vp[0], &theApp.T_SW->m_robot[0], theApp.T_SW->hash_track, theApp.T_SW->m_vp[0].hash_numindx, theApp.T_SW->m_vp[0].hash_gripindx, theApp.T_SW->m_vp[0].hash_releaseindx,
					//			theApp.T_SW->m_robot[0].GetDOF(), 2.0, theApp.T_SW->temp_time, 
					//			theApp.T_SW->event_time, theApp.T_SW->home_q, theApp.T_SW->event_q, theApp.T_SW->m_robot[0].GetQ(), 
					//			theApp.T_SW->Teaching_data_j, theApp.T_SW->des_q, 0.01 * H_DtoR, theApp.T_SW->bool_Playback);
					//}

					theApp.T_SW->playback_finalist(theApp.T_SW->m_vp[0], &theApp.T_SW->m_robot[0], theApp.T_SW->hash_track, 
							theApp.T_SW->m_robot[0].GetDOF(), 2.0, theApp.T_SW->temp_time, 
							theApp.T_SW->event_time, theApp.T_SW->home_q, theApp.T_SW->event_q, theApp.T_SW->m_robot[0].GetQ(), 
							theApp.T_SW->Teaching_data_j, theApp.T_SW->des_q, 0.01 * H_DtoR, theApp.T_SW->bool_Playback);


					//// Staubli와 연결되어 있을 때에는 staubli에서 데이터를 얻어와야 sim과 real이 동기화가 되겠지?
					//if(!theApp.T_SW->_tcpip.bConnected)						
					//	theApp.T_SW->m_robot[0].SetQ(theApp.T_SW->des_q); 
					
					/* Robot data 적용.. 양 쪽 thread에서 container에 동시에 접근할까봐 이 쪽에서 해줬다.. 나중에는 read->compute->write로 바꿔야겠음 */
					if(theApp.T_SW->_tcpip.bConnected) {
						/* send */
						/* playback일 때에는 같은 소켓이지만 joint 값을 보낸다 */
						if(theApp.T_SW->T_ND.temp_pos_ori[6] == true){
							for(int i=0; i<6; i++)
								theApp.T_SW->T_ND.temp_pos_ori[i] = theApp.T_SW->des_q(i) * H_RtoD;
							theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
						}
						/* receive */
						theApp.T_SW->_tcpip.RecvData(CONTAINER, theApp.T_SW->T_ND.recv_container); // 조금 더 많은 내용을 포함한 데이터를 받자.		
						/* set Q for animate */
						for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++) {
							theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->T_ND.recv_container[i+6] * H_DtoR);
							// joint 데이터 임시 저장
							jVec(i) = theApp.T_SW->T_ND.recv_container[i+6];	// Degree
						}
					}
					else
						for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
							theApp.T_SW->m_robot[0].SetQ(theApp.T_SW->des_q);
				}
					
				// onProcessing이라는 변수가 GUI에서 쓰이기 때문에 IK에서 bIKfin을 반환해 주어 이를 활용 가능케 했음
				if(!bIKfin)
					pParam->onProcessing[0] = true;
				else
					pParam->onProcessing[0] = false;
				
				
				//// Matrix로 저장
				static int cnt_traj = 0;
				//printf("idx: %d, rows: %d, num: %d, cnt_traj: %d  \n", theApp.T_SW->m_vp[0].idx, theApp.T_SW->m_vp[0].traj.rows(), num, cnt_traj);
				if(theApp.T_SW->m_vp[0].onRecord == true)
				{
					
					// TODO
					//// Teaching과 playback을 구별하기 위해 중간에 0 벡터를 삽입
					//jVec = theApp.T_SW->m_robot[0].GetQ();
					//theApp.T_SW->m_vp[0].traj.row(theApp.T_SW->m_vp[0].idx) = jVec;
					//theApp.T_SW->m_vp[0].idx++;
					////cout<<"theApp.T_SW->m_vp[0].traj.row(theApp.T_SW->m_vp[0].idx)"<<theApp.T_SW->m_vp[0].traj.row(theApp.T_SW->m_vp[0].idx)<<endl;

					if(cnt_traj > 100) cnt_traj = 0;
					cnt_traj++;
				}
				
					
			}
			else if(theApp.GetNumRobot()==2) { // 양팔일때 
				
				double pos_res = 1.0;
				double ori_res = 0.01;
				double pos_errorR = (pParam->goal_T[0].col(3).segment(0,3) - theApp.T_SW->m_robot[0].GetTool().col(3).segment(0,3)).norm();
				double pos_errorL = (pParam->goal_T[1].col(3).segment(0,3) - theApp.T_SW->m_robot[1].GetTool().col(3).segment(0,3)).norm();
				Vector3d ori_errorR;
				Vector3d ori_errorL;
				ori_errorR(0) = 1.0 - pParam->goal_T[0].col(0).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(0).segment(0,3));
				ori_errorR(1) = 1.0 - pParam->goal_T[0].col(1).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(1).segment(0,3));
				ori_errorR(2) = 1.0 - pParam->goal_T[0].col(2).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(2).segment(0,3));
				ori_errorL(0) = 1.0 - pParam->goal_T[1].col(0).segment(0,3).dot(theApp.T_SW->m_robot[1].GetTool().col(0).segment(0,3));
				ori_errorL(1) = 1.0 - pParam->goal_T[1].col(1).segment(0,3).dot(theApp.T_SW->m_robot[1].GetTool().col(1).segment(0,3));
				ori_errorL(2) = 1.0 - pParam->goal_T[1].col(2).segment(0,3).dot(theApp.T_SW->m_robot[1].GetTool().col(2).segment(0,3));
				VectorXd IKdq_total(16);
				IKdq_total = theApp.T_SW->IKLoop_dualArm_with_torso(&theApp.T_SW->m_robot[0], &theApp.T_SW->m_robot[1], pParam->goal_T[0], pParam->goal_T[1], pos_res, ori_res, abs(pos_errorR)+abs(pos_errorL), ori_errorR.norm()+ori_errorL.norm(), bIKfin);
				
				// onProcessing이라는 변수가 GUI에서 쓰이기 때문에 IK에서 bIKfin을 반환해 주어 이를 활용 가능케 했음
				if(!bIKfin){
					pParam->onProcessing[0] = true;
					pParam->onProcessing[1] = true;
				}
				else{
					pParam->onProcessing[0] = false;
					pParam->onProcessing[1] = false;
				}

				for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++){
					if (i < 2)
					{
						theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->m_robot[0].GetQ(i) + IKdq_total(i)); 
						theApp.T_SW->m_robot[1].SetQ(i, theApp.T_SW->m_robot[1].GetQ(i) + IKdq_total(i));
					}
					else 
					{
						theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->m_robot[0].GetQ(i) + IKdq_total(i));
						theApp.T_SW->m_robot[1].SetQ(i, theApp.T_SW->m_robot[1].GetQ(i) + IKdq_total(i+7)); 
					}
				}	
			}
				
		}

		Sleep(0);
	}
}
UINT CMainControlTab::T_getExoData(LPVOID _lParam)
{
	CMainControlTab *pParam = (CMainControlTab*) _lParam;
	
	while(1){
		if(theApp.T_SW->m_exo.button == SWITCH_R_TRI && (clock() - theApp.T_SW->m_exo.buttonclk)/(double)CLOCKS_PER_SEC >= 1){
			pParam->OnBnClickedTeachhere();
			theApp.T_SW->m_exo.buttonclk = clock();
		}
		if(theApp.T_SW->m_exo.button == SWITCH_L_TRI && (clock() - theApp.T_SW->m_exo.buttonclk)/(double)CLOCKS_PER_SEC >= 1 ){
			pParam->OnBnClickedTeachhere2();
			theApp.T_SW->m_exo.buttonclk = clock();
		}


		vector <float> exo_data; //exo raw 데이터
		Vector3d rxryrz_R, rxryrz_L, rxryrz_R_f, rxryrz_L_f, exo_pos_R_f, exo_pos_L_f;
		theApp.T_SW->m_exo.DAQ_Exo(exo_data, theApp.T_SW->m_exo.exoHT_ee_R, theApp.T_SW->m_exo.exoHT_ee_L, theApp.T_SW->m_exo.exo_pos_elboR, theApp.T_SW->m_exo.exo_pos_elboL, theApp.T_SW->m_exo.button);
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				theApp.T_SW->m_exo.exo_ori_R(i, j) = theApp.T_SW->m_exo.exoHT_ee_R(i, j);//exo_data[i*4 + j];
				theApp.T_SW->m_exo.exo_ori_L(i, j) = theApp.T_SW->m_exo.exoHT_ee_L(i, j);//exo_data[((i+3)*4+3) + j];
			}
			theApp.T_SW->m_exo.exo_pos_R(i) = theApp.T_SW->m_exo.exoHT_ee_R(i, 3);//exo_data[i*4 + 3];
			theApp.T_SW->m_exo.exo_pos_L(i) = theApp.T_SW->m_exo.exoHT_ee_L(i, 3);//exo_data[((i+3)*4+3) + 3];
		}

		theApp.T_SW->GetOrientation_eZYZ(theApp.T_SW->m_exo.exo_ori_R, theApp.T_SW->m_exo.exo_ori_L*theApp.T_SW->Rotate_with_Z(M_PI), rxryrz_R, rxryrz_L);
		Vector3d pos_forIK, ori_forIK;
		pos_forIK(0) = theApp.T_SW->m_exo.exo_pos_R(0);// +300.0;
		pos_forIK(1) = theApp.T_SW->m_exo.exo_pos_R(1);// +0.0;
		pos_forIK(2) = theApp.T_SW->m_exo.exo_pos_R(2);// +300.0;
		ori_forIK = rxryrz_R * H_DtoR;
		theApp.T_SW->m_robot[0].MakeRobotTransform(pos_forIK/*robot_pos_R*/, ori_forIK/*robot_ori_R*/, pParam->m_oglWindow.goal_T[0], "Ezyz");
		pos_forIK(0) = theApp.T_SW->m_exo.exo_pos_L(0);// +300.0;
		pos_forIK(1) = theApp.T_SW->m_exo.exo_pos_L(1);// +0.0;
		pos_forIK(2) = theApp.T_SW->m_exo.exo_pos_L(2);// +300.0;
		ori_forIK = rxryrz_L * H_DtoR;
		theApp.T_SW->m_robot[1].MakeRobotTransform(pos_forIK/*robot_pos_R*/, ori_forIK/*robot_ori_R*/, pParam->m_oglWindow.goal_T[1], "Ezyz");

		Sleep(0);
	}

	return 0;
}

UINT CMainControlTab::T_getOmniData(LPVOID _lParam)
{
	CMainControlTab *pParam = (CMainControlTab*) _lParam;
	
	Vector3d Vel_target;
	Vel_target.setZero();

	Vector3d position_buffer;
	position_buffer.setZero();
	Matrix3d orientation_buffer;
	orientation_buffer.setZero();
	Vector3d velocity_buffer;
	velocity_buffer.setZero();

	Vector3d staubli_home; //지금 값은 각 joint 값이 {0,55.7,113.06,0,11.24,0}
	staubli_home(0) = 299.99;
	staubli_home(1) = 20.0;
	staubli_home(2) = -360.63;

	Vector3d val_FT;
	val_FT.setZero();

	//// 티칭 내용 저장할 파일 경로 설정.
	const char* fname_j = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_j.txt";
	const char* fname_j_edited = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_j_edited.txt";
	const char* fname_hash_j = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash_j.txt";
	const char* fname_p = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_p.txt";
	const char* fname_p_edited = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_p_edited.txt";
	const char* fname_hash_p = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash.txt";
	
	//// hash테이블을 초기화하기 위해서. job file은 save 함수 쪽에서 알아서 초기화 된다. 
	ofstream(fname_hash_j, ios::out);
	ofstream(fname_hash_p, ios::out);

	//playback loop 돌기 위해서 시간을 임의로 결정해주자.
	theApp.T_SW->temp_time = 0.0;



	while(1){
		
		//LARGE_INTEGER liCounter1, liCounter2, liFrequency; // 함수에 걸리는 시간 측정하기 위해서
		//QueryPerformanceFrequency(&liFrequency);  // retrieves the frequency of the high-resolution performance counter 

		//QueryPerformanceCounter(&liCounter1);         // Start

		theApp.T_SW->m_omni.btn_curclk = clock();
		
		theApp.T_SW->m_omni.getHapticPos(position_buffer, orientation_buffer, velocity_buffer);
		theApp.T_SW->m_omni.RHmapper(theApp.T_SW->m_omni.HT_target, Vel_target, staubli_home, position_buffer, orientation_buffer, velocity_buffer); //robot의 home position과 haptics의 inkwell을 맞춰줌
																		//그러나 orientation 맞추기란 여간 어려운 것이 아님		
		
		if(theApp.T_SW->m_omni.BtnCmd.button1_prsd_time > 1.0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button1_prsd_time > 1.0)"<<endl;
			theApp.T_SW->T_ND.temp_button = BTN_1_LONG; //버튼 입력은 이제 이곳에 하자.
			
			if(theApp.T_SW->_tcpip.bConnected) //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);
			
			theApp.T_SW->m_omni.BtnCmd.button1_prsd_time = 0; //버튼 눌린 시간 초기화를 콜백에서 하면 너무 빠르게 돌아서 값을 얻어올 때 이미 초기화 되어있음.

			pParam->OnBnClickedPlayback();
		}
		else if(theApp.T_SW->m_omni.BtnCmd.button1_prsd_time < 1.0 && theApp.T_SW->m_omni.BtnCmd.button1_prsd_time != 0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button1_prsd_time < 1.0)"<<endl;
			theApp.T_SW->T_ND.temp_button = BTN_1_SHORT; //버튼 입력은 이제 이곳에 하자.

			if(theApp.T_SW->_tcpip.bConnected) //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);
			
			////// cartesian teaching
			//theApp.T_SW->temp_cartesian_data.resize(6);
			//for(int i = 0; i<6; i++)
			//	theApp.T_SW->temp_cartesian_data(i) = theApp.T_SW->T_ND.recv_container[i];//theApp.T_SW->m_omni.T_ND.received_posori[i];
			//theApp.T_SW->save_taught_data_p(fname_p, theApp.T_SW->temp_cartesian_data, theApp.T_SW->Teaching_data);
			//theApp.T_SW->Addheader(fname_p, fname_p_edited);
			//theApp.T_SW->save_hash_table(fname_hash_p, to_string((long double)theApp.T_SW->Teaching_data.rows()));
			
			////// joint teaching
			//theApp.T_SW->temp_joint_data.resize(6);
			//if(theApp.T_SW->_tcpip.bConnected)
			//	for(int i = 0; i<6; i++)
			//		theApp.T_SW->temp_joint_data(i) = theApp.T_SW->T_ND.recv_container[i+6];
			//else
			//	theApp.T_SW->temp_joint_data = theApp.T_SW->m_robot[0].GetQ();
			//theApp.T_SW->save_taught_data_q(fname_j, theApp.T_SW->temp_joint_data, theApp.T_SW->Teaching_data_j);
			//theApp.T_SW->Addheader(fname_j, fname_j_edited);
			//theApp.T_SW->save_hash_table(fname_hash_j, to_string((long double)theApp.T_SW->Teaching_data_j.rows()));

			////* STL로 Teaching 바꾼 후..  *////////
			pParam->OnBnClickedTeachhere(); //GUI에 visualize
			
			theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data, theApp.T_SW->Teaching_data_j);
			theApp.T_SW->Teaching_data_j *= H_DtoR;
			
			theApp.T_SW->save_hash_table(fname_hash_p, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
			theApp.T_SW->save_hash_table(fname_hash_j, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
			////********************************////

			theApp.T_SW->m_omni.BtnCmd.button1_prsd_time = 0; //버튼 눌린 시간 초기화를 콜백에서 하면 너무 빠르게 돌아서 값을 얻어올 때 이미 초기화 되어있음.
		}
		// 버튼 2 : 그리퍼 동작 (짧게 누르면 오픈, 길게 누르면 close)
		else if(theApp.T_SW->m_omni.BtnCmd.button2_prsd_time > 1.0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button2_prsd_time > 1.0)"<<endl;
			theApp.T_SW->T_ND.temp_button = BTN_2_LONG; //버튼 입력은 이제 이곳에 하자.
			if(theApp.T_SW->_tcpip.bConnected) //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);

			theApp.T_SW->save_hash_table(fname_hash_p, "Close");
			theApp.T_SW->save_hash_table(fname_hash_j, "Close");
			
			/* Gripper 동작 count. playback 할 때 등 사용될 수 있음. */
			theApp.T_SW->m_vp[0].nToolcnt++;

			////// hash table visulaize 위해서 임시로... //////
			pParam->OnBnClickedTeachhere();
			///////////////////////////////////////////////////

			////// 그리퍼가 동작했을 때, 어떤 상자에 대해 작동하는 것인지 알기 위한 알고리즘
			//double dist_to_Rbox = 0, dist_to_Gbox = 0, dist_to_Bbox = 0, dist_to_Ybox = 0;
			//for (int j = 0; j <3; j++){
			//	dist_to_Rbox += pow((double)(theApp.T_SW->m_omni.T_ND.recv_container[j] - theApp.T_SW->m_omni.snpshot[j]), 2);
			//	dist_to_Gbox += pow((double)(theApp.T_SW->m_omni.T_ND.recv_container[j] - theApp.T_SW->m_omni.snpshot[j+3]), 2);
			//	dist_to_Bbox += pow((double)(theApp.T_SW->m_omni.T_ND.recv_container[j] - theApp.T_SW->m_omni.snpshot[j+6]), 2);
			//	dist_to_Ybox += pow((double)(theApp.T_SW->m_omni.T_ND.recv_container[j] - theApp.T_SW->m_omni.snpshot[j+9]), 2);
			//}

			theApp.T_SW->m_omni.BtnCmd.button2_prsd_time = 0; //버튼 눌린 시간 초기화를 콜백에서 하면 너무 빠르게 돌아서 값을 얻어올 때 이미 초기화 되어있음.

			//if (dist_to_Rbox <= 50)
			//	boxID = "Red";
			//else if (dist_to_Gbox <= 50)
			//	boxID = "Green";
			//else if (dist_to_Bbox <= 50)
			//	boxID = "Green";
			//else if (dist_to_Ybox <= 50)
			//	boxID = "Green";
			//else
			//	boxID = 0;
		}
		else if(theApp.T_SW->m_omni.BtnCmd.button2_prsd_time < 1.0 && theApp.T_SW->m_omni.BtnCmd.button2_prsd_time != 0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button2_prsd_time < 1.0)"<<endl;
			theApp.T_SW->T_ND.temp_button = BTN_2_SHORT; //버튼 입력은 이제 이곳에 하자.
			if(theApp.T_SW->_tcpip.bConnected) //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);

			theApp.T_SW->save_hash_table(fname_hash_p, "Open");
			theApp.T_SW->save_hash_table(fname_hash_j, "Open");

			/* Gripper 동작 count. playback 할 때 등 사용될 수 있음. */
			theApp.T_SW->m_vp[0].nToolcnt++;

			////// hash table visulaize 위해서 임시로... //////
			pParam->OnBnClickedTeachhere();
			///////////////////////////////////////////////////


			theApp.T_SW->m_omni.BtnCmd.button2_prsd_time = 0; //버튼 눌린 시간 초기화를 콜백에서 하면 너무 빠르게 돌아서 값을 얻어올 때 이미 초기화 되어있음.
		}
		// 버튼 두 개 동시에 짧게 누르면 티칭 데이터 클리어, 길게 누르면 연속 경로 저장 : 이 때, 버튼 3과 staubli 쪽에 넘겨주는 3을 헷갈리지 말자.
		// 근데 현재의 버튼 콜백 함수로는 버튼 두 개 누른 것에 대해 인식이 잘 되지 않는다. 
		else if(theApp.T_SW->m_omni.BtnCmd.button3_prsd_time < 1.0 && theApp.T_SW->m_omni.BtnCmd.button3_prsd_time != 0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button3_prsd_time < 1.0)"<<endl;
			
			theApp.T_SW->Teaching_data.resize(0, 0);
			
			theApp.T_SW->T_ND.temp_button = 9; //버튼 입력은 이제 이곳에 하자.
			if(theApp.T_SW->_tcpip.bConnected) //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);

			theApp.T_SW->m_omni.BtnCmd.button3_prsd_time = 0; //버튼 눌린 시간 초기화를 콜백에서 하면 너무 빠르게 돌아서 값을 얻어올 때 이미 초기화 되어있음.
		}
		else if(theApp.T_SW->m_omni.BtnCmd.button3_prsd_time > 1.0 && theApp.T_SW->m_omni.BtnCmd.button3_prsd_time != 0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button3_prsd_time > 1.0)"<<endl;

			//cout<<"teaching continuous path"<<endl;
			theApp.T_SW->T_ND.temp_button = 5; //버튼 입력은 이제 이곳에 하자.
			if(theApp.T_SW->_tcpip.bConnected) //임시로.. 연결이 되었는지 check하는 기능을 robust하게 만들어야함.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);

			theApp.T_SW->temp_cartesian_data.resize(6);
			for(int i = 0; i<6; i++)
				theApp.T_SW->temp_cartesian_data(i) = theApp.T_SW->T_ND.recv_container[i];//;theApp.T_SW->m_omni.T_ND.received_posori[i];
			theApp.T_SW->save_countinuous_path(false, "%n", theApp.T_SW->temp_cartesian_data, theApp.T_SW->Teaching_data);			
	
			cout<<theApp.T_SW->Teaching_data.rows()<<endl;

			theApp.T_SW->m_omni.BtnCmd.button3_prsd_time = 0; //버튼 눌린 시간 초기화를 콜백에서 하면 너무 빠르게 돌아서 값을 얻어올 때 이미 초기화 되어있음.
		}
		else{
			theApp.T_SW->T_ND.temp_button = 0; //버튼 입력은 이제 이곳에 하자.
			//theApp.T_SW->m_omni.SendData_TBtnCMD(theApp.T_SW->m_omni.T_ND.temp_point[3]);
		}
		
		//* For position & orientation communication with Staubli *//
		Vector3d Ori_staubli;
		Matrix3d Ori_robotframe;
		Ori_robotframe = theApp.T_SW->m_omni.HT_target.linear();
		theApp.T_SW->GetOrientation_eXYZ(Ori_robotframe, Ori_staubli);

		/*****************************************************/
		theApp.T_SW->m_omni.HT_target.translation()(2) += 70.0; // for conveyor test
		if(theApp.T_SW->m_omni.HT_target.translation()(2) <= -305.0) // for conveyor test
			theApp.T_SW->m_omni.HT_target.translation()(2) = -305.0;
		/*****************************************************/

		for(int i=0; i<3; i++)
		{
			theApp.T_SW->T_ND.temp_pos_ori[i] = theApp.T_SW->m_omni.HT_target.translation().coeff(i);
			theApp.T_SW->T_ND.temp_pos_ori[i+3] = Ori_staubli.coeff(i) * H_RtoD;	//STaubli는 degree를 input으로 받는다.	
		}
	
		theApp.T_SW->T_ND.temp_pos_ori[6] = theApp.T_SW->bool_Playback; // 마지막 element는 playback여부

		//* For RTS simulator *//
		pParam->m_oglWindow.SetGoalPos(0, &theApp.T_SW->m_omni.HT_target);
		
		//SetEvent(theApp.T_SW->m_omni.event_sequence);

		//if(theApp.T_SW->_tcpip.bConnected){ 
		//	/* playback일 때에는 같은 소켓이지만 joint 값을 보낸다 */
		//	if(theApp.T_SW->T_ND.temp_pos_ori[6] == true){
		//		for(int i=0; i<6; i++)
		//			theApp.T_SW->T_ND.temp_pos_ori[i] = theApp.T_SW->des_q(i) * H_RtoD;
		//		theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
		//	}
		//	else
		//		theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
		//	theApp.T_SW->_tcpip.RecvData(CONTAINER, theApp.T_SW->T_ND.recv_container); // 조금 더 많은 내용을 포함한 데이터를 받자.		
		//}

		///* Robot data 적용.. 양 쪽 thread에서 container에 동시에 접근할까봐 이 쪽에서 해줬다.. 나중에는 read->compute->write로 바꿔야겠음 */
		//if(theApp.T_SW->_tcpip.bConnected)
		//	for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
		//		theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->T_ND.recv_container[i+6] * H_DtoR);

		Sleep(0);

		//SetEvent(theApp.T_SW->m_omni.event_sequence);          //Event 발생
		//PulseEvent(theApp.T_SW->m_omni.event_sequence);

		//QueryPerformanceCounter(&liCounter2);         // End
		//printf("Time : %f\n", (double)(liCounter2.QuadPart - liCounter1.QuadPart) / (double)liFrequency.QuadPart);
	}

	return 0;
}

IMPLEMENT_DYNAMIC(CMainControlTab, CDialogEx)

CMainControlTab::CMainControlTab(CWnd* pParent /*=NULL*/)
	: CDialogEx(CMainControlTab::IDD, pParent)
	, m_WorldPosStep(0.5)
	, m_WorldOriStep(0.5)
	, m_JointStep(0.5)
	, m_JointStep2(0.5)
	, m_ToolPosStep(0.5)
	, m_ToolOriStep(0.5)
{
	
}

CMainControlTab::~CMainControlTab()
{
}

void CMainControlTab::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_WORLDPOS_STEP, m_WorldPosStep);
	DDX_Text(pDX, IDC_WORLDORI_STEP, m_WorldOriStep);
	DDX_Text(pDX, IDC_JOINT_STEP, m_JointStep);
	DDX_Text(pDX, IDC_TOOLPOS_STEP, m_ToolPosStep);
	DDX_Text(pDX, IDC_TOOLORI_STEP, m_ToolOriStep);
	DDX_Text(pDX, IDC_JOINT_STEP2, m_JointStep2);
	DDX_Control(pDX, IDC_SPIN_WPSTEP, m_SpinWorldPosStep);
	DDX_Control(pDX, IDC_SPIN_WOSTEP, m_SpinWorldOriStep);
	DDX_Control(pDX, IDC_SPIN_JSTEP, m_SpinJointStep);
	DDX_Control(pDX, IDC_SPIN_JSTEP2, m_SpinJointStep2);
	DDX_Control(pDX, IDC_SPIN_TPSTEP, m_SpinToolPosStep);
	DDX_Control(pDX, IDC_SPIN_TOSTEP, m_SpinToolOriStep);
	DDX_Control(pDX, IDC_VIAPOINTLIST, m_vpList);
	DDX_Control(pDX, IDC_VIAPOINTLIST2, m_vpList2);
	DDX_Control(pDX, IDC_LIGHT, m_Light);
	DDX_Control(pDX, IDC_LIGHT2, m_Light2);
	DDX_Control(pDX, IDC_RECORD_LED, m_RecordLED);
}


BEGIN_MESSAGE_MAP(CMainControlTab, CDialogEx)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_WPSTEP, &CMainControlTab::OnDeltaposSpinWpstep)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_WOSTEP, &CMainControlTab::OnDeltaposSpinWostep)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_JSTEP, &CMainControlTab::OnDeltaposSpinJstep)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_TPSTEP, &CMainControlTab::OnDeltaposSpinTpstep)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_TOSTEP, &CMainControlTab::OnDeltaposSpinTostep)
	ON_WM_PAINT()
	ON_WM_SIZE()
	ON_WM_MOUSEWHEEL()
	ON_WM_LBUTTONDBLCLK()
	ON_WM_LBUTTONDOWN()
	ON_BN_CLICKED(IDC_J1PLUS, &CMainControlTab::OnBnClickedJ1plus)
	ON_BN_CLICKED(IDC_J1MINUS, &CMainControlTab::OnBnClickedJ1minus)
	ON_BN_CLICKED(IDC_J2PLUS, &CMainControlTab::OnBnClickedJ2plus)
	ON_BN_CLICKED(IDC_J2MINUS, &CMainControlTab::OnBnClickedJ2minus)
	ON_BN_CLICKED(IDC_J3PLUS, &CMainControlTab::OnBnClickedJ3plus)
	ON_BN_CLICKED(IDC_J3MINUS, &CMainControlTab::OnBnClickedJ3minus)
	ON_BN_CLICKED(IDC_J4PLUS, &CMainControlTab::OnBnClickedJ4plus)
	ON_BN_CLICKED(IDC_J4MINUS, &CMainControlTab::OnBnClickedJ4minus)
	ON_BN_CLICKED(IDC_J5PLUS, &CMainControlTab::OnBnClickedJ5plus)
	ON_BN_CLICKED(IDC_J5MINUS, &CMainControlTab::OnBnClickedJ5minus)
	ON_BN_CLICKED(IDC_J6PLUS, &CMainControlTab::OnBnClickedJ6plus)
	ON_BN_CLICKED(IDC_J6MINUS, &CMainControlTab::OnBnClickedJ6minus)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_XPLUS, &CMainControlTab::OnBnClickedXplus)
	ON_BN_CLICKED(IDC_XMINUS, &CMainControlTab::OnBnClickedXminus)
	ON_BN_CLICKED(IDC_YPLUS, &CMainControlTab::OnBnClickedYplus)
	ON_BN_CLICKED(IDC_YMINUS, &CMainControlTab::OnBnClickedYminus)
	ON_BN_CLICKED(IDC_ZPLUS, &CMainControlTab::OnBnClickedZplus)
	ON_BN_CLICKED(IDC_ZMINUS, &CMainControlTab::OnBnClickedZminus)
	ON_BN_CLICKED(IDC_RXPLUS, &CMainControlTab::OnBnClickedRxplus)
	ON_BN_CLICKED(IDC_RXMINUS, &CMainControlTab::OnBnClickedRxminus)
	ON_BN_CLICKED(IDC_RYPLUS, &CMainControlTab::OnBnClickedRyplus)
	ON_BN_CLICKED(IDC_RYMINUS, &CMainControlTab::OnBnClickedRyminus)
	ON_BN_CLICKED(IDC_RZPLUS, &CMainControlTab::OnBnClickedRzplus)
	ON_BN_CLICKED(IDC_RZMINUS, &CMainControlTab::OnBnClickedRzminus)
	ON_BN_CLICKED(IDC_ORICHECK, &CMainControlTab::OnBnClickedOricheck)
	ON_BN_CLICKED(IDC_LOADVIAPOINT, &CMainControlTab::OnBnClickedLoadviapoint)
	ON_BN_CLICKED(IDC_DRAWVIAPOINT, &CMainControlTab::OnBnClickedDrawviapoint)
	ON_NOTIFY(LVN_ITEMCHANGED, IDC_VIAPOINTLIST, &CMainControlTab::OnLvnItemchangedViapointlist)
	ON_BN_CLICKED(IDC_PLAYBACK, &CMainControlTab::OnBnClickedPlayback)
	ON_BN_CLICKED(IDC_RUNNING, &CMainControlTab::OnBnClickedRunning)
	ON_BN_CLICKED(IDC_J7PLUS, &CMainControlTab::OnBnClickedJ7plus)
	ON_BN_CLICKED(IDC_J7MINUS, &CMainControlTab::OnBnClickedJ7minus)
	ON_BN_CLICKED(IDC_J8PLUS, &CMainControlTab::OnBnClickedJ8plus)
	ON_BN_CLICKED(IDC_J8MINUS, &CMainControlTab::OnBnClickedJ8minus)
	ON_BN_CLICKED(IDC_J9PLUS, &CMainControlTab::OnBnClickedJ9plus)
	ON_BN_CLICKED(IDC_J9MINUS, &CMainControlTab::OnBnClickedJ9minus)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_JSTEP2, &CMainControlTab::OnDeltaposSpinJstep2)
	ON_BN_CLICKED(IDC_TXPLUS, &CMainControlTab::OnBnClickedTxplus)
	ON_BN_CLICKED(IDC_TXMINUS, &CMainControlTab::OnBnClickedTxminus)
	ON_BN_CLICKED(IDC_TYPLUS, &CMainControlTab::OnBnClickedTyplus)
	ON_BN_CLICKED(IDC_TYMINUS, &CMainControlTab::OnBnClickedTyminus)
	ON_BN_CLICKED(IDC_TZPLUS, &CMainControlTab::OnBnClickedTzplus)
	ON_BN_CLICKED(IDC_TZMINUS, &CMainControlTab::OnBnClickedTzminus)
	ON_BN_CLICKED(IDC_TRXPLUS, &CMainControlTab::OnBnClickedTrxplus)
	ON_BN_CLICKED(IDC_TRXMINUS, &CMainControlTab::OnBnClickedTrxminus)
	ON_BN_CLICKED(IDC_TRYPLUS, &CMainControlTab::OnBnClickedTryplus)
	ON_BN_CLICKED(IDC_TRYMINUS, &CMainControlTab::OnBnClickedTryminus)
	ON_BN_CLICKED(IDC_TRZPLUS, &CMainControlTab::OnBnClickedTrzplus)
	ON_BN_CLICKED(IDC_TRZMINUS, &CMainControlTab::OnBnClickedTrzminus)
	ON_BN_CLICKED(IDC_J1PLUS2, &CMainControlTab::OnBnClickedJ1plus2)
	ON_BN_CLICKED(IDC_J1MINUS2, &CMainControlTab::OnBnClickedJ1minus2)
	ON_BN_CLICKED(IDC_J2PLUS2, &CMainControlTab::OnBnClickedJ2plus2)
	ON_BN_CLICKED(IDC_J2MINUS2, &CMainControlTab::OnBnClickedJ2minus2)
	ON_BN_CLICKED(IDC_J3PLUS2, &CMainControlTab::OnBnClickedJ3plus2)
	ON_BN_CLICKED(IDC_J3MINUS2, &CMainControlTab::OnBnClickedJ3minus2)
	ON_BN_CLICKED(IDC_J4PLUS2, &CMainControlTab::OnBnClickedJ4plus2)
	ON_BN_CLICKED(IDC_J4MINUS2, &CMainControlTab::OnBnClickedJ4minus2)
	ON_BN_CLICKED(IDC_J5PLUS2, &CMainControlTab::OnBnClickedJ5plus2)
	ON_BN_CLICKED(IDC_J5MINUS2, &CMainControlTab::OnBnClickedJ5minus2)
	ON_BN_CLICKED(IDC_J6PLUS2, &CMainControlTab::OnBnClickedJ6plus2)
	ON_BN_CLICKED(IDC_J6MINUS2, &CMainControlTab::OnBnClickedJ6minus2)
	ON_BN_CLICKED(IDC_J7PLUS2, &CMainControlTab::OnBnClickedJ7plus2)
	ON_BN_CLICKED(IDC_J7MINUS2, &CMainControlTab::OnBnClickedJ7minus2)
	ON_BN_CLICKED(IDC_J8PLUS2, &CMainControlTab::OnBnClickedJ8plus2)
	ON_BN_CLICKED(IDC_J8MINUS2, &CMainControlTab::OnBnClickedJ8minus2)
	ON_BN_CLICKED(IDC_J9PLUS2, &CMainControlTab::OnBnClickedJ9plus2)
	ON_BN_CLICKED(IDC_J9MINUS2, &CMainControlTab::OnBnClickedJ9minus2)
	ON_BN_CLICKED(IDC_SAVEVIAPOINT, &CMainControlTab::OnBnClickedSaveviapoint)
	ON_BN_CLICKED(IDC_ADDVIA, &CMainControlTab::OnBnClickedAddvia)
	ON_BN_CLICKED(IDC_UP, &CMainControlTab::OnBnClickedUp)
	ON_BN_CLICKED(IDC_DELETEVIA, &CMainControlTab::OnBnClickedDeletevia)
	ON_BN_CLICKED(IDC_UP2, &CMainControlTab::OnBnClickedUp2)
	ON_NOTIFY(NM_DBLCLK, IDC_VIAPOINTLIST, &CMainControlTab::OnNMDblclkViapointlist)
	ON_BN_CLICKED(IDC_INITEXO, &CMainControlTab::OnBnClickedInitexo)
	ON_BN_CLICKED(IDC_TEACHHERE, &CMainControlTab::OnBnClickedTeachhere)
	ON_BN_CLICKED(IDC_DN, &CMainControlTab::OnBnClickedDn)
	ON_BN_CLICKED(IDC_DN2, &CMainControlTab::OnBnClickedDn2)
	ON_BN_CLICKED(IDC_TEACHHERE2, &CMainControlTab::OnBnClickedTeachhere2)
	ON_BN_CLICKED(IDC_ADDVIA2, &CMainControlTab::OnBnClickedAddvia2)
	ON_BN_CLICKED(IDC_DELETEVIA2, &CMainControlTab::OnBnClickedDeletevia2)
	ON_NOTIFY(LVN_ITEMCHANGED, IDC_VIAPOINTLIST2, &CMainControlTab::OnLvnItemchangedViapointlist2)
	ON_NOTIFY(NM_DBLCLK, IDC_VIAPOINTLIST2, &CMainControlTab::OnNMDblclkViapointlist2)
	ON_BN_CLICKED(IDC_INITOMNI, &CMainControlTab::OnBnClickedInitomni)
	ON_BN_CLICKED(IDC_TRAJ_RECORD, &CMainControlTab::OnBnClickedTrajRecord)
	ON_BN_CLICKED(IDC_CLEARVIA, &CMainControlTab::OnBnClickedClearvia)
	ON_BN_CLICKED(IDC_CLEARVIA2, &CMainControlTab::OnBnClickedClearvia2)
	ON_BN_CLICKED(IDC_TOPVIEW, &CMainControlTab::OnBnClickedTopview)
	ON_BN_CLICKED(IDC_FRONTVIEW, &CMainControlTab::OnBnClickedFrontview)
	ON_BN_CLICKED(IDC_LEFTVIEW, &CMainControlTab::OnBnClickedLeftview)
	ON_BN_CLICKED(IDC_VIEWUP, &CMainControlTab::OnBnClickedViewup)
	ON_BN_CLICKED(IDC_VIEWDOWN, &CMainControlTab::OnBnClickedViewdown)
	ON_BN_CLICKED(IDC_VIEWLEFT, &CMainControlTab::OnBnClickedViewleft)
	ON_BN_CLICKED(IDC_VIEWRIGHT, &CMainControlTab::OnBnClickedViewright)
	END_MESSAGE_MAP()




BOOL CMainControlTab::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  여기에 추가 초기화 작업을 추가합니다.
	
	// --------------------------------------------------------
	// 텍스트 출력창 초기화
	COLORREF bColor = RGB(64,64,64);
	COLORREF tColor = RGB(255,255,0);
	

	// Cartesian(World)
	const int nDof=6;
	CLargeText *CarLabel[nDof] = {&m_LabelX, &m_LabelY, &m_LabelZ, &m_LabelRX, &m_LabelRY, &m_LabelRZ};
	int idCarLabel[nDof] = {IDC_LABELX, IDC_LABELY, IDC_LABELZ, IDC_LABELRX, IDC_LABELRY, IDC_LABELRZ};

	CLargeText *CarData[nDof] = {&m_WorldX, &m_WorldY, &m_WorldZ, &m_WorldRX, &m_WorldRY, &m_WorldRZ};
	int idCarData[nDof] = {IDC_WORLDX, IDC_WORLDY, IDC_WORLDZ, IDC_WORLDRX, IDC_WORLDRY, IDC_WORLDRZ};

	CString capCarLabel[nDof] = {"X:", "Y:", "Z:", "RX:", "RY:", "RZ:"};

	for(int i=0; i<nDof; i++)
	{
		CarLabel[i]->SubclassDlgItem(idCarLabel[i], this);
		CarLabel[i]->SetFontSize(RES_FONTSIZE);
		CarLabel[i]->SetBackgroundColor(bColor);
		CarLabel[i]->SetTextColor(tColor);
		CarLabel[i]->SetCaption(capCarLabel[i]);
		
		CarData[i]->SubclassDlgItem(idCarData[i], this);
		CarData[i]->SetFontSize(VALUE_FONTSIZE);
	}
	



	// Joint 1
	const int nJoint = theApp.T_SW->m_robot[0].GetDOF();

	CLargeText *JntLabel[] = {&m_LabelJ1, &m_LabelJ2, &m_LabelJ3, &m_LabelJ4, &m_LabelJ5, &m_LabelJ6, &m_LabelJ7, &m_LabelJ8, &m_LabelJ9};
	int idJntLabel[] = {IDC_LABELJ1, IDC_LABELJ2, IDC_LABELJ3, IDC_LABELJ4, IDC_LABELJ5, IDC_LABELJ6, IDC_LABELJ7, IDC_LABELJ8, IDC_LABELJ9};

	CLargeText *JntData[] = {&m_Joint1, &m_Joint2, &m_Joint3, &m_Joint4, &m_Joint5, &m_Joint6, &m_Joint7, &m_Joint8, &m_Joint9};
	int idJntData[] = {IDC_JOINT1, IDC_JOINT2, IDC_JOINT3, IDC_JOINT4, IDC_JOINT5, IDC_JOINT6, IDC_JOINT7, IDC_JOINT8, IDC_JOINT9};

	int idJntBtnPlus[] = {IDC_J1PLUS, IDC_J2PLUS, IDC_J3PLUS, IDC_J4PLUS, IDC_J5PLUS, IDC_J6PLUS, IDC_J7PLUS, IDC_J8PLUS, IDC_J9PLUS};
	int idJntBtnMinus[] = {IDC_J1MINUS, IDC_J2MINUS, IDC_J3MINUS, IDC_J4MINUS, IDC_J5MINUS, IDC_J6MINUS, IDC_J7MINUS, IDC_J8MINUS, IDC_J9MINUS};

	int idJntDegree[] = {IDC_DEGREEJ1, IDC_DEGREEJ2, IDC_DEGREEJ3, IDC_DEGREEJ4, IDC_DEGREEJ5, IDC_DEGREEJ6, IDC_DEGREEJ7, IDC_DEGREEJ8, IDC_DEGREEJ9};
	
	CString capJntLabel[] = {"J1:", "J2:", "J3:", "J4:", "J5:", "J6:", "J7:", "J8:", "J9:"};

	for(int i=0; i<nJoint; i++)
	{
		JntLabel[i]->SubclassDlgItem(idJntLabel[i], this);
		JntLabel[i]->SetFontSize(RES_FONTSIZE);
		JntLabel[i]->SetBackgroundColor(bColor);
		JntLabel[i]->SetTextColor(tColor);
		JntLabel[i]->SetCaption(capJntLabel[i]);
		
		JntData[i]->SubclassDlgItem(idJntData[i], this);
		JntData[i]->SetFontSize(VALUE_FONTSIZE);
	}
	
	// 잉여 조인트 컨트롤은 숨긴다. 
	for(int i=nJoint; i<MAXJOINT; i++) {
		GetDlgItem(idJntLabel[i])->ShowWindow(SW_HIDE);		// Joint Label
		GetDlgItem(idJntData[i])->ShowWindow(SW_HIDE);		// Joint Data

		GetDlgItem(idJntBtnPlus[i])->ShowWindow(SW_HIDE);	// plus button
		GetDlgItem(idJntBtnMinus[i])->ShowWindow(SW_HIDE);	// minus button
		GetDlgItem(idJntDegree[i])->ShowWindow(SW_HIDE);	// degree
	}





	// Tool
	CLargeText *ToolLabel[] = {&m_LabelTX, &m_LabelTY, &m_LabelTZ, &m_LabelTRX, &m_LabelTRY, &m_LabelTRZ};
	CLargeText *ToolData[] = {&m_ToolX, &m_ToolY, &m_ToolZ, &m_ToolRX, &m_ToolRY, &m_ToolRZ};
	int idToolLabel[nDof] = {IDC_LABELTX, IDC_LABELTY, IDC_LABELTZ, IDC_LABELTRX, IDC_LABELTRY, IDC_LABELTRZ};
	int idToolData[] = {IDC_TOOLX, IDC_TOOLY, IDC_TOOLZ, IDC_TOOLRX, IDC_TOOLRY, IDC_TOOLRZ};
	int idToolBtnPlus[] = {IDC_TXPLUS, IDC_TYPLUS, IDC_TZPLUS, IDC_TRXPLUS, IDC_TRYPLUS, IDC_TRZPLUS};
	int idToolBtnMinus[] = {IDC_TXMINUS, IDC_TYMINUS, IDC_TZMINUS, IDC_TRXMINUS, IDC_TRYMINUS, IDC_TRZMINUS};
	int idToolEtc[] = {IDC_STATIC_TOOL1, IDC_STATIC_TOOL2, IDC_STATIC_TOOL3, IDC_STATIC_TOOL4, IDC_STATIC_TOOL5, IDC_STATIC_TOOL6, IDC_STATIC_TOOL7, IDC_STATIC_TOOL8};

	CString capToolLabel[] = {"TX:", "TY:", "TZ:", "TRX:", "TRY:", "TRZ:"};

	for(int i=0; i<nDof; i++)
	{
		ToolLabel[i]->SubclassDlgItem(idToolLabel[i], this);
		ToolLabel[i]->SetFontSize(RES_FONTSIZE);
		ToolLabel[i]->SetBackgroundColor(bColor);
		ToolLabel[i]->SetTextColor(tColor);
		ToolLabel[i]->SetCaption(capToolLabel[i]);
		
		ToolData[i]->SubclassDlgItem(idToolData[i], this);
		ToolData[i]->SetFontSize(VALUE_FONTSIZE);
	}
	


	
	// Joint 2
	CLargeText *JntLabel2[] = {&m_LabelJ1_2, &m_LabelJ2_2, &m_LabelJ3_2, &m_LabelJ4_2, &m_LabelJ5_2, &m_LabelJ6_2, &m_LabelJ7_2, &m_LabelJ8_2, &m_LabelJ9_2};
	int idJntLabel2[] = {IDC_LABELJ1_2, IDC_LABELJ2_2, IDC_LABELJ3_2, IDC_LABELJ4_2, IDC_LABELJ5_2, IDC_LABELJ6_2, IDC_LABELJ7_2, IDC_LABELJ8_2, IDC_LABELJ9_2};

	CLargeText *JntData2[] = {&m_Joint1_2, &m_Joint2_2, &m_Joint3_2, &m_Joint4_2, &m_Joint5_2, &m_Joint6_2, &m_Joint7_2, &m_Joint8_2, &m_Joint9_2};
	int idJntData2[] = {IDC_JOINT1_2, IDC_JOINT2_2, IDC_JOINT3_2, IDC_JOINT4_2, IDC_JOINT5_2, IDC_JOINT6_2, IDC_JOINT7_2, IDC_JOINT8_2, IDC_JOINT9_2};

	int idJntBtnPlus2[] = {IDC_J1PLUS2, IDC_J2PLUS2, IDC_J3PLUS2, IDC_J4PLUS2, IDC_J5PLUS2, IDC_J6PLUS2, IDC_J7PLUS2, IDC_J8PLUS2, IDC_J9PLUS2};
	int idJntBtnMinus2[] = {IDC_J1MINUS2, IDC_J2MINUS2, IDC_J3MINUS2, IDC_J4MINUS2, IDC_J5MINUS2, IDC_J6MINUS2, IDC_J7MINUS2, IDC_J8MINUS2, IDC_J9MINUS2};

	int idJntDegree2[] = {IDC_DEGREEJ1_2, IDC_DEGREEJ2_2, IDC_DEGREEJ3_2, IDC_DEGREEJ4_2, IDC_DEGREEJ5_2, IDC_DEGREEJ6_2, IDC_DEGREEJ7_2, IDC_DEGREEJ8_2, IDC_DEGREEJ9_2};
	
	int idViaControl[] = {IDC_VIAPOINTLIST2, IDC_TEACHHERE2, IDC_ADDVIA2, IDC_DELETEVIA2, IDC_CLEARVIA2, IDC_UP2, IDC_DN2};
	
	
	CString capJntLabel2[] = {"J1:", "J2:", "J3:", "J4:", "J5:", "J6:", "J7:", "J8:", "J9:"};
	

	if(theApp.GetNumRobot() > 1) {
		const int nJoint2 = theApp.T_SW->m_robot[1].GetDOF();

		for(int i=0; i<nJoint2; i++)
		{
			JntLabel2[i]->SubclassDlgItem(idJntLabel2[i], this);
			JntLabel2[i]->SetFontSize(RES_FONTSIZE);
			JntLabel2[i]->SetBackgroundColor(bColor);
			JntLabel2[i]->SetTextColor(tColor);
			JntLabel2[i]->SetCaption(capJntLabel2[i]);
		
			JntData2[i]->SubclassDlgItem(idJntData2[i], this);
			JntData2[i]->SetFontSize(VALUE_FONTSIZE);
		}

		// 잉여 조인트 컨트롤은 숨긴다. 
		for(int i=nJoint; i<MAXJOINT; i++) {
			GetDlgItem(idJntLabel2[i])->ShowWindow(SW_HIDE);		// Joint Label
			GetDlgItem(idJntData2[i])->ShowWindow(SW_HIDE);		// Joint Data

			GetDlgItem(idJntBtnPlus2[i])->ShowWindow(SW_HIDE);	// plus button
			GetDlgItem(idJntBtnMinus2[i])->ShowWindow(SW_HIDE);	// minus button
			GetDlgItem(idJntDegree2[i])->ShowWindow(SW_HIDE);	// degree
		}
	}else {
		// 로봇이 없을 경우 컨트롤을 다 지운다.  

		// Via Point 컨트롤들
		for(int i=0; i<sizeof(idViaControl)/sizeof(int); i++) {
			GetDlgItem(idViaControl[i])->ShowWindow(SW_HIDE);
		}


		// Tool 좌표 컨트롤 지우기
		for(int i=0; i<6; i++) {
			GetDlgItem(idToolLabel[i])->ShowWindow(SW_HIDE);
			GetDlgItem(idToolData[i])->ShowWindow(SW_HIDE);
			GetDlgItem(idToolBtnPlus[i])->ShowWindow(SW_HIDE);
			GetDlgItem(idToolBtnMinus[i])->ShowWindow(SW_HIDE);
			
		}
		// Tool etc 지우기
		for(int i=0; i<sizeof(idToolEtc)/sizeof(int); i++)
			GetDlgItem(idToolEtc[i])->ShowWindow(SW_HIDE);

		GetDlgItem(IDC_TOOLPOS_STEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SPIN_TPSTEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_TOOLORI_STEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SPIN_TOSTEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_LIGHT2)->ShowWindow(SW_HIDE);


		// Joint 컨트롤 지우기
		for(int i=0; i<MAXJOINT; i++) {
			GetDlgItem(idJntLabel2[i])->ShowWindow(SW_HIDE);		// Joint Label
			GetDlgItem(idJntData2[i])->ShowWindow(SW_HIDE);		// Joint Data

			GetDlgItem(idJntBtnPlus2[i])->ShowWindow(SW_HIDE);	// plus button
			GetDlgItem(idJntBtnMinus2[i])->ShowWindow(SW_HIDE);	// minus button
			GetDlgItem(idJntDegree2[i])->ShowWindow(SW_HIDE);	// degree
		}

		// Joint etc 지우기
		GetDlgItem(IDC_STATIC_STEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_JOINT_STEP2)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SPIN_JSTEP2)->ShowWindow(SW_HIDE);
		
	}


	// --------------------------------------------------------
	// Sping Control 초기화

	// World spin 
	m_SpinWorldPosStep.SetRange(1, 20);
	m_SpinWorldPosStep.SetPos(5);
	m_SpinWorldOriStep.SetRange(1, 20);
	m_SpinWorldOriStep.SetPos(5);
	
	// Joint 1 spin
	m_SpinJointStep.SetRange(1, 20);
	m_SpinJointStep.SetPos(5);
	

	// Tool spin
	m_SpinToolPosStep.SetRange(1, 20);
	m_SpinToolPosStep.SetPos(5);
	m_SpinToolOriStep.SetRange(1, 20);
	m_SpinToolOriStep.SetPos(5);

	// Joint 2 spin
	m_SpinJointStep2.SetRange(1, 20);
	m_SpinJointStep2.SetPos(5);
	
	
	// --------------------------------------------------------
	// OpenGL initialization
	CRect rect;

	// Get size and position of the picture control
	GetDlgItem(IDC_ROBOTSIMUL)->GetWindowRect(rect);
	
	//rect.SetRect(rect.left, rect.top, 800, 600);	// 터치 패드 크기에 맞게 조절해보기...

	// Convert screen coordinates to client coordinates
	ScreenToClient(rect);


	// Create OpenGL Control window
	m_oglWindow.oglCreate(rect, this);

	// Setup the OpenGL Window's timer to render
	// **TODO
	m_oglWindow.m_unpTimer = m_oglWindow.SetTimer(1,1,NULL);
	
	if(SetTimer(1, 10, NULL)!=1) {
		AfxMessageBox("Failed to set timer !");
	}


	// Via-point List initialization
	char *szText[8] = {"Num", "X", "Y", "Z", "RX", "RY", "RZ", "Hash"};
	int nWid[8] = {40, 60, 60, 60, 60, 60, 60, 60};
	LV_COLUMN lCol;
	lCol.mask = LVCF_FMT|LVCF_SUBITEM|LVCF_TEXT|LVCF_WIDTH;	// 구조체의 기능을 확장할 플래그 지정
	lCol.fmt = LVCFMT_LEFT;

	for(int i=0; i<8; i++)
	{
		lCol.pszText = szText[i];			// 컬럼의 제목을 지정
		lCol.iSubItem = i;					// 서브 아이템의 인덱스를 지정
		lCol.cx = nWid[i];					// 컬럼의 넓이를 지정
		m_vpList.InsertColumn(i, &lCol);	// LVCOLUMN구조체로 만들어진 값을 토대로 리스트 컨트롤에 삽입
		m_vpList2.InsertColumn(i, &lCol);
	}
	m_vpList.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);	// List control style 설정
	m_vpList2.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);	// List control style 설정, 2
	

	// via-point class 초기화!!
	for(int i=0; i<theApp.GetNumRobot(); i++)
		theApp.T_SW->m_vp[i].initViaPointClass(theApp.T_SW->m_robot[i]);


	// --------------------------------------------------------
	// Light Control 1 (World)
	m_Light.SubclassDlgItem(IDC_LIGHT, this);
	m_Light.SetMode(LIGHT_USERDEFINED);
	m_Light.SetColor(RGB(196,196,196));

	// Light Control 2 (TOOL, or Second arm)
	m_Light2.SubclassDlgItem(IDC_LIGHT2, this);
	m_Light2.SetMode(LIGHT_USERDEFINED);
	m_Light2.SetColor(RGB(196,196,196));

	// Record LED Control
	m_RecordLED.SubclassDlgItem(IDC_RECORD_LED, this);
	m_RecordLED.SetMode(LIGHT_USERDEFINED);
	m_RecordLED.SetColor(RGB(196,196,196));


	// Edit 창 숨기기
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT1),SW_HIDE);
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT2),SW_HIDE);


	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT1),SW_HIDE);


	// Picture Control window 크기 출력
	CStatic *staticSize = (CStatic *)GetDlgItem(IDC_ROBOTSIMUL);
	CRect pc_rect;
	
	staticSize->GetClientRect(pc_rect);

	int width = pc_rect.Width();
	int height = pc_rect.Height();

	printf("Picture control: width : %d, height : %d \n", width, height);

	return TRUE;  // return TRUE unless you set the focus to a control
	// 예외: OCX 속성 페이지는 FALSE를 반환해야 합니다.
}



BOOL CMainControlTab::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.


	/*CWnd* pwndCtrl = GetFocus();
	int id = pwndCtrl->GetDlgCtrlID();
	printf("id: %d \n", id);*/
	
	if(pMsg->message == WM_KEYUP){
		m_oglWindow.m_ShiftOn=FALSE;
	}
		

	if(pMsg->message == WM_KEYDOWN || pMsg->message == WM_SYSKEYDOWN)
	{
		//printf("key down \n");
		// 여기의 if문은 다이얼로그에서 Enter키와 ESC키를 누를 때
		// 창이 닫히는 것을 방지하기 위한 구문
		if(pMsg->wParam == VK_ESCAPE)
		{
			return FALSE;
		}

		if(pMsg->wParam == VK_RETURN)
		{
			if(pMsg->hwnd == GetDlgItem(IDC_EDIT1)->GetSafeHwnd())
			{
				cout << "enter!!! " << endl;
				// 값 쓰기
				CString str;
				GetDlgItemText(IDC_EDIT1, str);
				int count = str.GetLength();
				int i=0;


				if(ListclickX==7) {			// Hash table column
					
					// open, close, openv, closev인지를 체크
					char* temp = theApp.T_SW->m_vp[0].GetHash(ListclickY);	// 기존 hash table 값
					if((!strcmp(temp,"Open") || !strcmp(temp,"OpenV") || !strcmp(temp,"Close") || !strcmp(temp,"CloseV")) && temp) {
						char *hash = LPSTR(LPCTSTR(str));	// CString to char
						if(!strcmp(hash,"Open") || !strcmp(hash,"OpenV") || !strcmp(hash,"Close") || !strcmp(hash,"CloseV")) {
							theApp.T_SW->m_vp[0].SetHash(ListclickY, hash);
						}else {
						MessageBox("잘못된 형식입니다. ");
						}
					}else {
						MessageBox("숫자를 수정할 수 없습니다. ");
					}

				}else {						// 나머지 column
					for(i=0; i<count; i++)
					{
						char temp = str.GetAt(i);
					
						// 음수 처리
						if(i==0 && temp =='-') continue;
					
						// 소숫점 처리
						if(temp == '.') continue;

						// 입력된 키가 0~9 사이인지를 체크
						if(temp >='0' && temp <='9') continue;
						else break;
					}	

					if(i==count) {
					// TODO
					printf("lcy: %d, lcx: %d \n", ListclickY, ListclickX-1);
					theApp.T_SW->m_vp[0].SetVal(ListclickY,ListclickX-1,atof(str));
					printf("%f \n", theApp.T_SW->m_vp[0].GetVal(ListclickY,ListclickX-1));
					}
					else {
						MessageBox("잘못된 형식입니다. ");
					}
				}

				vpListUpdate(0);
				vpShowUpdate();

				::SendDlgItemMessageA(m_hWnd, IDC_EDIT1, WM_KILLFOCUS,0,0);
				::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT1),SW_HIDE);
				return FALSE;
			}
			else if(pMsg->hwnd == GetDlgItem(IDC_EDIT2)->GetSafeHwnd())
			{
				cout << "enter Left!!! " << endl;
				// 값 쓰기
				CString str;
				GetDlgItemText(IDC_EDIT2, str);
				int count = str.GetLength();
				int i=0;

				for(i=0; i<count; i++)
				{
					char temp = str.GetAt(i);
					
					// 음수 처리
					if(i==0 && temp =='-') continue;
					
					// 소숫점 처리
					if(temp == '.') continue;

					// 입력된 키가 0~9 사이인지를 체크
					if(temp >='0' && temp <='9') continue;
					else break;
				}

				if(i==count) {
					// TODO
					printf("lcy: %d, lcx: %d \n", ListclickY, ListclickX-1);
					theApp.T_SW->m_vp[1].SetVal(ListclickY,ListclickX-1,atof(str));
					printf("%f \n", theApp.T_SW->m_vp[1].GetVal(ListclickY,ListclickX-1));
					vpShowUpdate();
					vpListUpdate(1);
				}
				else {
					MessageBox("잘못된 형식입니다. ");
				}

				::SendDlgItemMessageA(m_hWnd, IDC_EDIT2, WM_KILLFOCUS,0,0);
				::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT2),SW_HIDE);
				return FALSE;
			}
			else 
			{
				return FALSE;
			}
		}

		// 특수키 검사
		BOOL bShift = ((GetKeyState(VK_SHIFT) & 0x8000) != 0);	// Shift 키 검사
		BOOL bControl = ((GetKeyState(VK_CONTROL) & 0x8000) != 0); // Control 키 검사
		BOOL bAlt = ((GetKeyState(VK_LMENU) & 0x8000) != 0);		// Alt 키 검사

		if(!bControl && !bShift && bAlt)	// Alt만 눌렸을 때
			m_oglWindow.m_AltOn=TRUE;
		else if(!bControl && bShift && !bAlt) // Shift만 눌렀을 때
			m_oglWindow.m_ShiftOn=TRUE;
		else if(bControl && !bShift && !bAlt)
		{
			if(pMsg->wParam == 'f' || pMsg->wParam == 'F') {	// Front
				m_oglWindow.setFrontView();
				printf("Control + F \n");
			}else if(pMsg->wParam == 'l' || pMsg->wParam == 'L') {	// Left
				m_oglWindow.setLeftView();
				printf("Control + L \n");
			}else if(pMsg->wParam == 'r' || pMsg->wParam == 'R') {	// Right
				printf("Control + R \n");
			}else if(pMsg->wParam == 't' || pMsg->wParam == 'T') {	// Top
				m_oglWindow.setTopView();
				printf("Control + T \n");
			}else if(pMsg->wParam == 'i' || pMsg->wParam == 'I') {	// Isometric
				m_oglWindow.setIsometricView();
				printf("Control + I \n");
			}
		}

	}
	

	return CDialogEx::PreTranslateMessage(pMsg);
}




void CMainControlTab::OnDeltaposSpinWpstep(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	
	CString get_value, set_value;
	GetDlgItemTextA(IDC_WORLDPOS_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(컨트롤 값을 얻어오기)
	m_WorldPosStep = atof(get_value);	// 10진 문제열을 double형 실수로..

	if(pNMUpDown->iDelta>0)	// Click up button
	{
		m_WorldPosStep+=0.5;
		if(m_WorldPosStep>30)	m_WorldPosStep=30.0f;
	}
	else	// click down button
	{
		m_WorldPosStep-=0.5;
		if(m_WorldPosStep<0.5)	m_WorldPosStep=0.5f;
	}

	set_value.Format("%1.1f", m_WorldPosStep);
	SetDlgItemTextA(IDC_WORLDPOS_STEP, set_value);
		UpdateData(FALSE);	// variable -> control
	
	*pResult = 0;

}


void CMainControlTab::OnDeltaposSpinWostep(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_WORLDORI_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(컨트롤 값을 얻어오기)
	m_WorldOriStep = atof(get_value);	// 10진 문제열을 double형 실수로..

	if(pNMUpDown->iDelta>0)	// Click up button
	{
		m_WorldOriStep+=0.5;
		if(m_WorldOriStep>10)	m_WorldOriStep=10.0f;
	}
	else	// click down button
	{
		m_WorldOriStep-=0.5;
		if(m_WorldOriStep<0.5)	m_WorldOriStep=0.5f;
	}

	set_value.Format("%1.1f", m_WorldOriStep);
	SetDlgItemTextA(IDC_WORLDORI_STEP, set_value);
		UpdateData(FALSE);	// variable -> control


	*pResult = 0;
}


void CMainControlTab::OnDeltaposSpinJstep(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.


	CString get_value, set_value;
	GetDlgItemTextA(IDC_JOINT_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(컨트롤 값을 얻어오기)
	m_JointStep = atof(get_value);	// 10진 문제열을 double형 실수로..

	if(pNMUpDown->iDelta>0)	// Click up button
	{
		m_JointStep+=0.5;
		if(m_JointStep>10)	m_JointStep=10.0f;
	}
	else	// click down button
	{
		m_JointStep-=0.5;
		if(m_JointStep<0.5)	m_JointStep=0.5f;
	}

	set_value.Format("%1.1f", m_JointStep);
	SetDlgItemTextA(IDC_JOINT_STEP, set_value);
		UpdateData(FALSE);	// variable -> control


	*pResult = 0;
}



void CMainControlTab::OnDeltaposSpinJstep2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_JOINT_STEP2, get_value);
		UpdateData(TRUE);	// control -> variable(컨트롤 값을 얻어오기)
	m_JointStep2 = atof(get_value);	// 10진 문제열을 double형 실수로..

	if(pNMUpDown->iDelta>0)	// Click up button
	{
		m_JointStep2+=0.5;
		if(m_JointStep2>10)	m_JointStep2=10.0f;
	}
	else	// click down button
	{
		m_JointStep2-=0.5;
		if(m_JointStep2<0.5)	m_JointStep2=0.5f;
	}

	set_value.Format("%1.1f", m_JointStep2);
	SetDlgItemTextA(IDC_JOINT_STEP2, set_value);
		UpdateData(FALSE);	// variable -> control


	*pResult = 0;
}


void CMainControlTab::OnDeltaposSpinTpstep(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_TOOLPOS_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(컨트롤 값을 얻어오기)
	m_ToolPosStep = atof(get_value);	// 10진 문제열을 double형 실수로..

	if(pNMUpDown->iDelta>0)	// Click up button
	{
		m_ToolPosStep+=0.5;
		if(m_ToolPosStep>30)	m_ToolPosStep=30.0f;
	}
	else	// click down button
	{
		m_ToolPosStep-=0.5;
		if(m_ToolPosStep<0.5)	m_ToolPosStep=0.5f;
	}

	set_value.Format("%1.1f", m_ToolPosStep);
	SetDlgItemTextA(IDC_TOOLPOS_STEP, set_value);
		UpdateData(FALSE);	// variable -> control

	*pResult = 0;	
}


void CMainControlTab::OnDeltaposSpinTostep(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_TOOLORI_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(컨트롤 값을 얻어오기)
	m_ToolOriStep = atof(get_value);	// 10진 문제열을 double형 실수로..

	if(pNMUpDown->iDelta>0)	// Click up button
	{
		m_ToolOriStep+=0.5;
		if(m_ToolOriStep>10)	m_ToolOriStep=10.0f;
	}
	else	// click down button
	{
		m_ToolOriStep-=0.5;
		if(m_ToolOriStep<0.5)	m_ToolOriStep=0.5f;
	}

	set_value.Format("%1.1f", m_ToolOriStep);
	SetDlgItemTextA(IDC_TOOLORI_STEP, set_value);
		UpdateData(FALSE);	// variable -> control

	*pResult = 0;
}




// --------------------------------------------------------------------
// 여기서부터 OpenGL
void CMainControlTab::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
	// 그리기 메시지에 대해서는 CDialogEx::OnPaint()을(를) 호출하지 마십시오.

	//printf("onPaint\n");
	CDialog::OnPaint();

}


void CMainControlTab::OnSize(UINT nType, int cx, int cy)
{
	CDialogEx::OnSize(nType, cx, cy);

	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
	switch (nType)
	{
		case SIZE_RESTORED:
		{
			if (m_oglWindow.m_bIsMaximized)
			{
			m_oglWindow.OnSize(nType, cx, cy);
			m_oglWindow.m_bIsMaximized = false;
			}
 
			break;
		}
 
		case SIZE_MAXIMIZED:
		{
			m_oglWindow.OnSize(nType, cx, cy);
			m_oglWindow.m_bIsMaximized = true;
 
			break;
		}
	}
}


BOOL CMainControlTab::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.

	// message 함수가 그렇게 빨리는 못잡아냄.. 훨을 너무 빨리 돌리면 그만큼 속도가 안난다. 

	CRect rt;
	((CStatic*)GetDlgItem(IDC_ROBOTSIMUL))->GetWindowRect(&rt);
	ScreenToClient(&rt);
	ScreenToClient(&pt);
	
	if(rt.PtInRect(pt)) {
		if(zDelta==120){	// wheel up
			m_oglWindow.m_fZoom -= -abs(m_oglWindow.m_fZoom*0.1);
			m_oglWindow.OnDraw(NULL);
			//printf("wheel up %lf\n", m_oglWindow.m_fZoom);
		}
		if(zDelta==-120){	// wheel down
			m_oglWindow.m_fZoom -= abs(m_oglWindow.m_fZoom*0.1);
			m_oglWindow.OnDraw(NULL);
			//printf("wheel down %lf\n", m_oglWindow.m_fZoom);
		}
	}

	return CDialogEx::OnMouseWheel(nFlags, zDelta, pt);
}


void CMainControlTab::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	printf("Double Click \n");
	CDialogEx::OnLButtonDblClk(nFlags, point);
}



void CMainControlTab::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	
	//printf("Notification \n");
	/// 창 내부 아무 곳이나 클릭해도 포커스를 얻어옴
	///////// 부모 클래스 포인터 얻어오기
	CHT_RTSDlg *dlg = (CHT_RTSDlg*) AfxGetMainWnd();
	dlg->m_MainControlTab.SetFocus();

	CDialogEx::OnLButtonDown(nFlags, point);
}



// 현재 좌표 및 조인트 값 출력
void CMainControlTab::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.

	switch(nIDEvent)
	{
		case 1:
		{
			// onProcess 상태 출력
			if(m_oglWindow.GetonProcess(0)){
				// Red 점등
				m_Light.SubclassDlgItem(IDC_LIGHT, this);
				m_Light.SetMode(LIGHT_USERDEFINED);
				m_Light.SetColor(RGB(255,0,0));
			}else {
				// Green 점등
				m_Light.SubclassDlgItem(IDC_LIGHT, this);
				m_Light.SetMode(LIGHT_USERDEFINED);
				m_Light.SetColor(RGB(0,255,0));
			}

			// 로봇 모델이 두 개인 경우...
			if(theApp.GetNumRobot() > 1) {
				// onProcess 상태 출력
				if(m_oglWindow.GetonProcess(1)){
					// Red 점등
					m_Light2.SubclassDlgItem(IDC_LIGHT2, this);
					m_Light2.SetMode(LIGHT_USERDEFINED);
					m_Light2.SetColor(RGB(255,0,0));
				}else {
					// Green 점등
					m_Light2.SubclassDlgItem(IDC_LIGHT2, this);
					m_Light2.SetMode(LIGHT_USERDEFINED);
					m_Light2.SetColor(RGB(0,255,0));
				}
			}
			

			// Joint 1 Coordinate출력 -----------------------------
			CLargeText *JntData[] = {&m_Joint1, &m_Joint2, &m_Joint3, &m_Joint4, &m_Joint5, &m_Joint6, &m_Joint7, &m_Joint8, &m_Joint9};
			int dof = theApp.T_SW->m_robot[0].GetDOF();
			CString str;
			for(int i=0; i<dof; ++i)
			{
				str.Format("%.2f", theApp.T_SW->m_robot[0].GetQ(i)*RtoD);	// Radian to degree	
				JntData[i]->SetCaption(str);
			}


			// World Coordinate 출력 -----------------------------
			CLargeText *WldData[6] = {&m_WorldX, &m_WorldY, &m_WorldZ, &m_WorldRX, &m_WorldRY, &m_WorldRZ};
			// Position
			for(int i=0; i<3; ++i)
			{
				str.Format("%.2f", theApp.T_SW->m_robot[0].GetTool()(i,3));
				WldData[i]->SetCaption(str);
			}
			// Orientation
			Vector3d eAngle;
			eAngle = theApp.T_SW->m_robot[0].GetOrientation();
			for(int i=0; i<3; ++i)
			{
				str.Format("%.2f", eAngle(i)*RtoD);
				WldData[i+3]->SetCaption(str);
			}


			// 로봇 모델이 두 개인 경우...
			if(theApp.GetNumRobot() > 1) {
				// Joint 2 Coordinate출력 -----------------------------
				CLargeText *JntData2[] = {&m_Joint1_2, &m_Joint2_2, &m_Joint3_2, &m_Joint4_2, &m_Joint5_2, &m_Joint6_2, &m_Joint7_2, &m_Joint8_2, &m_Joint9_2};
				int dof = theApp.T_SW->m_robot[1].GetDOF();
				CString str;
				for(int i=0; i<dof; ++i)
				{
					str.Format("%.2f", theApp.T_SW->m_robot[1].GetQ(i)*RtoD);	// Radian to degree	
					JntData2[i]->SetCaption(str);
				}

				
				// Tool Coordinate 출력 -----------------------------
				CLargeText *ToolData[6] = {&m_ToolX, &m_ToolY, &m_ToolZ, &m_ToolRX, &m_ToolRY, &m_ToolRZ};
				// Position
				for(int i=0; i<3; ++i)
				{
					str.Format("%.2f", theApp.T_SW->m_robot[1].GetTool()(i,3));
					ToolData[i]->SetCaption(str);
				}
				// Orientation
				Vector3d eAngle;
				eAngle = theApp.T_SW->m_robot[1].GetOrientation();
				for(int i=0; i<3; ++i)
				{
					str.Format("%.2f", eAngle(i)*RtoD);
					ToolData[i+3]->SetCaption(str);
				}

			}
		}
	}
	

	CDialogEx::OnTimer(nIDEvent);
}




// ------------------------------------------------------------------------
// Joint Control

// J1 up
void CMainControlTab::OnBnClickedJ1plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(0) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(0, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J1 down
void CMainControlTab::OnBnClickedJ1minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(0) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI){
		theApp.T_SW->m_robot[0].SetQ(0, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J2 up
void CMainControlTab::OnBnClickedJ2plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(1) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(1, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J2 down
void CMainControlTab::OnBnClickedJ2minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(1) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(1, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J3 up
void CMainControlTab::OnBnClickedJ3plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(2) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(2, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J3 down
void CMainControlTab::OnBnClickedJ3minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(2) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(2, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J4 up
void CMainControlTab::OnBnClickedJ4plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(3) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(3, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J4 down
void CMainControlTab::OnBnClickedJ4minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(3) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(3, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J5 up
void CMainControlTab::OnBnClickedJ5plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(4) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(4, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J5 down
void CMainControlTab::OnBnClickedJ5minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(4) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(4, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J6 up
void CMainControlTab::OnBnClickedJ6plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(5) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(5, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J6 down
void CMainControlTab::OnBnClickedJ6minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(5) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(5, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J7 up
void CMainControlTab::OnBnClickedJ7plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(6) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(6, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J7 down
void CMainControlTab::OnBnClickedJ7minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(6) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(6, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J8 up
void CMainControlTab::OnBnClickedJ8plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(7) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(7, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J8 down
void CMainControlTab::OnBnClickedJ8minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(7) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(7, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J9 up
void CMainControlTab::OnBnClickedJ9plus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(8) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(8, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J9 down
void CMainControlTab::OnBnClickedJ9minus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[0].GetQ(8) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(8, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}





void CMainControlTab::OnBnClickedRunning()
{
	m_oglWindow.SetinitGoal();
	
	if(!m_oglWindow.onIKstart) {
		if ( pCompute_Thread == NULL){
			pCompute_Thread = AfxBeginThread(T_Compute, &m_oglWindow, 0, 0, CREATE_SUSPENDED);
			cout<<"Compute Thread function began"<<endl;
		}
		pCompute_Thread->ResumeThread();
		m_oglWindow.onIKstart = TRUE;
		GetDlgItem(IDC_RUNNING)->SetWindowTextA("Running");	
	} 
	else {
		pCompute_Thread->SuspendThread();
		cout<<"Compute Thread function suspended"<<endl;
		m_oglWindow.onIKstart=FALSE;
		GetDlgItem(IDC_RUNNING)->SetWindowTextA("Initialization");
	}
}



// ------------------------------------------------------------------------
// Cartesian Position Control

void CMainControlTab::OnBnClickedXplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE){
		printf("X plus! \n");
		m_oglWindow.SetDesPosition(0,m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedXminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE){
		printf("X minus! \n");
		m_oglWindow.SetDesPosition(0,-m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedYplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("Y plus! \n");
		m_oglWindow.SetDesPosition(1,m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedYminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("Y minus! \n");
		m_oglWindow.SetDesPosition(1,-m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedZplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("Z plus! \n");
		m_oglWindow.SetDesPosition(2,m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedZminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("Z minus! \n");
		m_oglWindow.SetDesPosition(2,-m_WorldPosStep,0);
	}
}




// ------------------------------------------------------------------------
// Cartesian Orientation Control
void CMainControlTab::OnBnClickedRxplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.	
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RX plus! \n");
		m_oglWindow.SetDesPosition(3,m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRxminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RX minus! \n");
		m_oglWindow.SetDesPosition(3,-m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRyplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RY plus! \n");
		m_oglWindow.SetDesPosition(4,m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRyminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RY minus! \n");
		m_oglWindow.SetDesPosition(4,-m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRzplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RZ plus! \n");
		m_oglWindow.SetDesPosition(5,m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRzminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RZ minus! \n");
		m_oglWindow.SetDesPosition(5,-m_WorldOriStep,0);
	}
}







//// Tool Position Control--------------------
void CMainControlTab::OnBnClickedTxplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE){
		printf("X plus! \n");
		m_oglWindow.SetDesPosition(0,m_ToolPosStep,1);
	}
}

void CMainControlTab::OnBnClickedTxminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE){
		printf("X minus! \n");
		m_oglWindow.SetDesPosition(0,-m_ToolPosStep,1);
	}
}


void CMainControlTab::OnBnClickedTyplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("Y plus! \n");
		m_oglWindow.SetDesPosition(1,m_ToolPosStep,1);
	}
}


void CMainControlTab::OnBnClickedTyminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("Y minus! \n");
		m_oglWindow.SetDesPosition(1,-m_ToolPosStep,1);
	}
}


void CMainControlTab::OnBnClickedTzplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("Z plus! \n");
		m_oglWindow.SetDesPosition(2,m_ToolPosStep,1);
	}
}


void CMainControlTab::OnBnClickedTzminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("Z minus! \n");
		m_oglWindow.SetDesPosition(2,-m_ToolPosStep,1);
	}
}





//// Tool Orientation Control--------------------
void CMainControlTab::OnBnClickedTrxplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RX plus! \n");
		m_oglWindow.SetDesPosition(3,m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTrxminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RX minus! \n");
		m_oglWindow.SetDesPosition(3,-m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTryplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RY plus! \n");
		m_oglWindow.SetDesPosition(4,m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTryminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RY minus! \n");
		m_oglWindow.SetDesPosition(4,-m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTrzplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RZ plus! \n");
		m_oglWindow.SetDesPosition(5,m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTrzminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RZ minus! \n");
		m_oglWindow.SetDesPosition(5,-m_ToolOriStep,1);
	}
}







// Joint 2 Control
void CMainControlTab::OnBnClickedJ1plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(0) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(0, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ1minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(0) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(0, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ2plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(1) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(1, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ2minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(1) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(1, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ3plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(2) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(2, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ3minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(2) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(2, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ4plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(3) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(3, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ4minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(3) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(3, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ5plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(4) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(4, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ5minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(4) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(4, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ6plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(5) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(5, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ6minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(5) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(5, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ7plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(6) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(6, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ7minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(6) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(6, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ8plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(7) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(7, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ8minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(7) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(7, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ9plus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(8) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(8, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ9minus2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	float angle = theApp.T_SW->m_robot[1].GetQ(8) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(8, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}






// Temp button function
void CMainControlTab::OnBnClickedOricheck()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	cout << "save trajectories" << endl;
	
	// text에 저장
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	// 파일 로드 및 좌표 저장
	char szFilter[] =  "All Files (*.*)|*.*|Text Files (*.txt)|*.txt|";
	char fileName[] = "Trajectories_01";

	// 파일 저장 dialog 열기
	CFileDialog dlg(FALSE, "txt", fileName, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | 
										OFN_NOCHANGEDIR, szFilter, NULL);
	
	dlg.m_ofn.nFilterIndex = 2;			// txt선택

	FILE *fp = NULL;
	if(dlg.DoModal() == IDOK)
	{
		CString fileName = dlg.GetFileName();
		cout << fileName << endl;
		fp = fopen(fileName, "w+");
		if(fp != 0) {
			
			if(theApp.GetNumRobot() == 1){
				for(int r=0; r<theApp.T_SW->m_vp[0].traj.rows(); r++)	// row
				{
					for(int c=0; c<theApp.T_SW->m_vp[0].traj.cols(); c++)	// column
					{
						fprintf_s(fp, "%lf ", theApp.T_SW->m_vp[0].traj(r,c));
					}
					fprintf_s(fp, "\n");
				}
			}

		}
		fclose(fp);

	}

	//theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data, theApp.T_SW->Teaching_data_j);
	//m_oglWindow.TestFunc();
}


// Load via point
void CMainControlTab::OnBnClickedLoadviapoint()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	// 파일 로드 및 좌표 저장
	char szFilter[] = "text files(*.txt)|*.txt|All Files(*.*)|*.*|";
	CFileDialog dlg(TRUE, NULL, NULL, OFN_HIDEREADONLY, szFilter);
	//CFileDialog dlg(TRUE, "*.txt", NULL, OFN_HIDEREADONLY, "text files(*.txt)|*.txt|", NULL);

	char line_temp[1024];
	
	FILE* fp;
	if(dlg.DoModal() == IDOK)
	{
		int nRobot = theApp.GetNumRobot();

		// 기존에 저장 되어 있던 경유점 정보 초기화
		theApp.T_SW->m_vp[0].SetClear();				// Data
		m_vpList.DeleteAllItems();	// List Control
		vpListUpdate(0);
		if(nRobot == 2) {
			theApp.T_SW->m_vp[1].SetClear();				// Data
			m_vpList2.DeleteAllItems();	// List Control
			vpListUpdate(1);
		}
		vpShowUpdate();				// OpenGL
		
		
		
		// 양팔일 경우, Left, Right 둘 중 어느 것을 먼저 선택해도 가능 하도록..
		if(nRobot == 2) {	 
			CString tempName;
			CString post[2];
			int order[2];

			int nIndexR = dlg.GetFileTitle().GetLength() - dlg.GetFileTitle().ReverseFind('_R');
			int nIndexL = dlg.GetFileTitle().GetLength() - dlg.GetFileTitle().ReverseFind('_L');
			if(nIndexR == 1 && nIndexL != 1) {
				cout << "Right Arm!" << endl;
				tempName = dlg.GetFileTitle();
				tempName.Delete(dlg.GetFileTitle().ReverseFind('_R')-1, 2);
				post[0] = "_R"; post[1] = "_L";
				order[0] = 0;	order[1] = 1;
			}else if(nIndexL == 1 && nIndexR != 1) {
				cout << "Left Arm!" << endl;
				tempName = dlg.GetFileTitle();
				tempName.Delete(dlg.GetFileTitle().ReverseFind('_L')-1, 2);
				post[0] = "_L"; post[1] = "_R";
				order[0] = 1;	order[1] = 0;
			}else {
				cout << "Cannot load.." << endl;
				return;
			}

			for(int n=0; n<theApp.GetNumRobot(); n++)
			{
				CString fileName = tempName + post[n] + ".txt";
				cout << fileName << endl;
				fp = fopen(fileName.GetBuffer(100),"r");

				if(fp==NULL) {
					MessageBox("파일 열기 에러");
					return;
				}

				UINT line_count = 0;
				while(!feof(fp))
				{
					memset(line_temp, NULL, 1024*sizeof(char));		// reset buffer
					fgets(line_temp,1024-5,fp);						// 한 라인 읽기
					if(strlen(line_temp) < 2) break;				// line에 character가 2개 보다 적으면 데이터 없는 것으로 간주

					char value_temp[30] = {""};
					int dim = 12+theApp.T_SW->m_robot[order[n]].GetDOF();	// H matrix element + robot dof
					double *ar_val = new double[dim];
					UINT index = 0;
					UINT iDof = 0;

					// 한 Line을 읽어서 배열에 저장
					for(int i=0; i<strlen(line_temp); i++)
					{
						if(line_temp[i] == 32)	// space 검출(ASCii==32 space)
						{
							// 소수점 이하 n번째 자리 이하 자르기
							int cut = 3;
							ar_val[iDof] = ((int) (atof(value_temp)*pow(10.0,cut)))/pow(10.0,cut);	// space를 기준으로 문자를 숫자로 바꾸어 배열에 저장(x, y, z, rx, ry, rz)
					
							memset(value_temp, NULL, sizeof(int)*30);
							index = 0;
							iDof++;
						}
						value_temp[index] = line_temp[i];
						index++;
					}

					theApp.T_SW->m_vp[order[n]].PushValArray(ar_val);
					memset(ar_val, NULL, dim*sizeof(double));

					iDof = 0;
					line_count++;
					free(ar_val);
				}

				// 리스트에 값 출력, http://shaeod.tistory.com/251 (숫자열을 문자열로 변환 관련 링크)
				LVITEM item;
				item.mask = LVIF_TEXT;
				char nullBuffer[50];
				
				cout << "Line_count: " << line_count << endl;
				for(int j=0; j<line_count; j++)
				{
					// num 출력
					item.iItem = j;
					item.iSubItem = 0;
					item.pszText = itoa(j+1, nullBuffer, 10);
					(order[n]==0) ? m_vpList.InsertItem(&item): m_vpList2.InsertItem(&item);	// 최상 목록 추가
					for(int i=0; i<DIM; i++)	// column 원소 추가
					{
						item.iSubItem = i+1;
						item.pszText = gcvt(theApp.T_SW->m_vp[order[n]].GetVal(j,i), 7, nullBuffer);
						(order[n]==0) ? m_vpList.SetItem(&item) : m_vpList2.SetItem(&item); // 하위 목록 추가는 SetItem 사용
					}
				}
			}
			
		



		// 한팔일 경우
		}else if(nRobot == 1) {	
			CString fileName = dlg.GetFileName();
			fp = fopen(fileName.GetBuffer(100),"r");

			if(fp==NULL) {
				MessageBox("파일 열기 에러");
				return;
			}
			UINT line_count = 0;

			while(!feof(fp))
			{
				memset(line_temp, NULL, 1024*sizeof(char));		// reset buffer
				fgets(line_temp,1024-5,fp);						// 한 라인 읽기
				if(strlen(line_temp) < 2) break;				// line에 character가 2개 보다 적으면 데이터 없는 것으로 간주

				char value_temp[30] = {""};
				int dim = 12+theApp.T_SW->m_robot[0].GetDOF();	// H matrix element + robot dof
				double *ar_val = new double[dim];
				UINT index = 0;
				UINT iDof = 0;

				// 한 Line을 읽어서 배열에 저장
				for(int i=0; i<strlen(line_temp); i++)
				{
					if(line_temp[i] == 32)	// space 검출(ASCii==32 space)
					{
						// 소수점 이하 n번째 자리 이하 자르기
						int n = 6;
						ar_val[iDof] = ((int) (atof(value_temp)*pow(10.0,n)))/pow(10.0,n);	// space를 기준으로 문자를 숫자로 바꾸어 배열에 저장(x, y, z, rx, ry, rz)
					
						// Push back으로 구현해야할 듯!!
						memset(value_temp, NULL, sizeof(int)*30);
						index = 0;
						iDof++;
					}
					value_temp[index] = line_temp[i];
					index++;
				}
			

				theApp.T_SW->m_vp[0].PushValArray(ar_val);
				memset(ar_val, NULL, dim*sizeof(double));

				iDof = 0;
				line_count++;
				free(ar_val);
			}
			
			// 리스트에 값 출력, http://shaeod.tistory.com/251 (숫자열을 문자열로 변환 관련 링크)
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
				
			cout << "Line_count: " << line_count << endl;
			for(int j=0; j<line_count; j++)
			{
				// num 출력
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				m_vpList.InsertItem(&item);
				for(int i=0; i<DIM; i++)	// column 원소 추가
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[0].GetVal(j,i), 7, nullBuffer);
					m_vpList.SetItem(&item);
				}
			}


		}

		vpShowUpdate();
	}


		
}



// Save via point
void CMainControlTab::OnBnClickedSaveviapoint()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	// 파일 로드 및 좌표 저장
	char szFilter[] =  "All Files (*.*)|*.*|Text Files (*.txt)|*.txt|";
	char fileName[] = "Via-Point01";
	CFileDialog dlg(FALSE, "txt", fileName, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | 
										OFN_NOCHANGEDIR, szFilter, NULL);

	dlg.m_ofn.nFilterIndex = 2;				// txt선택
	int nRobot = theApp.GetNumRobot();	// 한팔 로봇인지, 양팔 로봇인지를 파악하기 위함
	
	
	
	FILE *fp = NULL;
	if(dlg.DoModal() == IDOK)
	{
		if(nRobot == 2) {	// 양팔인 경우
			CString post[2] = {"_R", "_L"};
			CString fileName = dlg.GetFileTitle();
			
			for(int n=0; n<nRobot; n++)
			{
				CString fileName2 = fileName + post[n] + ".txt";
				cout << fileName2 << endl;
				fp = fopen(fileName2, "w+");
				if(fp != 0) {
					ViaPoint *vp = &theApp.T_SW->m_vp[n];
					for(int j=0; j<vp->GetNum(); j++) {

					// [rx ry rz pos joints]
					// H matrix 
					Matrix4d hMat = vp->GetHmatrix(j);
					for(int c=0; c<4; c++)	// column
					for(int r=0; r<3; r++)	// row
					{
						fprintf_s(fp, "%lf ", hMat(r, c));
					}

					// Joints
					VectorXd Jnt = vp->GetJoint(j);
					for(int i=0; i<Jnt.size(); i++)
					{
						fprintf_s(fp, "%lf ", Jnt(i));
					}
					
					fprintf_s(fp, "\n");
					}
				}
				fclose(fp);		
			}

		}else {		// 한팔인 경우
			CString fileName = dlg.GetFileName();
			cout << fileName << endl;
			fp = fopen(fileName, "w+");
			if(fp != 0) {
				ViaPoint *vp = &theApp.T_SW->m_vp[0];
				for(int j=0; j<vp->GetNum(); j++) {

					// [rx ry rz pos joints]
					// H matrix
					Matrix4d hMat = vp->GetHmatrix(j);
					for(int c=0; c<4; c++)	// column
					for(int r=0; r<3; r++)	// row
					{
						fprintf_s(fp, "%lf ", hMat(r, c));
					}

					// Joints
					VectorXd Jnt = vp->GetJoint(j);
					for(int i=0; i<Jnt.size(); i++)
					{
						fprintf_s(fp, "%lf ", Jnt(i));
					}

					fprintf_s(fp, "\n");
				}
			}
			fclose(fp);		
		}

	}
}






// 현재 저장된 경유점을 openGL via-point에 복사한다.
void CMainControlTab::vpShowUpdate()
{
	m_oglWindow.viaShow();
}


void CMainControlTab::vpHideUpdate()
{
	// openGL via-point 객체에 저장된 경유점을 제거한다. 
	m_oglWindow.viaHide();
}

void CMainControlTab::vpListUpdate(int index)
{
	if(index < 0 || index > 1) {
		cout << "[vpListUpdate] wrong index!! " << endl;
		return;
	}

	// 리스트에 값 출력
	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];
	int i,j;
	
	for(j=0; j<theApp.T_SW->m_vp[index].GetNum(); j++)
	{
		// num 출력
		item.iItem = j;
		item.iSubItem = 0;
		item.pszText = itoa(j+1, nullBuffer, 10);
		for(i=0; i<DIM; i++)	// column 원소에 데이터 추가
		{
			item.iSubItem = i+1;
			item.pszText = gcvt(theApp.T_SW->m_vp[index].GetVal(j,i), 7, nullBuffer);
			// 0 이면 왼팔(기본팔), 1이면 오른팔
			if(index==0)
				m_vpList.SetItem(&item);
			else if(index==1)
				m_vpList2.SetItem(&item);
		}

		// Hash data 출력
		item.iSubItem = i+1;
		item.pszText = theApp.T_SW->m_vp[index].GetHash(j);

		if(index==0)
			m_vpList.SetItem(&item);
		else if(index==1)
			m_vpList2.SetItem(&item);
	}
}



// 시뮬레이터 내에 via point 나타내기
void CMainControlTab::OnBnClickedDrawviapoint()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	static BOOL isShow = FALSE;
	
	// Via-point Show
	if(!isShow) {
		vpShowUpdate();
		GetDlgItem(IDC_DRAWVIAPOINT)->SetWindowTextA("Hide via point");
		isShow = TRUE;
	}
	// Via-point Hide
	else {
		vpHideUpdate();	
		GetDlgItem(IDC_DRAWVIAPOINT)->SetWindowTextA("Show via point");
		isShow = FALSE;
	}
}


// 리스트 컨트롤에서 선택된 아이템 인덱스 가져오기
void CMainControlTab::OnLvnItemchangedViapointlist(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int index = -1;
	
	if(pNMLV->iItem >= 0 && pNMLV->iItem < m_vpList.GetItemCount())
	{
		index = pNMLV->iItem;
	}
	
	::SendDlgItemMessageA(m_hWnd, IDC_EDIT1, WM_KILLFOCUS,0,0);
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT1),SW_HIDE);
	
	theApp.T_SW->m_vp[0].SetIndex(index);
	m_oglWindow.selectedVP[0] = index;		// 선택된 리스트 아이템 설정

	*pResult = 0;
}


// 경유점 Playback!
void CMainControlTab::OnBnClickedPlayback()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	//Vector3d vpPos, vpOri;
	//
	//for(int j=0; j<1/*nVP*/; j++)
	//{		
	//	if(ViaPoint[j][0]==0.0f && ViaPoint[j][1]==0.0f && ViaPoint[j][2]==0.0f) break;
	//	for(int i=0; i<3; i++)
	//		vpPos(i) = ViaPoint[j][i];
	//	for(int i=0; i<3; i++)
	//		vpOri(i) = ViaPoint[j][i+3];	

	//	m_oglWindow.oglSetGoalPos(vpPos, vpOri);
	//	while(m_oglWindow.GetonProcess());
	//}

	////// playback을 위한 데이터 셋팅
	theApp.T_SW->bool_load = true;

	if(theApp.GetNumRobot() == 2){
		theApp.T_SW->event_time_R = theApp.T_SW->temp_time;
		theApp.T_SW->event_time_L = theApp.T_SW->temp_time;
		theApp.T_SW->event_q.resize(theApp.T_SW->m_robot[0].GetDOF());
		theApp.T_SW->event_q = theApp.T_SW->m_robot[0].GetQ();
		VectorXd temp_homeq(6);
		temp_homeq(0) = 0.0, temp_homeq(1) = 40.35, temp_homeq(2) = 97.83, temp_homeq(3) = 0.0, temp_homeq(4) = 19.92, temp_homeq(5) = 0.0;
		theApp.T_SW->home_q = H_DtoR * temp_homeq;

		theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data_R, theApp.T_SW->Teaching_data_j_R);
		theApp.T_SW->m_vp[1].Convert2Matrix(theApp.T_SW->Teaching_data_L, theApp.T_SW->Teaching_data_j_L);
		theApp.T_SW->Teaching_data_j_R *= H_DtoR;
		theApp.T_SW->Teaching_data_j_L *= H_DtoR;
	}
	else if(theApp.GetNumRobot() == 1){
		if(theApp.T_SW->m_vp[0].nToolcnt == 0){
			theApp.T_SW->m_vp[0].decode_hash_table();

			theApp.T_SW->event_time = theApp.T_SW->temp_time;
			theApp.T_SW->event_q.resize(theApp.T_SW->m_robot[0].GetDOF());
			theApp.T_SW->event_q = theApp.T_SW->m_robot[0].GetQ();
			VectorXd temp_homeq(6);
			temp_homeq(0) = 0.0, temp_homeq(1) = 40.35, temp_homeq(2) = 97.83, temp_homeq(3) = 0.0, temp_homeq(4) = 19.92, temp_homeq(5) = 0.0;
			theApp.T_SW->home_q = H_DtoR * temp_homeq;

			theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data, theApp.T_SW->Teaching_data_j);
			theApp.T_SW->Teaching_data_j *= H_DtoR;
			cout<<"Teaching_data_j"<<theApp.T_SW->Teaching_data_j<<endl;
			//getch();
		}
		else if(theApp.T_SW->m_vp[0].nToolcnt != 0){
			const char* path_hash = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash_j.txt";
			//theApp.T_SW->decode_hash_table(path_hash, theApp.T_SW->m_vp[0].hash_numindx, theApp.T_SW->m_vp[0].hash_gripindx, theApp.T_SW->m_vp[0].hash_releaseindx, 
			//	theApp.T_SW->m_vp[0].hash_gripindxV, theApp.T_SW->m_vp[0].hash_releaseindxV);
			theApp.T_SW->m_vp[0].decode_hash_table();

			theApp.T_SW->event_time = theApp.T_SW->temp_time;
			theApp.T_SW->event_q.resize(theApp.T_SW->m_robot[0].GetDOF());
			theApp.T_SW->event_q = theApp.T_SW->m_robot[0].GetQ();
			VectorXd temp_homeq(6);
			temp_homeq(0) = 0.0, temp_homeq(1) = 40.35, temp_homeq(2) = 97.83, temp_homeq(3) = 0.0, temp_homeq(4) = 19.92, temp_homeq(5) = 0.0;
			theApp.T_SW->home_q = H_DtoR * temp_homeq;

			theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data, theApp.T_SW->Teaching_data_j);
			theApp.T_SW->Teaching_data_j *= H_DtoR;

	
			/************************************************************************************************/
			/* Vision과 연동할 때 필요한 부분.. */
			//theApp.T_SW->bool_vision = false;
			if(theApp.T_SW->bool_vision) //vision 연결되어 있으면 playback 호출 순간에 snapshot 찍어야 함. 개선해야 할 부분..
				theApp.T_SW->Snapshot(theApp.T_SW->snpshot);

			/* IK loop */
			// 1. 상자의 정보를 얻어와서 순간적인 IK goalT 생성.
			Matrix4d _goalT_Gapproach, _goalT_Greach, _goalT_Gleave;
			Matrix4d _goalT_Rapproach, _goalT_Rreach, _goalT_Rleave;
			_goalT_Gapproach.setIdentity(), _goalT_Greach.setIdentity(), _goalT_Gleave.setIdentity();
			_goalT_Rapproach.setIdentity(), _goalT_Rreach.setIdentity(), _goalT_Rleave.setIdentity();
			for(int r=0; r<3; r++){
				for(int c=0; c<3; c++){
					//_goal_T(r, c) = GetRfrom_eXYZ(0.0, 0.0, snpshot(3))(r, c); // vision data에서는 yaw만 있음
					////// Vision 있을 때 없을 때.
					if(theApp.T_SW->bool_vision) //vision 연결되어 있으면 playback 호출 순간에 snapshot 찍어야 함. 개선해야 할 부분..
					{
						_goalT_Gapproach(r, c) = theApp.T_SW->GetRfrom_eXYZ(0.0, -180.0 * H_DtoR, theApp.T_SW->snpshot(3))(r, c); // redbox rz
						_goalT_Greach(r, c) = theApp.T_SW->GetRfrom_eXYZ(0.0, -180.0 * H_DtoR, theApp.T_SW->snpshot(3))(r, c); // redbox rz
						_goalT_Rapproach(r, c) = theApp.T_SW->GetRfrom_eXYZ(0.0, -180.0 * H_DtoR, theApp.T_SW->snpshot(15))(r, c); // yellowbox rz
						_goalT_Rreach(r, c) = theApp.T_SW->GetRfrom_eXYZ(0.0, -180.0 * H_DtoR, theApp.T_SW->snpshot(15))(r, c); // yellowbox rz
					}
					else
					{
						_goalT_Gapproach(r, c) = theApp.T_SW->GetRfrom_eXYZ(0.0, 158.10 * H_DtoR, 0.0)(r, c); 
						_goalT_Greach(r, c) = theApp.T_SW->GetRfrom_eXYZ(0.0, 158.10 * H_DtoR, 0.0)(r, c);

						_goalT_Rapproach(r, c) = theApp.T_SW->GetRfrom_eXYZ(0.0, 158.10 * H_DtoR, 0.0)(r, c); 
						_goalT_Rreach(r, c) = theApp.T_SW->GetRfrom_eXYZ(0.0, 158.10 * H_DtoR, 0.0)(r, c);
					}
				}
				//_goal_T(r, 3) = snpshot(r); //snapshot에는 RGBY순서로 저장되어 있으니 그 값을 맞게 쓰자. 약속된 상자순서 ㅠㅠ
				////// Vision 있을 때 없을 때.
				if(theApp.T_SW->bool_vision) //vision 연결되어 있으면 playback 호출 순간에 snapshot 찍어야 함. 개선해야 할 부분..
				{
					// redbox x y z 
					_goalT_Gapproach(0, 3) = theApp.T_SW->snpshot(0), _goalT_Gapproach(1, 3) = theApp.T_SW->snpshot(1), _goalT_Gapproach(2, 3) = theApp.T_SW->snpshot(2) + 20.0;
					_goalT_Greach(0, 3) = theApp.T_SW->snpshot(0), _goalT_Greach(1, 3) = theApp.T_SW->snpshot(1), _goalT_Greach(2, 3) = theApp.T_SW->snpshot(2);
					// yellowbox x y z 
					_goalT_Rapproach(0, 3) = theApp.T_SW->snpshot(12), _goalT_Rapproach(1, 3) = theApp.T_SW->snpshot(13), _goalT_Rapproach(2, 3) = theApp.T_SW->snpshot(14) + 20.0 + 30.0;
					_goalT_Rreach(0, 3) = theApp.T_SW->snpshot(12), _goalT_Rreach(1, 3) = theApp.T_SW->snpshot(13), _goalT_Rreach(2, 3) = theApp.T_SW->snpshot(14) + 30.0;
				}
				else
				{
					_goalT_Gapproach(0, 3) = 476.61, _goalT_Gapproach(1, 3) = 20.00, _goalT_Gapproach(2, 3) = -214.12;
					_goalT_Greach(0, 3) = 476.61, _goalT_Greach(1, 3) = 20.00, _goalT_Greach(2, 3) = -234.12;

					_goalT_Rapproach(0, 3) = 476.61, _goalT_Gapproach(1, 3) = 20.00, _goalT_Gapproach(2, 3) = -214.12;
					_goalT_Rreach(0, 3) = 476.61, _goalT_Greach(1, 3) = 20.00, _goalT_Greach(2, 3) = -234.12;
				}
			}
	
			// 2. IK loop를 위한 parameter 설정.
			bool bIKfin = false;
			while(!bIKfin){
				double pos_res = 10.0;
				double ori_res = 0.3;
				double pos_error = (_goalT_Gapproach.col(3).segment(0,3) - theApp.T_SW->m_robot[0].GetTool().col(3).segment(0,3)).norm();
				Vector3d ori_error;
				ori_error(0) = 1.0 - _goalT_Gapproach.col(0).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(0).segment(0,3));
				ori_error(1) = 1.0 - _goalT_Gapproach.col(1).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(1).segment(0,3));
				ori_error(2) = 1.0 - _goalT_Gapproach.col(2).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(2).segment(0,3));
				VectorXd IKdq(theApp.T_SW->m_robot[0].GetDOF());
				IKdq.setZero();
				IKdq = theApp.T_SW->IKLoop(&theApp.T_SW->m_robot[0], _goalT_Gapproach, pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);
				for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
					theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->m_robot[0].GetQ(i) + IKdq(i)); 
				if(bIKfin == true){
					//// approach 변수에 via point 저장.. 우선 joint space 값만..
					theApp.T_SW->m_vp[0].vp_Gapproach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					theApp.T_SW->m_vp[0].vp_Gapproach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK 다 풀고난 값을 목적지로 적용
					cout<<"IK for vision data approach to grip finished"<<theApp.T_SW->m_vp[0].vp_Gapproach_q<<endl;
					cout<<"**********************"<<endl;
				}
			}

			bIKfin = false;
			while(!bIKfin){
				double pos_res = 10.0;
				double ori_res = 0.3;
				double pos_error = (_goalT_Greach.col(3).segment(0,3) - theApp.T_SW->m_robot[0].GetTool().col(3).segment(0,3)).norm();
				Vector3d ori_error;
				ori_error(0) = 1.0 - _goalT_Greach.col(0).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(0).segment(0,3));
				ori_error(1) = 1.0 - _goalT_Greach.col(1).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(1).segment(0,3));
				ori_error(2) = 1.0 - _goalT_Greach.col(2).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(2).segment(0,3));
				VectorXd IKdq(theApp.T_SW->m_robot[0].GetDOF());
				IKdq.setZero();
				IKdq = theApp.T_SW->IKLoop(&theApp.T_SW->m_robot[0], _goalT_Greach, pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);
				for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
					theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->m_robot[0].GetQ(i) + IKdq(i)); 
				if(bIKfin == true){
					//// reach 변수에 via point 저장.. 우선 joint space 값만..
					theApp.T_SW->m_vp[0].vp_Greach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					theApp.T_SW->m_vp[0].vp_Greach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK 다 풀고난 값을 목적지로 적용
					cout<<"IK for vision data reach to grip finished"<<theApp.T_SW->m_vp[0].vp_Greach_q<<endl;
					cout<<"**********************"<<endl;
				}
			}

			bIKfin = false;
			while(!bIKfin){
				double pos_res = 10.0;
				double ori_res = 0.3;
				double pos_error = (_goalT_Rapproach.col(3).segment(0,3) - theApp.T_SW->m_robot[0].GetTool().col(3).segment(0,3)).norm();
				Vector3d ori_error;
				ori_error(0) = 1.0 - _goalT_Rapproach.col(0).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(0).segment(0,3));
				ori_error(1) = 1.0 - _goalT_Rapproach.col(1).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(1).segment(0,3));
				ori_error(2) = 1.0 - _goalT_Rapproach.col(2).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(2).segment(0,3));
				VectorXd IKdq(theApp.T_SW->m_robot[0].GetDOF());
				IKdq.setZero();
				IKdq = theApp.T_SW->IKLoop(&theApp.T_SW->m_robot[0], _goalT_Rapproach, pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);
				for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
					theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->m_robot[0].GetQ(i) + IKdq(i)); 
				if(bIKfin == true){
					//// reach 변수에 via point 저장.. 우선 joint space 값만..
					theApp.T_SW->m_vp[0].vp_Rapproach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					theApp.T_SW->m_vp[0].vp_Rapproach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK 다 풀고난 값을 목적지로 적용
					cout<<"IK for vision data reach to release finished"<<theApp.T_SW->m_vp[0].vp_Rapproach_q<<endl;
					cout<<"**********************"<<endl;
				}
			}

			bIKfin = false;
			while(!bIKfin){
				double pos_res = 10.0;
				double ori_res = 0.3;
				double pos_error = (_goalT_Rreach.col(3).segment(0,3) - theApp.T_SW->m_robot[0].GetTool().col(3).segment(0,3)).norm();
				Vector3d ori_error;
				ori_error(0) = 1.0 - _goalT_Rreach.col(0).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(0).segment(0,3));
				ori_error(1) = 1.0 - _goalT_Rreach.col(1).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(1).segment(0,3));
				ori_error(2) = 1.0 - _goalT_Rreach.col(2).segment(0,3).dot(theApp.T_SW->m_robot[0].GetTool().col(2).segment(0,3));
				VectorXd IKdq(theApp.T_SW->m_robot[0].GetDOF());
				IKdq.setZero();
				IKdq = theApp.T_SW->IKLoop(&theApp.T_SW->m_robot[0], _goalT_Rreach, pos_res, ori_res, pos_error, ori_error.norm(), bIKfin);
				for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
					theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->m_robot[0].GetQ(i) + IKdq(i)); 
				if(bIKfin == true){
					//// reach 변수에 via point 저장.. 우선 joint space 값만..
					theApp.T_SW->m_vp[0].vp_Rreach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					theApp.T_SW->m_vp[0].vp_Rreach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK 다 풀고난 값을 목적지로 적용
					cout<<"IK for vision data reach to release finished"<<theApp.T_SW->m_vp[0].vp_Rreach_q<<endl;
					cout<<"**********************"<<endl;
				}
			}

			//// vp_leave_q 는 vp_approach_q와 같다고 가정..
			theApp.T_SW->m_vp[0].vp_Gleave_q.resize(theApp.T_SW->m_robot[0].GetDOF());
			theApp.T_SW->m_vp[0].vp_Gleave_q = theApp.T_SW->m_vp[0].vp_Gapproach_q;

			theApp.T_SW->m_vp[0].vp_Rleave_q.resize(theApp.T_SW->m_robot[0].GetDOF());
			theApp.T_SW->m_vp[0].vp_Rleave_q = theApp.T_SW->m_vp[0].vp_Rapproach_q;
			/************************************************************************************************/
		}
	}

	theApp.T_SW->bool_load = false;
	theApp.T_SW->bool_Playback = true;
}




// Via-point 추가, Right Arm
void CMainControlTab::OnBnClickedAddvia()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.	
	
	int index = theApp.T_SW->m_vp[0].GetIndex()+1;	// 선택한 index의 아래에 값을 삽입 하기 위함
	theApp.T_SW->m_vp[0].InsertPoint(index);

	theApp.T_SW->m_vp[0].ShowList();

	// List 컨트롤 number 업데이트
	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];

	m_vpList.InsertItem(index, " ");
	
	for(int i=0; i<DIM; i++)	// column 원소 추가
	{	
		m_vpList.SetItem(index, i+1, LVIF_TEXT, gcvt(theApp.T_SW->m_vp[0].GetVal(index,i), 7, nullBuffer), NULL, NULL, NULL, NULL);
	}
	
	for(int j=0; j<theApp.T_SW->m_vp[0].GetNum(); j++)
	{
		// num 출력
		item.iItem = j;
		item.iSubItem = 0;
		item.pszText = itoa(j+1, nullBuffer, 10);
		m_vpList.SetItem(&item);
	}
	
	vpShowUpdate();
}


// Via-point 제거, Right Arm
void CMainControlTab::OnBnClickedDeletevia()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int selcount = m_vpList.GetSelectedCount();
	if(selcount <= 0) return;

	// 지우기전에 대화상자로 한번 더 물어봄
	if(IDYES == MessageBox("Are you Sure?", "Delete ViaPoint?",MB_YESNO)) {
		
		POSITION pos = m_vpList.GetFirstSelectedItemPosition();	// pos: 1-base

		for(int i=0; i<selcount; i++) {
			int temp = m_vpList.GetNextItem(-1, LVNI_SELECTED);
			int item_index = temp;	// index: 0-base
		
			theApp.T_SW->m_vp[0].DeleteArray(item_index);			// vp 객체에서 삭제
			m_vpList.DeleteItem(item_index);	// List control에서 삭제
		}
	
		// List 컨트롤 number 업데이트
		LVITEM item;
		item.mask = LVIF_TEXT;
		char nullBuffer[50];
	
		for(int j=0; j<theApp.T_SW->m_vp[0].GetNum(); j++)
		{
			// num 출력
			item.iItem = j;
			item.iSubItem = 0;
			item.pszText = itoa(j+1, nullBuffer, 10);
			m_vpList.SetItem(&item);
		}
	
		// View update
		vpShowUpdate();	
	}
}


// 위로 이동, Right Arm
void CMainControlTab::OnBnClickedUp()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_vpList.GetItemCount() > 1) {
		int index = theApp.T_SW->m_vp[0].GetIndex();
		
		// 2번째 줄 부터..
		if(index > 0 && theApp.T_SW->m_vp[0].SetChange(index, index-1)) {

			// 리스트에 값 출력
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
	
			for(int j=index-1; j<=index; j++)
			{
				// num 출력
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				//m_vpList.InsertItem(&item);	// 최상 목록 추가
				for(int i=0; i<DIM; i++)
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[0].GetVal(j,i), 7, nullBuffer);
					m_vpList.SetItem(&item); // 하위 목록 추가는 SetItem 사용
				}
			}

			// 기존의 선택된 파란선 해제
			m_vpList.SetSelectionMark(index);
			m_vpList.SetItemState(index, LVIS_FOCUSED, LVIS_SELECTED);

			// 새로운 아이템에 파란선 보여줌
			m_vpList.SetSelectionMark(index-1);
			m_vpList.SetItemState(index-1, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

			
			vpShowUpdate();

		}
		// Focus 설정
		m_vpList.SetFocus();
	}
}


// 아래로 이동, Right Arm
void CMainControlTab::OnBnClickedDn()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	if(m_vpList.GetItemCount() > 1) {
		int index = theApp.T_SW->m_vp[0].GetIndex();
		
		// 마지막 이전 줄 까지만
		if(index < theApp.T_SW->m_vp[0].GetNum() && theApp.T_SW->m_vp[0].SetChange(index, index+1)) {
			
			// 리스트에 값 출력
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
	
			for(int j=index; j<=index+1; j++)
			{
				// num 출력
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				for(int i=0; i<DIM; i++)
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[0].GetVal(j,i), 7, nullBuffer);
					m_vpList.SetItem(&item); // 하위 목록 추가는 SetItem 사용
				}
			}

			// 기존의 선택된 파란선 해제
			m_vpList.SetSelectionMark(index);
			m_vpList.SetItemState(index, LVIS_FOCUSED, LVIS_SELECTED);

			// 새로운 아이템에 파란선 보여줌
			m_vpList.SetSelectionMark(index+1);
			m_vpList.SetItemState(index+1, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

			vpShowUpdate();
		}
		// Focus 설정
		m_vpList.SetFocus();
	}
}



// 리스트 컨트롤 더블 클릭시 호출 됨. 에디트 창을 클릭한 해당 위치에서 호출하여 
// 리스트 컨트롤 값을 수정할 수 있게끔 하는 함수
void CMainControlTab::OnNMDblclkViapointlist(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	
	HWND hWnd1 = ::GetDlgItem(m_hWnd, IDC_VIAPOINTLIST);
	RECT rect;
	int nItem, nSubItem;
	ListclickY = nItem = pNMItemActivate->iItem;			// nRow
	ListclickX = nSubItem = pNMItemActivate->iSubItem;		// nColumn

	if(nItem < 0 || nSubItem == 0) return;

	CString str = m_vpList.GetItemText(nItem, nSubItem);
	
	RECT rect1, rect2;
	// this macro is used to retrieve the Rectangle of the selected SubItem
	ListView_GetSubItemRect(hWnd1, pNMItemActivate->iItem, pNMItemActivate->iSubItem, LVIR_BOUNDS, &rect);
	// Get the Rectangle of the ListControl
	::GetWindowRect(pNMItemActivate->hdr.hwndFrom, &rect1);
	// Get the Rectangle of the Dialog
	::GetWindowRect(m_hWnd, &rect2);

	int x = rect1.left - rect2.left;
	int y = rect1.top - rect2.top;

	if(nItem != -1)
		::SetWindowPos(::GetDlgItem(m_hWnd, IDC_EDIT1), HWND_TOP,
		rect.left+x, rect.top+y,
		rect.right-rect.left-3,
		rect.bottom - rect.top-1, NULL);
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT1), SW_SHOW);
	::SetFocus(::GetDlgItem(m_hWnd, IDC_EDIT1));

	//// Draw a Rectangle around the SubItem
	//::Rectangle(::GetDC(pNMItemActivate->hdr.hwndFrom), 
	//	rect.left, rect.top-1, rect.right, rect.bottom);
	
	// Set the listItem text in the EditBox
	::SetWindowTextA(::GetDlgItem(m_hWnd, IDC_EDIT1),str);


	*pResult = 0;
}


// Teach Here, Right Arm
void CMainControlTab::OnBnClickedTeachhere()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	ViaPoint *vp = &theApp.T_SW->m_vp[0];	// Right Arm

	if(vp->GetIndex() < 0) vp->SetIndex(0);	// List control을 선택 안했을 때..
	

	// vp mem 업로드
	cout << "button: " << theApp.T_SW->T_ND.temp_button << endl;
	if(theApp.T_SW->T_ND.temp_button == BTN_2_LONG) {	// close
		
		vp->SetRobotVP(vp->GetNum(), theApp.T_SW->m_robot[0], "Close");
	}
	else if(theApp.T_SW->T_ND.temp_button == BTN_2_SHORT) {	// open
		vp->SetRobotVP(vp->GetNum(), theApp.T_SW->m_robot[0], "Open");
	}else {
		char* ch;
		char nullBuffer[50];	
		ch = itoa(vp->GetNum()+1, nullBuffer, 10);
		vp->SetRobotVP(vp->GetNum(), theApp.T_SW->m_robot[0], ch);
	}
	vp->ShowList();


	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];

	// num 출력
	item.iItem = vp->GetNum();
	item.iSubItem = 0;
	item.pszText = itoa(vp->GetNum(), nullBuffer, 10);
	m_vpList.InsertItem(&item);	// 최상 목록 추가
	

	if(theApp.GetNumRobot() == 1){
		//theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data, theApp.T_SW->Teaching_data_j);
		//theApp.T_SW->Teaching_data_j *= H_DtoR;

		//const char* fname_hash_p = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash.txt";
		//const char* fname_hash_j = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash_j.txt";
		//theApp.T_SW->save_hash_table(fname_hash_p, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
		//theApp.T_SW->save_hash_table(fname_hash_j, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
	}
	else if(theApp.GetNumRobot() == 2){
		theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data_R, theApp.T_SW->Teaching_data_j_R);
		theApp.T_SW->m_vp[1].Convert2Matrix(theApp.T_SW->Teaching_data_L, theApp.T_SW->Teaching_data_j_L);
		theApp.T_SW->Teaching_data_j_R *= H_DtoR;
		theApp.T_SW->Teaching_data_j_L *= H_DtoR;

		cout<<"We have to progress to save hash table for dual arm!"<<endl;
	}
		
	// List control 업로드
	vpListUpdate(0);	
	
	// Simulator update
	vpShowUpdate();
	//theApp.T_SW->m_vp->ShowList();
}









//// -----------------------------------------------------------------------------------
//// -----------------------------------------------------------------------------------
//// -----------------------------------------------------------------------------------
//// -----------------------------------------------------------------------------------
//// Left Arm Via-point Teaching!!

// 위로 이동, Right Arm
void CMainControlTab::OnBnClickedUp2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_vpList.GetItemCount() > 1) {
		int index = theApp.T_SW->m_vp[1].GetIndex();
		
		// 2번째 줄 부터..
		if(index > 0 && theApp.T_SW->m_vp[1].SetChange(index, index-1)) {

			// 리스트에 값 출력
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
	
			for(int j=index-1; j<=index; j++)
			{
				// num 출력
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				//m_vpList.InsertItem(&item);	// 최상 목록 추가
				for(int i=0; i<DIM; i++)
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[1].GetVal(j,i), 7, nullBuffer);
					m_vpList2.SetItem(&item); // 하위 목록 추가는 SetItem 사용
				}
			}

			// 기존의 선택된 파란선 해제
			m_vpList2.SetSelectionMark(index);
			m_vpList2.SetItemState(index, LVIS_FOCUSED, LVIS_SELECTED);

			// 새로운 아이템에 파란선 보여줌
			m_vpList2.SetSelectionMark(index-1);
			m_vpList2.SetItemState(index-1, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

			
			vpShowUpdate();

		}
		// Focus 설정
		m_vpList2.SetFocus();
	}
}

// 아래로 이동, Left Arm
void CMainControlTab::OnBnClickedDn2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(m_vpList.GetItemCount() > 1) {
		int index = theApp.T_SW->m_vp[1].GetIndex();
		
		// 마지막 이전 줄 까지만
		if(index < theApp.T_SW->m_vp[1].GetNum() && theApp.T_SW->m_vp[1].SetChange(index, index+1)) {
			
			// 리스트에 값 출력
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
	
			for(int j=index; j<=index+1; j++)
			{
				// num 출력
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				for(int i=0; i<DIM; i++)
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[1].GetVal(j,i), 7, nullBuffer);
					m_vpList2.SetItem(&item); // 하위 목록 추가는 SetItem 사용
				}
			}

			// 기존의 선택된 파란선 해제
			m_vpList2.SetSelectionMark(index);
			m_vpList2.SetItemState(index, LVIS_FOCUSED, LVIS_SELECTED);

			// 새로운 아이템에 파란선 보여줌
			m_vpList2.SetSelectionMark(index+1);
			m_vpList2.SetItemState(index+1, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

			vpShowUpdate();
		}
		// Focus 설정
		m_vpList2.SetFocus();
	}
}


void CMainControlTab::OnBnClickedAddvia2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int index = theApp.T_SW->m_vp[1].GetIndex()+1;	// 선택한 index의 아래에 값을 삽입 하기 위함
	theApp.T_SW->m_vp[1].InsertPoint(index);

	theApp.T_SW->m_vp[1].ShowList();

	// List 컨트롤 number 업데이트
	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];

	m_vpList2.InsertItem(index, " ");
	
	for(int i=0; i<DIM; i++)	// column 원소 추가
	{	
		m_vpList2.SetItem(index, i+1, LVIF_TEXT, gcvt(theApp.T_SW->m_vp[1].GetVal(index,i), 7, nullBuffer), NULL, NULL, NULL, NULL);
	}
	
	for(int j=0; j<theApp.T_SW->m_vp[1].GetNum(); j++)
	{
		// num 출력
		item.iItem = j;
		item.iSubItem = 0;
		item.pszText = itoa(j+1, nullBuffer, 10);
		m_vpList2.SetItem(&item);
	}
	
	vpShowUpdate();
}


void CMainControlTab::OnBnClickedDeletevia2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int selcount = m_vpList2.GetSelectedCount();
	if(selcount <= 0) return;

	if(IDYES == MessageBox("Are you Sure?", "Delete ViaPoint?",MB_YESNO)) {

		POSITION pos = m_vpList2.GetFirstSelectedItemPosition();	// pos: 1-base

		for(int i=0; i<selcount; i++) {
			int temp = m_vpList2.GetNextItem(-1, LVNI_SELECTED);
			int item_index = temp;	// index: 0-base

			theApp.T_SW->m_vp[1].DeleteArray(item_index);			// vp 객체에서 삭제
			m_vpList2.DeleteItem(item_index);	// List control에서 삭제
		}
	
		// List 컨트롤 number 업데이트
		LVITEM item;
		item.mask = LVIF_TEXT;
		char nullBuffer[50];
	
		for(int j=0; j<theApp.T_SW->m_vp[1].GetNum(); j++)
		{
			// num 출력
			item.iItem = j;
			item.iSubItem = 0;
			item.pszText = itoa(j+1, nullBuffer, 10);
			m_vpList2.SetItem(&item);
		}
	
		// View update
		vpShowUpdate();
	}
}


void CMainControlTab::OnBnClickedTeachhere2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	ViaPoint *vp = &theApp.T_SW->m_vp[1];	// Right Arm

	if(vp->GetIndex() < 0) vp->SetIndex(0);	// List control을 선택 안했을 때..
	

	// vp mem 업로드
	vp->SetRobotVP(vp->GetNum(), theApp.T_SW->m_robot[1],"");
	vp->ShowList();	// TODO


	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];

	// num 출력
	item.iItem = vp->GetNum();
	item.iSubItem = 0;
	item.pszText = itoa(vp->GetNum(), nullBuffer, 10);
	m_vpList2.InsertItem(&item);	// 최상 목록 추가
	
	
	// List control 업로드
	vpListUpdate(1);	
	
	// Simulator update
	vpShowUpdate();
}

void CMainControlTab::OnLvnItemchangedViapointlist2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	int index = -1;
	
	if(pNMLV->iItem >= 0 && pNMLV->iItem < m_vpList2.GetItemCount())
	{
		index = pNMLV->iItem;
	}

	//cout << index << endl;
	

	::SendDlgItemMessageA(m_hWnd, IDC_EDIT2, WM_KILLFOCUS,0,0);
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT2),SW_HIDE);
	
	theApp.T_SW->m_vp[1].SetIndex(index);
	m_oglWindow.selectedVP[1] = index;		// 선택된 리스트 아이템 설정

	*pResult = 0;
}


void CMainControlTab::OnNMDblclkViapointlist2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	HWND hWnd1 = ::GetDlgItem(m_hWnd, IDC_VIAPOINTLIST2);
	RECT rect;
	int nItem, nSubItem;
	ListclickY = nItem = pNMItemActivate->iItem;			// nRow
	ListclickX = nSubItem = pNMItemActivate->iSubItem;		// nColumn

	if(nItem < 0 || nSubItem == 0) return;

	CString str = m_vpList2.GetItemText(nItem, nSubItem);
	
	RECT rect1, rect2;
	// this macro is used to retrieve the Rectangle of the selected SubItem
	ListView_GetSubItemRect(hWnd1, pNMItemActivate->iItem, pNMItemActivate->iSubItem, LVIR_BOUNDS, &rect);
	// Get the Rectangle of the ListControl
	::GetWindowRect(pNMItemActivate->hdr.hwndFrom, &rect1);
	// Get the Rectangle of the Dialog
	::GetWindowRect(m_hWnd, &rect2);

	int x = rect1.left - rect2.left;
	int y = rect1.top - rect2.top;

	if(nItem != -1)
		::SetWindowPos(::GetDlgItem(m_hWnd, IDC_EDIT2), HWND_TOP,
		rect.left+x, rect.top+y,
		rect.right-rect.left-3,
		rect.bottom - rect.top-1, NULL);
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT2), SW_SHOW);
	::SetFocus(::GetDlgItem(m_hWnd, IDC_EDIT2));

	//// Draw a Rectangle around the SubItem
	//::Rectangle(::GetDC(pNMItemActivate->hdr.hwndFrom), 
	//	rect.left, rect.top-1, rect.right, rect.bottom);
	
	// Set the listItem text in the EditBox
	::SetWindowTextA(::GetDlgItem(m_hWnd, IDC_EDIT2),str);

	*pResult = 0;
}


void CMainControlTab::OnBnClickedInitexo()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	
	if(!theApp.T_SW->m_exo.connect_stats) {
		theApp.T_SW->m_exo.Init_Exo(L"COM8");
		GetDlgItem(IDC_INITEXO)->SetWindowTextA("Diable_Exo");
		if ( pEXO_Thread == NULL){
			pEXO_Thread = AfxBeginThread(T_getExoData, this, 0, 0, CREATE_SUSPENDED);
			cout<<"Exo Thread function began"<<endl;
		}
		pEXO_Thread->ResumeThread();
	}
	// Via-point Hide
	else {
		pEXO_Thread->SuspendThread();
		cout<<"Exo Thread function suspended"<<endl;
		/*pEXO_Thread->ExitInstance();
		pEXO_Thread = NULL;
		theApp.T_SW->m_exo.Discnt_Exo();*/
		GetDlgItem(IDC_INITEXO)->SetWindowTextA("Init_Exo");
	}
}


void CMainControlTab::OnBnClickedInitomni()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(!theApp.T_SW->m_omni.connect_stats){
		if(theApp.T_SW->m_omni.Haptic_initialize()){
			GetDlgItem(IDC_INITOMNI)->SetWindowTextA("Diable_Omni");
			if ( pOMNI_Thread == NULL){
				pOMNI_Thread = AfxBeginThread(T_getOmniData, this, 0, 0, CREATE_SUSPENDED);
				cout<<"Omni Thread function began"<<endl;
			}
			pOMNI_Thread->ResumeThread();
			if(!theApp.T_SW->bool_Teleoperation)
				theApp.T_SW->bool_Teleoperation = true;
		}
	}
	else{
		pOMNI_Thread->SuspendThread();
		theApp.T_SW->m_omni.Haptic_finish();
		cout<<"Omni Thread function suspended"<<endl;
		/*pOMNI_Thread = NULL;
		theApp.T_SW->m_omni.Haptic_finish();
		if ( pOMNI_Thread == NULL)
			cout<<"Omni Thread function terminated"<<endl;*/ // terminate가 안되네..실수있음.
		
		if(theApp.T_SW->bool_Teleoperation)
			theApp.T_SW->bool_Teleoperation = false;
		
		GetDlgItem(IDC_INITOMNI)->SetWindowTextA("Init_Omni");
	}
}


void CMainControlTab::OnBnClickedTrajRecord()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	// Test , TODO
	cout << "hash record" << endl;
	char* hash = "CloseV";
	theApp.T_SW->m_vp[0].GetHash(0);
	theApp.T_SW->m_vp[0].SetHash(0, hash);

	vpListUpdate(0);
	vpShowUpdate();

	// Trajectories Record on/off
	if(theApp.GetNumRobot() == 1){
		if(theApp.T_SW->m_vp[0].onRecord) {
			cout << "Trajectory Record Off!! " << endl;
			// Record LED OFF (Gray)
			m_RecordLED.SubclassDlgItem(IDC_RECORD_LED, this);
			m_RecordLED.SetMode(LIGHT_USERDEFINED);
			m_RecordLED.SetColor(RGB(196,196,196));
			GetDlgItem(IDC_TRAJ_RECORD)->SetWindowTextA("Traj Record On");
			theApp.T_SW->m_vp[0].onRecord = false;
		}else {
			cout << "Trajectory Record On!! " << endl;
			// Record LED ON (Red)
			m_RecordLED.SubclassDlgItem(IDC_RECORD_LED, this);
			m_RecordLED.SetMode(LIGHT_USERDEFINED);
			m_RecordLED.SetColor(RGB(255,0,0));
			GetDlgItem(IDC_TRAJ_RECORD)->SetWindowTextA("Traj Record Off");
			theApp.T_SW->m_vp[0].onRecord = true;	
		}
	}
	
}


// 모든 via-point clear(왼팔)
void CMainControlTab::OnBnClickedClearvia()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	
	int num = theApp.T_SW->m_vp[0].GetNum();
	if(num <= 0) return;

	// 지우기전에 대화상자로 한번 더 물어봄
	if(IDYES == MessageBox("Are you Sure?", "Clear ViaPoint?",MB_YESNO)) {
		
		theApp.T_SW->m_vp[0].DeleteAllArray();
		m_vpList.DeleteAllItems();
	
		// View update
		vpShowUpdate();	
		theApp.T_SW->m_vp[0].SetIndex(-1);	// index 초기화
	}
}


// 모든 via-point clear(오른팔)
void CMainControlTab::OnBnClickedClearvia2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int num = theApp.T_SW->m_vp[1].GetNum();
	if(num <= 0) return;

	// 지우기전에 대화상자로 한번 더 물어봄
	if(IDYES == MessageBox("Are you Sure?", "Clear ViaPoint?",MB_YESNO)) {
		
		theApp.T_SW->m_vp[1].DeleteAllArray();
		m_vpList2.DeleteAllItems();
	
		// View update
		vpShowUpdate();	
		theApp.T_SW->m_vp[1].SetIndex(-1);	// index 초기화
	}
}


void CMainControlTab::OnBnClickedTopview()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_oglWindow.setTopView();
}


void CMainControlTab::OnBnClickedFrontview()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_oglWindow.setFrontView();
}


void CMainControlTab::OnBnClickedLeftview()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_oglWindow.setLeftView();
}


//// ---------------------------------------------------------
//// OpenGL View control
void CMainControlTab::OnBnClickedViewup()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_oglWindow.setViewUP();
}


void CMainControlTab::OnBnClickedViewdown()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_oglWindow.setViewDOWN();
}


void CMainControlTab::OnBnClickedViewleft()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_oglWindow.setViewLEFT();
}


void CMainControlTab::OnBnClickedViewright()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_oglWindow.setViewRIGHT();
}
