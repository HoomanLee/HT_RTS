// MainControlTab.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "HT_RTS.h"
#include "MainControlTab.h"
#include "afxdialogex.h"

#include "HT_RTSDlg.h"	// �θ� ������ Ŭ���� �߰�

#define MAXJOINT 9

#define RES_FONTSIZE	100
#define VALUE_FONTSIZE	100

CWinThread* pEXO_Thread = NULL;
CWinThread* pOMNI_Thread = NULL;
CWinThread* pCompute_Thread = NULL;


// CMainControlTab ��ȭ �����Դϴ�.

UINT CMainControlTab::T_Compute(LPVOID _lParam)
{
	COpenGLControl *pParam = (COpenGLControl*) _lParam;

	bool bIKfin = false;

	while(1)
	{
		/* �ð� ������Ʈ.. ���߿� update ��� ����� �ű⿡ �ֵ���.. */
		theApp.T_SW->temp_time = clock()/(double)CLOCKS_PER_SEC;

		if (pParam->onIKstart){
			//// omni���� �� ������.
			if(theApp.T_SW->m_omni.connect_stats){
				//HT_target�� omni thread�� �Բ� �����ϸ� �޸� �浹 �� ���� ������, omni thread���� �ƿ� ���� �����ع�����.
				//for(int r=0; r<3; r++){
				//	for(int c=0; c<3; c++)
				//		goal_T[0](r, c) = theApp.T_SW->m_omni.HT_target.linear()(r, c);
				//	goal_T[0](r, 3) = theApp.T_SW->m_omni.HT_target.translation()(r);
				//}
				/*if(theApp.T_SW->bool_Playback)
					theApp.T_SW->m_robot[0].SetQ(theApp.T_SW->des_q); */
			}
			//// exo���� �� ������.
			if(theApp.T_SW->m_exo.connect_stats){
				/* ���� �Ʒ� �ڵ忡�� �ߴµ�, �޸� �浹 �� ���� ������ exo thread���� �ƿ� ���� �ٲ������. */
				//vector <float> exo_data; //exo raw ������
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
			// ������ ���� ����. ������ ���� ���� �߰� ���! 
			if(theApp.GetNumRobot()==1) {	// �����϶�

				// TODO, ���� ������ ���� �ӽ� ������... -----------------------
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
					double max_dq = 0.1 * H_DtoR; // cout�� �ϸ� 1 * H_DtoR�� ������
					for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++){
						if (IKdq(i) > max_dq){
							//cout<<"dq '" <<i<<" ' exceeds"<<IKdq * H_RtoD<<endl;
							IKdq = max_dq * IKdq/IKdq.norm();
						}
					}

					/**************** write value to robots ***************************/
					//// Staubli�� ����Ǿ� ���� ������ staubli���� �����͸� ���;� sim�� real�� ����ȭ�� �ǰ���?
					//WaitForSingleObject(theApp.T_SW->m_omni.event_sequence, 4/*INFINITE*/); // event�� signaled ���°� �Ǳ⸦ ��ٸ���.

					if(theApp.T_SW->_tcpip.bConnected){ 
						/* send */
						theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
						/* receive */
						theApp.T_SW->_tcpip.RecvData(CONTAINER, theApp.T_SW->T_ND.recv_container); // ���� �� ���� ������ ������ �����͸� ����.		
						/* set Q for animate */
						for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++) {
							theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->T_ND.recv_container[i+6] * H_DtoR);
							// joint ������ �ӽ� ����
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
					////// Playback �� ��ü ���� �˰������� ������
					//theApp.T_SW->playback_controller_q(theApp.T_SW->m_robot[0].GetDOF(), theApp.T_SW->pb_stepper, 2.0, theApp.T_SW->temp_time
					//, theApp.T_SW->event_time, theApp.T_SW->home_q, theApp.T_SW->event_q, theApp.T_SW->m_robot[0].GetQ(), 
					//theApp.T_SW->Teaching_data_j, theApp.T_SW->des_q, 0.01 * H_DtoR, theApp.T_SW->bool_Playback);
					//// Playback�� Staubli�� �ñ�� sim�� Staubli data�� update
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


					//// Staubli�� ����Ǿ� ���� ������ staubli���� �����͸� ���;� sim�� real�� ����ȭ�� �ǰ���?
					//if(!theApp.T_SW->_tcpip.bConnected)						
					//	theApp.T_SW->m_robot[0].SetQ(theApp.T_SW->des_q); 
					
					/* Robot data ����.. �� �� thread���� container�� ���ÿ� �����ұ�� �� �ʿ��� �����.. ���߿��� read->compute->write�� �ٲ�߰��� */
					if(theApp.T_SW->_tcpip.bConnected) {
						/* send */
						/* playback�� ������ ���� ���������� joint ���� ������ */
						if(theApp.T_SW->T_ND.temp_pos_ori[6] == true){
							for(int i=0; i<6; i++)
								theApp.T_SW->T_ND.temp_pos_ori[i] = theApp.T_SW->des_q(i) * H_RtoD;
							theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
						}
						/* receive */
						theApp.T_SW->_tcpip.RecvData(CONTAINER, theApp.T_SW->T_ND.recv_container); // ���� �� ���� ������ ������ �����͸� ����.		
						/* set Q for animate */
						for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++) {
							theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->T_ND.recv_container[i+6] * H_DtoR);
							// joint ������ �ӽ� ����
							jVec(i) = theApp.T_SW->T_ND.recv_container[i+6];	// Degree
						}
					}
					else
						for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
							theApp.T_SW->m_robot[0].SetQ(theApp.T_SW->des_q);
				}
					
				// onProcessing�̶�� ������ GUI���� ���̱� ������ IK���� bIKfin�� ��ȯ�� �־� �̸� Ȱ�� ������ ����
				if(!bIKfin)
					pParam->onProcessing[0] = true;
				else
					pParam->onProcessing[0] = false;
				
				
				//// Matrix�� ����
				static int cnt_traj = 0;
				//printf("idx: %d, rows: %d, num: %d, cnt_traj: %d  \n", theApp.T_SW->m_vp[0].idx, theApp.T_SW->m_vp[0].traj.rows(), num, cnt_traj);
				if(theApp.T_SW->m_vp[0].onRecord == true)
				{
					
					// TODO
					//// Teaching�� playback�� �����ϱ� ���� �߰��� 0 ���͸� ����
					//jVec = theApp.T_SW->m_robot[0].GetQ();
					//theApp.T_SW->m_vp[0].traj.row(theApp.T_SW->m_vp[0].idx) = jVec;
					//theApp.T_SW->m_vp[0].idx++;
					////cout<<"theApp.T_SW->m_vp[0].traj.row(theApp.T_SW->m_vp[0].idx)"<<theApp.T_SW->m_vp[0].traj.row(theApp.T_SW->m_vp[0].idx)<<endl;

					if(cnt_traj > 100) cnt_traj = 0;
					cnt_traj++;
				}
				
					
			}
			else if(theApp.GetNumRobot()==2) { // �����϶� 
				
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
				
				// onProcessing�̶�� ������ GUI���� ���̱� ������ IK���� bIKfin�� ��ȯ�� �־� �̸� Ȱ�� ������ ����
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


		vector <float> exo_data; //exo raw ������
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

	Vector3d staubli_home; //���� ���� �� joint ���� {0,55.7,113.06,0,11.24,0}
	staubli_home(0) = 299.99;
	staubli_home(1) = 20.0;
	staubli_home(2) = -360.63;

	Vector3d val_FT;
	val_FT.setZero();

	//// ƼĪ ���� ������ ���� ��� ����.
	const char* fname_j = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_j.txt";
	const char* fname_j_edited = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_j_edited.txt";
	const char* fname_hash_j = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash_j.txt";
	const char* fname_p = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_p.txt";
	const char* fname_p_edited = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_p_edited.txt";
	const char* fname_hash_p = "F:/Staubli/CS8/Hooman7.6.1/usr/usrapp/STaubli_Teaching_hash.txt";
	
	//// hash���̺��� �ʱ�ȭ�ϱ� ���ؼ�. job file�� save �Լ� �ʿ��� �˾Ƽ� �ʱ�ȭ �ȴ�. 
	ofstream(fname_hash_j, ios::out);
	ofstream(fname_hash_p, ios::out);

	//playback loop ���� ���ؼ� �ð��� ���Ƿ� ����������.
	theApp.T_SW->temp_time = 0.0;



	while(1){
		
		//LARGE_INTEGER liCounter1, liCounter2, liFrequency; // �Լ��� �ɸ��� �ð� �����ϱ� ���ؼ�
		//QueryPerformanceFrequency(&liFrequency);  // retrieves the frequency of the high-resolution performance counter 

		//QueryPerformanceCounter(&liCounter1);         // Start

		theApp.T_SW->m_omni.btn_curclk = clock();
		
		theApp.T_SW->m_omni.getHapticPos(position_buffer, orientation_buffer, velocity_buffer);
		theApp.T_SW->m_omni.RHmapper(theApp.T_SW->m_omni.HT_target, Vel_target, staubli_home, position_buffer, orientation_buffer, velocity_buffer); //robot�� home position�� haptics�� inkwell�� ������
																		//�׷��� orientation ���߱�� ���� ����� ���� �ƴ�		
		
		if(theApp.T_SW->m_omni.BtnCmd.button1_prsd_time > 1.0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button1_prsd_time > 1.0)"<<endl;
			theApp.T_SW->T_ND.temp_button = BTN_1_LONG; //��ư �Է��� ���� �̰��� ����.
			
			if(theApp.T_SW->_tcpip.bConnected) //�ӽ÷�.. ������ �Ǿ����� check�ϴ� ����� robust�ϰ� ��������.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);
			
			theApp.T_SW->m_omni.BtnCmd.button1_prsd_time = 0; //��ư ���� �ð� �ʱ�ȭ�� �ݹ鿡�� �ϸ� �ʹ� ������ ���Ƽ� ���� ���� �� �̹� �ʱ�ȭ �Ǿ�����.

			pParam->OnBnClickedPlayback();
		}
		else if(theApp.T_SW->m_omni.BtnCmd.button1_prsd_time < 1.0 && theApp.T_SW->m_omni.BtnCmd.button1_prsd_time != 0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button1_prsd_time < 1.0)"<<endl;
			theApp.T_SW->T_ND.temp_button = BTN_1_SHORT; //��ư �Է��� ���� �̰��� ����.

			if(theApp.T_SW->_tcpip.bConnected) //�ӽ÷�.. ������ �Ǿ����� check�ϴ� ����� robust�ϰ� ��������.
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

			////* STL�� Teaching �ٲ� ��..  *////////
			pParam->OnBnClickedTeachhere(); //GUI�� visualize
			
			theApp.T_SW->m_vp[0].Convert2Matrix(theApp.T_SW->Teaching_data, theApp.T_SW->Teaching_data_j);
			theApp.T_SW->Teaching_data_j *= H_DtoR;
			
			theApp.T_SW->save_hash_table(fname_hash_p, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
			theApp.T_SW->save_hash_table(fname_hash_j, to_string((long double)theApp.T_SW->m_vp[0].GetNum()));
			////********************************////

			theApp.T_SW->m_omni.BtnCmd.button1_prsd_time = 0; //��ư ���� �ð� �ʱ�ȭ�� �ݹ鿡�� �ϸ� �ʹ� ������ ���Ƽ� ���� ���� �� �̹� �ʱ�ȭ �Ǿ�����.
		}
		// ��ư 2 : �׸��� ���� (ª�� ������ ����, ��� ������ close)
		else if(theApp.T_SW->m_omni.BtnCmd.button2_prsd_time > 1.0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button2_prsd_time > 1.0)"<<endl;
			theApp.T_SW->T_ND.temp_button = BTN_2_LONG; //��ư �Է��� ���� �̰��� ����.
			if(theApp.T_SW->_tcpip.bConnected) //�ӽ÷�.. ������ �Ǿ����� check�ϴ� ����� robust�ϰ� ��������.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);

			theApp.T_SW->save_hash_table(fname_hash_p, "Close");
			theApp.T_SW->save_hash_table(fname_hash_j, "Close");
			
			/* Gripper ���� count. playback �� �� �� ���� �� ����. */
			theApp.T_SW->m_vp[0].nToolcnt++;

			////// hash table visulaize ���ؼ� �ӽ÷�... //////
			pParam->OnBnClickedTeachhere();
			///////////////////////////////////////////////////

			////// �׸��۰� �������� ��, � ���ڿ� ���� �۵��ϴ� ������ �˱� ���� �˰���
			//double dist_to_Rbox = 0, dist_to_Gbox = 0, dist_to_Bbox = 0, dist_to_Ybox = 0;
			//for (int j = 0; j <3; j++){
			//	dist_to_Rbox += pow((double)(theApp.T_SW->m_omni.T_ND.recv_container[j] - theApp.T_SW->m_omni.snpshot[j]), 2);
			//	dist_to_Gbox += pow((double)(theApp.T_SW->m_omni.T_ND.recv_container[j] - theApp.T_SW->m_omni.snpshot[j+3]), 2);
			//	dist_to_Bbox += pow((double)(theApp.T_SW->m_omni.T_ND.recv_container[j] - theApp.T_SW->m_omni.snpshot[j+6]), 2);
			//	dist_to_Ybox += pow((double)(theApp.T_SW->m_omni.T_ND.recv_container[j] - theApp.T_SW->m_omni.snpshot[j+9]), 2);
			//}

			theApp.T_SW->m_omni.BtnCmd.button2_prsd_time = 0; //��ư ���� �ð� �ʱ�ȭ�� �ݹ鿡�� �ϸ� �ʹ� ������ ���Ƽ� ���� ���� �� �̹� �ʱ�ȭ �Ǿ�����.

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
			theApp.T_SW->T_ND.temp_button = BTN_2_SHORT; //��ư �Է��� ���� �̰��� ����.
			if(theApp.T_SW->_tcpip.bConnected) //�ӽ÷�.. ������ �Ǿ����� check�ϴ� ����� robust�ϰ� ��������.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);

			theApp.T_SW->save_hash_table(fname_hash_p, "Open");
			theApp.T_SW->save_hash_table(fname_hash_j, "Open");

			/* Gripper ���� count. playback �� �� �� ���� �� ����. */
			theApp.T_SW->m_vp[0].nToolcnt++;

			////// hash table visulaize ���ؼ� �ӽ÷�... //////
			pParam->OnBnClickedTeachhere();
			///////////////////////////////////////////////////


			theApp.T_SW->m_omni.BtnCmd.button2_prsd_time = 0; //��ư ���� �ð� �ʱ�ȭ�� �ݹ鿡�� �ϸ� �ʹ� ������ ���Ƽ� ���� ���� �� �̹� �ʱ�ȭ �Ǿ�����.
		}
		// ��ư �� �� ���ÿ� ª�� ������ ƼĪ ������ Ŭ����, ��� ������ ���� ��� ���� : �� ��, ��ư 3�� staubli �ʿ� �Ѱ��ִ� 3�� �򰥸��� ����.
		// �ٵ� ������ ��ư �ݹ� �Լ��δ� ��ư �� �� ���� �Ϳ� ���� �ν��� �� ���� �ʴ´�. 
		else if(theApp.T_SW->m_omni.BtnCmd.button3_prsd_time < 1.0 && theApp.T_SW->m_omni.BtnCmd.button3_prsd_time != 0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button3_prsd_time < 1.0)"<<endl;
			
			theApp.T_SW->Teaching_data.resize(0, 0);
			
			theApp.T_SW->T_ND.temp_button = 9; //��ư �Է��� ���� �̰��� ����.
			if(theApp.T_SW->_tcpip.bConnected) //�ӽ÷�.. ������ �Ǿ����� check�ϴ� ����� robust�ϰ� ��������.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);

			theApp.T_SW->m_omni.BtnCmd.button3_prsd_time = 0; //��ư ���� �ð� �ʱ�ȭ�� �ݹ鿡�� �ϸ� �ʹ� ������ ���Ƽ� ���� ���� �� �̹� �ʱ�ȭ �Ǿ�����.
		}
		else if(theApp.T_SW->m_omni.BtnCmd.button3_prsd_time > 1.0 && theApp.T_SW->m_omni.BtnCmd.button3_prsd_time != 0){
			cout<<"(theApp.T_SW->m_omni.BtnCmd.button3_prsd_time > 1.0)"<<endl;

			//cout<<"teaching continuous path"<<endl;
			theApp.T_SW->T_ND.temp_button = 5; //��ư �Է��� ���� �̰��� ����.
			if(theApp.T_SW->_tcpip.bConnected) //�ӽ÷�.. ������ �Ǿ����� check�ϴ� ����� robust�ϰ� ��������.
				theApp.T_SW->_tcpip.SendData_TBtnCMD(theApp.T_SW->T_ND.temp_button);

			theApp.T_SW->temp_cartesian_data.resize(6);
			for(int i = 0; i<6; i++)
				theApp.T_SW->temp_cartesian_data(i) = theApp.T_SW->T_ND.recv_container[i];//;theApp.T_SW->m_omni.T_ND.received_posori[i];
			theApp.T_SW->save_countinuous_path(false, "%n", theApp.T_SW->temp_cartesian_data, theApp.T_SW->Teaching_data);			
	
			cout<<theApp.T_SW->Teaching_data.rows()<<endl;

			theApp.T_SW->m_omni.BtnCmd.button3_prsd_time = 0; //��ư ���� �ð� �ʱ�ȭ�� �ݹ鿡�� �ϸ� �ʹ� ������ ���Ƽ� ���� ���� �� �̹� �ʱ�ȭ �Ǿ�����.
		}
		else{
			theApp.T_SW->T_ND.temp_button = 0; //��ư �Է��� ���� �̰��� ����.
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
			theApp.T_SW->T_ND.temp_pos_ori[i+3] = Ori_staubli.coeff(i) * H_RtoD;	//STaubli�� degree�� input���� �޴´�.	
		}
	
		theApp.T_SW->T_ND.temp_pos_ori[6] = theApp.T_SW->bool_Playback; // ������ element�� playback����

		//* For RTS simulator *//
		pParam->m_oglWindow.SetGoalPos(0, &theApp.T_SW->m_omni.HT_target);
		
		//SetEvent(theApp.T_SW->m_omni.event_sequence);

		//if(theApp.T_SW->_tcpip.bConnected){ 
		//	/* playback�� ������ ���� ���������� joint ���� ������ */
		//	if(theApp.T_SW->T_ND.temp_pos_ori[6] == true){
		//		for(int i=0; i<6; i++)
		//			theApp.T_SW->T_ND.temp_pos_ori[i] = theApp.T_SW->des_q(i) * H_RtoD;
		//		theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
		//	}
		//	else
		//		theApp.T_SW->_tcpip.SendData(POS_ORIDATA, theApp.T_SW->T_ND.temp_pos_ori);
		//	theApp.T_SW->_tcpip.RecvData(CONTAINER, theApp.T_SW->T_ND.recv_container); // ���� �� ���� ������ ������ �����͸� ����.		
		//}

		///* Robot data ����.. �� �� thread���� container�� ���ÿ� �����ұ�� �� �ʿ��� �����.. ���߿��� read->compute->write�� �ٲ�߰��� */
		//if(theApp.T_SW->_tcpip.bConnected)
		//	for(int i=0; i<theApp.T_SW->m_robot[0].GetDOF(); i++)
		//		theApp.T_SW->m_robot[0].SetQ(i, theApp.T_SW->T_ND.recv_container[i+6] * H_DtoR);

		Sleep(0);

		//SetEvent(theApp.T_SW->m_omni.event_sequence);          //Event �߻�
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

	// TODO:  ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.
	
	// --------------------------------------------------------
	// �ؽ�Ʈ ���â �ʱ�ȭ
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
	
	// �׿� ����Ʈ ��Ʈ���� �����. 
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

		// �׿� ����Ʈ ��Ʈ���� �����. 
		for(int i=nJoint; i<MAXJOINT; i++) {
			GetDlgItem(idJntLabel2[i])->ShowWindow(SW_HIDE);		// Joint Label
			GetDlgItem(idJntData2[i])->ShowWindow(SW_HIDE);		// Joint Data

			GetDlgItem(idJntBtnPlus2[i])->ShowWindow(SW_HIDE);	// plus button
			GetDlgItem(idJntBtnMinus2[i])->ShowWindow(SW_HIDE);	// minus button
			GetDlgItem(idJntDegree2[i])->ShowWindow(SW_HIDE);	// degree
		}
	}else {
		// �κ��� ���� ��� ��Ʈ���� �� �����.  

		// Via Point ��Ʈ�ѵ�
		for(int i=0; i<sizeof(idViaControl)/sizeof(int); i++) {
			GetDlgItem(idViaControl[i])->ShowWindow(SW_HIDE);
		}


		// Tool ��ǥ ��Ʈ�� �����
		for(int i=0; i<6; i++) {
			GetDlgItem(idToolLabel[i])->ShowWindow(SW_HIDE);
			GetDlgItem(idToolData[i])->ShowWindow(SW_HIDE);
			GetDlgItem(idToolBtnPlus[i])->ShowWindow(SW_HIDE);
			GetDlgItem(idToolBtnMinus[i])->ShowWindow(SW_HIDE);
			
		}
		// Tool etc �����
		for(int i=0; i<sizeof(idToolEtc)/sizeof(int); i++)
			GetDlgItem(idToolEtc[i])->ShowWindow(SW_HIDE);

		GetDlgItem(IDC_TOOLPOS_STEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SPIN_TPSTEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_TOOLORI_STEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SPIN_TOSTEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_LIGHT2)->ShowWindow(SW_HIDE);


		// Joint ��Ʈ�� �����
		for(int i=0; i<MAXJOINT; i++) {
			GetDlgItem(idJntLabel2[i])->ShowWindow(SW_HIDE);		// Joint Label
			GetDlgItem(idJntData2[i])->ShowWindow(SW_HIDE);		// Joint Data

			GetDlgItem(idJntBtnPlus2[i])->ShowWindow(SW_HIDE);	// plus button
			GetDlgItem(idJntBtnMinus2[i])->ShowWindow(SW_HIDE);	// minus button
			GetDlgItem(idJntDegree2[i])->ShowWindow(SW_HIDE);	// degree
		}

		// Joint etc �����
		GetDlgItem(IDC_STATIC_STEP)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_JOINT_STEP2)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SPIN_JSTEP2)->ShowWindow(SW_HIDE);
		
	}


	// --------------------------------------------------------
	// Sping Control �ʱ�ȭ

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
	
	//rect.SetRect(rect.left, rect.top, 800, 600);	// ��ġ �е� ũ�⿡ �°� �����غ���...

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
	lCol.mask = LVCF_FMT|LVCF_SUBITEM|LVCF_TEXT|LVCF_WIDTH;	// ����ü�� ����� Ȯ���� �÷��� ����
	lCol.fmt = LVCFMT_LEFT;

	for(int i=0; i<8; i++)
	{
		lCol.pszText = szText[i];			// �÷��� ������ ����
		lCol.iSubItem = i;					// ���� �������� �ε����� ����
		lCol.cx = nWid[i];					// �÷��� ���̸� ����
		m_vpList.InsertColumn(i, &lCol);	// LVCOLUMN����ü�� ������� ���� ���� ����Ʈ ��Ʈ�ѿ� ����
		m_vpList2.InsertColumn(i, &lCol);
	}
	m_vpList.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);	// List control style ����
	m_vpList2.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);	// List control style ����, 2
	

	// via-point class �ʱ�ȭ!!
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


	// Edit â �����
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT1),SW_HIDE);
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT2),SW_HIDE);


	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT1),SW_HIDE);


	// Picture Control window ũ�� ���
	CStatic *staticSize = (CStatic *)GetDlgItem(IDC_ROBOTSIMUL);
	CRect pc_rect;
	
	staticSize->GetClientRect(pc_rect);

	int width = pc_rect.Width();
	int height = pc_rect.Height();

	printf("Picture control: width : %d, height : %d \n", width, height);

	return TRUE;  // return TRUE unless you set the focus to a control
	// ����: OCX �Ӽ� �������� FALSE�� ��ȯ�ؾ� �մϴ�.
}



BOOL CMainControlTab::PreTranslateMessage(MSG* pMsg)
{
	// TODO: ���⿡ Ư��ȭ�� �ڵ带 �߰� ��/�Ǵ� �⺻ Ŭ������ ȣ���մϴ�.


	/*CWnd* pwndCtrl = GetFocus();
	int id = pwndCtrl->GetDlgCtrlID();
	printf("id: %d \n", id);*/
	
	if(pMsg->message == WM_KEYUP){
		m_oglWindow.m_ShiftOn=FALSE;
	}
		

	if(pMsg->message == WM_KEYDOWN || pMsg->message == WM_SYSKEYDOWN)
	{
		//printf("key down \n");
		// ������ if���� ���̾�α׿��� EnterŰ�� ESCŰ�� ���� ��
		// â�� ������ ���� �����ϱ� ���� ����
		if(pMsg->wParam == VK_ESCAPE)
		{
			return FALSE;
		}

		if(pMsg->wParam == VK_RETURN)
		{
			if(pMsg->hwnd == GetDlgItem(IDC_EDIT1)->GetSafeHwnd())
			{
				cout << "enter!!! " << endl;
				// �� ����
				CString str;
				GetDlgItemText(IDC_EDIT1, str);
				int count = str.GetLength();
				int i=0;


				if(ListclickX==7) {			// Hash table column
					
					// open, close, openv, closev������ üũ
					char* temp = theApp.T_SW->m_vp[0].GetHash(ListclickY);	// ���� hash table ��
					if((!strcmp(temp,"Open") || !strcmp(temp,"OpenV") || !strcmp(temp,"Close") || !strcmp(temp,"CloseV")) && temp) {
						char *hash = LPSTR(LPCTSTR(str));	// CString to char
						if(!strcmp(hash,"Open") || !strcmp(hash,"OpenV") || !strcmp(hash,"Close") || !strcmp(hash,"CloseV")) {
							theApp.T_SW->m_vp[0].SetHash(ListclickY, hash);
						}else {
						MessageBox("�߸��� �����Դϴ�. ");
						}
					}else {
						MessageBox("���ڸ� ������ �� �����ϴ�. ");
					}

				}else {						// ������ column
					for(i=0; i<count; i++)
					{
						char temp = str.GetAt(i);
					
						// ���� ó��
						if(i==0 && temp =='-') continue;
					
						// �Ҽ��� ó��
						if(temp == '.') continue;

						// �Էµ� Ű�� 0~9 ���������� üũ
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
						MessageBox("�߸��� �����Դϴ�. ");
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
				// �� ����
				CString str;
				GetDlgItemText(IDC_EDIT2, str);
				int count = str.GetLength();
				int i=0;

				for(i=0; i<count; i++)
				{
					char temp = str.GetAt(i);
					
					// ���� ó��
					if(i==0 && temp =='-') continue;
					
					// �Ҽ��� ó��
					if(temp == '.') continue;

					// �Էµ� Ű�� 0~9 ���������� üũ
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
					MessageBox("�߸��� �����Դϴ�. ");
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

		// Ư��Ű �˻�
		BOOL bShift = ((GetKeyState(VK_SHIFT) & 0x8000) != 0);	// Shift Ű �˻�
		BOOL bControl = ((GetKeyState(VK_CONTROL) & 0x8000) != 0); // Control Ű �˻�
		BOOL bAlt = ((GetKeyState(VK_LMENU) & 0x8000) != 0);		// Alt Ű �˻�

		if(!bControl && !bShift && bAlt)	// Alt�� ������ ��
			m_oglWindow.m_AltOn=TRUE;
		else if(!bControl && bShift && !bAlt) // Shift�� ������ ��
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	
	CString get_value, set_value;
	GetDlgItemTextA(IDC_WORLDPOS_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(��Ʈ�� ���� ������)
	m_WorldPosStep = atof(get_value);	// 10�� �������� double�� �Ǽ���..

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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_WORLDORI_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(��Ʈ�� ���� ������)
	m_WorldOriStep = atof(get_value);	// 10�� �������� double�� �Ǽ���..

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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.


	CString get_value, set_value;
	GetDlgItemTextA(IDC_JOINT_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(��Ʈ�� ���� ������)
	m_JointStep = atof(get_value);	// 10�� �������� double�� �Ǽ���..

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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_JOINT_STEP2, get_value);
		UpdateData(TRUE);	// control -> variable(��Ʈ�� ���� ������)
	m_JointStep2 = atof(get_value);	// 10�� �������� double�� �Ǽ���..

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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_TOOLPOS_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(��Ʈ�� ���� ������)
	m_ToolPosStep = atof(get_value);	// 10�� �������� double�� �Ǽ���..

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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	CString get_value, set_value;
	GetDlgItemTextA(IDC_TOOLORI_STEP, get_value);
		UpdateData(TRUE);	// control -> variable(��Ʈ�� ���� ������)
	m_ToolOriStep = atof(get_value);	// 10�� �������� double�� �Ǽ���..

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
// ���⼭���� OpenGL
void CMainControlTab::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰��մϴ�.
	// �׸��� �޽����� ���ؼ��� CDialogEx::OnPaint()��(��) ȣ������ ���ʽÿ�.

	//printf("onPaint\n");
	CDialog::OnPaint();

}


void CMainControlTab::OnSize(UINT nType, int cx, int cy)
{
	CDialogEx::OnSize(nType, cx, cy);

	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰��մϴ�.
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
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.

	// message �Լ��� �׷��� ������ ����Ƴ�.. ���� �ʹ� ���� ������ �׸�ŭ �ӵ��� �ȳ���. 

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
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	printf("Double Click \n");
	CDialogEx::OnLButtonDblClk(nFlags, point);
}



void CMainControlTab::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.
	
	//printf("Notification \n");
	/// â ���� �ƹ� ���̳� Ŭ���ص� ��Ŀ���� ����
	///////// �θ� Ŭ���� ������ ������
	CHT_RTSDlg *dlg = (CHT_RTSDlg*) AfxGetMainWnd();
	dlg->m_MainControlTab.SetFocus();

	CDialogEx::OnLButtonDown(nFlags, point);
}



// ���� ��ǥ �� ����Ʈ �� ���
void CMainControlTab::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: ���⿡ �޽��� ó���� �ڵ带 �߰� ��/�Ǵ� �⺻���� ȣ���մϴ�.

	switch(nIDEvent)
	{
		case 1:
		{
			// onProcess ���� ���
			if(m_oglWindow.GetonProcess(0)){
				// Red ����
				m_Light.SubclassDlgItem(IDC_LIGHT, this);
				m_Light.SetMode(LIGHT_USERDEFINED);
				m_Light.SetColor(RGB(255,0,0));
			}else {
				// Green ����
				m_Light.SubclassDlgItem(IDC_LIGHT, this);
				m_Light.SetMode(LIGHT_USERDEFINED);
				m_Light.SetColor(RGB(0,255,0));
			}

			// �κ� ���� �� ���� ���...
			if(theApp.GetNumRobot() > 1) {
				// onProcess ���� ���
				if(m_oglWindow.GetonProcess(1)){
					// Red ����
					m_Light2.SubclassDlgItem(IDC_LIGHT2, this);
					m_Light2.SetMode(LIGHT_USERDEFINED);
					m_Light2.SetColor(RGB(255,0,0));
				}else {
					// Green ����
					m_Light2.SubclassDlgItem(IDC_LIGHT2, this);
					m_Light2.SetMode(LIGHT_USERDEFINED);
					m_Light2.SetColor(RGB(0,255,0));
				}
			}
			

			// Joint 1 Coordinate��� -----------------------------
			CLargeText *JntData[] = {&m_Joint1, &m_Joint2, &m_Joint3, &m_Joint4, &m_Joint5, &m_Joint6, &m_Joint7, &m_Joint8, &m_Joint9};
			int dof = theApp.T_SW->m_robot[0].GetDOF();
			CString str;
			for(int i=0; i<dof; ++i)
			{
				str.Format("%.2f", theApp.T_SW->m_robot[0].GetQ(i)*RtoD);	// Radian to degree	
				JntData[i]->SetCaption(str);
			}


			// World Coordinate ��� -----------------------------
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


			// �κ� ���� �� ���� ���...
			if(theApp.GetNumRobot() > 1) {
				// Joint 2 Coordinate��� -----------------------------
				CLargeText *JntData2[] = {&m_Joint1_2, &m_Joint2_2, &m_Joint3_2, &m_Joint4_2, &m_Joint5_2, &m_Joint6_2, &m_Joint7_2, &m_Joint8_2, &m_Joint9_2};
				int dof = theApp.T_SW->m_robot[1].GetDOF();
				CString str;
				for(int i=0; i<dof; ++i)
				{
					str.Format("%.2f", theApp.T_SW->m_robot[1].GetQ(i)*RtoD);	// Radian to degree	
					JntData2[i]->SetCaption(str);
				}

				
				// Tool Coordinate ��� -----------------------------
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(0) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(0, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J1 down
void CMainControlTab::OnBnClickedJ1minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(0) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI){
		theApp.T_SW->m_robot[0].SetQ(0, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J2 up
void CMainControlTab::OnBnClickedJ2plus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(1) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(1, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J2 down
void CMainControlTab::OnBnClickedJ2minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(1) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(1, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J3 up
void CMainControlTab::OnBnClickedJ3plus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(2) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(2, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J3 down
void CMainControlTab::OnBnClickedJ3minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(2) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(2, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J4 up
void CMainControlTab::OnBnClickedJ4plus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(3) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(3, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J4 down
void CMainControlTab::OnBnClickedJ4minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(3) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(3, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J5 up
void CMainControlTab::OnBnClickedJ5plus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(4) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(4, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J5 down
void CMainControlTab::OnBnClickedJ5minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(4) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(4, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J6 up
void CMainControlTab::OnBnClickedJ6plus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(5) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(5, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J6 down
void CMainControlTab::OnBnClickedJ6minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(5) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(5, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J7 up
void CMainControlTab::OnBnClickedJ7plus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(6) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(6, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J7 down
void CMainControlTab::OnBnClickedJ7minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(6) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(6, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J8 up
void CMainControlTab::OnBnClickedJ8plus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(7) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(7, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J8 down
void CMainControlTab::OnBnClickedJ8minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(7) - m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(7, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J9 up
void CMainControlTab::OnBnClickedJ9plus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[0].GetQ(8) + m_JointStep*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[0].SetQ(8, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}

// J9 down
void CMainControlTab::OnBnClickedJ9minus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE){
		printf("X plus! \n");
		m_oglWindow.SetDesPosition(0,m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedXminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE){
		printf("X minus! \n");
		m_oglWindow.SetDesPosition(0,-m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedYplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("Y plus! \n");
		m_oglWindow.SetDesPosition(1,m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedYminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("Y minus! \n");
		m_oglWindow.SetDesPosition(1,-m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedZplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("Z plus! \n");
		m_oglWindow.SetDesPosition(2,m_WorldPosStep,0);
	}
}

void CMainControlTab::OnBnClickedZminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("Z minus! \n");
		m_oglWindow.SetDesPosition(2,-m_WorldPosStep,0);
	}
}




// ------------------------------------------------------------------------
// Cartesian Orientation Control
void CMainControlTab::OnBnClickedRxplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.	
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RX plus! \n");
		m_oglWindow.SetDesPosition(3,m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRxminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RX minus! \n");
		m_oglWindow.SetDesPosition(3,-m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRyplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RY plus! \n");
		m_oglWindow.SetDesPosition(4,m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRyminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RY minus! \n");
		m_oglWindow.SetDesPosition(4,-m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRzplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RZ plus! \n");
		m_oglWindow.SetDesPosition(5,m_WorldOriStep,0);
	}
}

void CMainControlTab::OnBnClickedRzminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(0)==FALSE) {
		printf("RZ minus! \n");
		m_oglWindow.SetDesPosition(5,-m_WorldOriStep,0);
	}
}







//// Tool Position Control--------------------
void CMainControlTab::OnBnClickedTxplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE){
		printf("X plus! \n");
		m_oglWindow.SetDesPosition(0,m_ToolPosStep,1);
	}
}

void CMainControlTab::OnBnClickedTxminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE){
		printf("X minus! \n");
		m_oglWindow.SetDesPosition(0,-m_ToolPosStep,1);
	}
}


void CMainControlTab::OnBnClickedTyplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("Y plus! \n");
		m_oglWindow.SetDesPosition(1,m_ToolPosStep,1);
	}
}


void CMainControlTab::OnBnClickedTyminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("Y minus! \n");
		m_oglWindow.SetDesPosition(1,-m_ToolPosStep,1);
	}
}


void CMainControlTab::OnBnClickedTzplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("Z plus! \n");
		m_oglWindow.SetDesPosition(2,m_ToolPosStep,1);
	}
}


void CMainControlTab::OnBnClickedTzminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("Z minus! \n");
		m_oglWindow.SetDesPosition(2,-m_ToolPosStep,1);
	}
}





//// Tool Orientation Control--------------------
void CMainControlTab::OnBnClickedTrxplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RX plus! \n");
		m_oglWindow.SetDesPosition(3,m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTrxminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RX minus! \n");
		m_oglWindow.SetDesPosition(3,-m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTryplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RY plus! \n");
		m_oglWindow.SetDesPosition(4,m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTryminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RY minus! \n");
		m_oglWindow.SetDesPosition(4,-m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTrzplus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RZ plus! \n");
		m_oglWindow.SetDesPosition(5,m_ToolOriStep,1);
	}
}

void CMainControlTab::OnBnClickedTrzminus()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_oglWindow.GetonProcess(1)==FALSE) {
		printf("RZ minus! \n");
		m_oglWindow.SetDesPosition(5,-m_ToolOriStep,1);
	}
}







// Joint 2 Control
void CMainControlTab::OnBnClickedJ1plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(0) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(0, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ1minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(0) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(0, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ2plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(1) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(1, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ2minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(1) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(1, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ3plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(2) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(2, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ3minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(2) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(2, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ4plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(3) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(3, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ4minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(3) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(3, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ5plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(4) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(4, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ5minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(4) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(4, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ6plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(5) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(5, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ6minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(5) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(5, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ7plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(6) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(6, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ7minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(6) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(6, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ8plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(7) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(7, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ8minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(7) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(7, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ9plus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(8) + m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(8, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}


void CMainControlTab::OnBnClickedJ9minus2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	float angle = theApp.T_SW->m_robot[1].GetQ(8) - m_JointStep2*DtoR;
	if(angle > -M_PI && angle < M_PI) {
		theApp.T_SW->m_robot[1].SetQ(8, angle);
		m_oglWindow.SetDesPositionByJoint();
	}
}






// Temp button function
void CMainControlTab::OnBnClickedOricheck()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	cout << "save trajectories" << endl;
	
	// text�� ����
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	// ���� �ε� �� ��ǥ ����
	char szFilter[] =  "All Files (*.*)|*.*|Text Files (*.txt)|*.txt|";
	char fileName[] = "Trajectories_01";

	// ���� ���� dialog ����
	CFileDialog dlg(FALSE, "txt", fileName, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | 
										OFN_NOCHANGEDIR, szFilter, NULL);
	
	dlg.m_ofn.nFilterIndex = 2;			// txt����

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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	// ���� �ε� �� ��ǥ ����
	char szFilter[] = "text files(*.txt)|*.txt|All Files(*.*)|*.*|";
	CFileDialog dlg(TRUE, NULL, NULL, OFN_HIDEREADONLY, szFilter);
	//CFileDialog dlg(TRUE, "*.txt", NULL, OFN_HIDEREADONLY, "text files(*.txt)|*.txt|", NULL);

	char line_temp[1024];
	
	FILE* fp;
	if(dlg.DoModal() == IDOK)
	{
		int nRobot = theApp.GetNumRobot();

		// ������ ���� �Ǿ� �ִ� ������ ���� �ʱ�ȭ
		theApp.T_SW->m_vp[0].SetClear();				// Data
		m_vpList.DeleteAllItems();	// List Control
		vpListUpdate(0);
		if(nRobot == 2) {
			theApp.T_SW->m_vp[1].SetClear();				// Data
			m_vpList2.DeleteAllItems();	// List Control
			vpListUpdate(1);
		}
		vpShowUpdate();				// OpenGL
		
		
		
		// ������ ���, Left, Right �� �� ��� ���� ���� �����ص� ���� �ϵ���..
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
					MessageBox("���� ���� ����");
					return;
				}

				UINT line_count = 0;
				while(!feof(fp))
				{
					memset(line_temp, NULL, 1024*sizeof(char));		// reset buffer
					fgets(line_temp,1024-5,fp);						// �� ���� �б�
					if(strlen(line_temp) < 2) break;				// line�� character�� 2�� ���� ������ ������ ���� ������ ����

					char value_temp[30] = {""};
					int dim = 12+theApp.T_SW->m_robot[order[n]].GetDOF();	// H matrix element + robot dof
					double *ar_val = new double[dim];
					UINT index = 0;
					UINT iDof = 0;

					// �� Line�� �о �迭�� ����
					for(int i=0; i<strlen(line_temp); i++)
					{
						if(line_temp[i] == 32)	// space ����(ASCii==32 space)
						{
							// �Ҽ��� ���� n��° �ڸ� ���� �ڸ���
							int cut = 3;
							ar_val[iDof] = ((int) (atof(value_temp)*pow(10.0,cut)))/pow(10.0,cut);	// space�� �������� ���ڸ� ���ڷ� �ٲپ� �迭�� ����(x, y, z, rx, ry, rz)
					
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

				// ����Ʈ�� �� ���, http://shaeod.tistory.com/251 (���ڿ��� ���ڿ��� ��ȯ ���� ��ũ)
				LVITEM item;
				item.mask = LVIF_TEXT;
				char nullBuffer[50];
				
				cout << "Line_count: " << line_count << endl;
				for(int j=0; j<line_count; j++)
				{
					// num ���
					item.iItem = j;
					item.iSubItem = 0;
					item.pszText = itoa(j+1, nullBuffer, 10);
					(order[n]==0) ? m_vpList.InsertItem(&item): m_vpList2.InsertItem(&item);	// �ֻ� ��� �߰�
					for(int i=0; i<DIM; i++)	// column ���� �߰�
					{
						item.iSubItem = i+1;
						item.pszText = gcvt(theApp.T_SW->m_vp[order[n]].GetVal(j,i), 7, nullBuffer);
						(order[n]==0) ? m_vpList.SetItem(&item) : m_vpList2.SetItem(&item); // ���� ��� �߰��� SetItem ���
					}
				}
			}
			
		



		// ������ ���
		}else if(nRobot == 1) {	
			CString fileName = dlg.GetFileName();
			fp = fopen(fileName.GetBuffer(100),"r");

			if(fp==NULL) {
				MessageBox("���� ���� ����");
				return;
			}
			UINT line_count = 0;

			while(!feof(fp))
			{
				memset(line_temp, NULL, 1024*sizeof(char));		// reset buffer
				fgets(line_temp,1024-5,fp);						// �� ���� �б�
				if(strlen(line_temp) < 2) break;				// line�� character�� 2�� ���� ������ ������ ���� ������ ����

				char value_temp[30] = {""};
				int dim = 12+theApp.T_SW->m_robot[0].GetDOF();	// H matrix element + robot dof
				double *ar_val = new double[dim];
				UINT index = 0;
				UINT iDof = 0;

				// �� Line�� �о �迭�� ����
				for(int i=0; i<strlen(line_temp); i++)
				{
					if(line_temp[i] == 32)	// space ����(ASCii==32 space)
					{
						// �Ҽ��� ���� n��° �ڸ� ���� �ڸ���
						int n = 6;
						ar_val[iDof] = ((int) (atof(value_temp)*pow(10.0,n)))/pow(10.0,n);	// space�� �������� ���ڸ� ���ڷ� �ٲپ� �迭�� ����(x, y, z, rx, ry, rz)
					
						// Push back���� �����ؾ��� ��!!
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
			
			// ����Ʈ�� �� ���, http://shaeod.tistory.com/251 (���ڿ��� ���ڿ��� ��ȯ ���� ��ũ)
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
				
			cout << "Line_count: " << line_count << endl;
			for(int j=0; j<line_count; j++)
			{
				// num ���
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				m_vpList.InsertItem(&item);
				for(int i=0; i<DIM; i++)	// column ���� �߰�
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	// ���� �ε� �� ��ǥ ����
	char szFilter[] =  "All Files (*.*)|*.*|Text Files (*.txt)|*.txt|";
	char fileName[] = "Via-Point01";
	CFileDialog dlg(FALSE, "txt", fileName, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | 
										OFN_NOCHANGEDIR, szFilter, NULL);

	dlg.m_ofn.nFilterIndex = 2;				// txt����
	int nRobot = theApp.GetNumRobot();	// ���� �κ�����, ���� �κ������� �ľ��ϱ� ����
	
	
	
	FILE *fp = NULL;
	if(dlg.DoModal() == IDOK)
	{
		if(nRobot == 2) {	// ������ ���
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

		}else {		// ������ ���
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






// ���� ����� �������� openGL via-point�� �����Ѵ�.
void CMainControlTab::vpShowUpdate()
{
	m_oglWindow.viaShow();
}


void CMainControlTab::vpHideUpdate()
{
	// openGL via-point ��ü�� ����� �������� �����Ѵ�. 
	m_oglWindow.viaHide();
}

void CMainControlTab::vpListUpdate(int index)
{
	if(index < 0 || index > 1) {
		cout << "[vpListUpdate] wrong index!! " << endl;
		return;
	}

	// ����Ʈ�� �� ���
	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];
	int i,j;
	
	for(j=0; j<theApp.T_SW->m_vp[index].GetNum(); j++)
	{
		// num ���
		item.iItem = j;
		item.iSubItem = 0;
		item.pszText = itoa(j+1, nullBuffer, 10);
		for(i=0; i<DIM; i++)	// column ���ҿ� ������ �߰�
		{
			item.iSubItem = i+1;
			item.pszText = gcvt(theApp.T_SW->m_vp[index].GetVal(j,i), 7, nullBuffer);
			// 0 �̸� ����(�⺻��), 1�̸� ������
			if(index==0)
				m_vpList.SetItem(&item);
			else if(index==1)
				m_vpList2.SetItem(&item);
		}

		// Hash data ���
		item.iSubItem = i+1;
		item.pszText = theApp.T_SW->m_vp[index].GetHash(j);

		if(index==0)
			m_vpList.SetItem(&item);
		else if(index==1)
			m_vpList2.SetItem(&item);
	}
}



// �ùķ����� ���� via point ��Ÿ����
void CMainControlTab::OnBnClickedDrawviapoint()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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


// ����Ʈ ��Ʈ�ѿ��� ���õ� ������ �ε��� ��������
void CMainControlTab::OnLvnItemchangedViapointlist(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int index = -1;
	
	if(pNMLV->iItem >= 0 && pNMLV->iItem < m_vpList.GetItemCount())
	{
		index = pNMLV->iItem;
	}
	
	::SendDlgItemMessageA(m_hWnd, IDC_EDIT1, WM_KILLFOCUS,0,0);
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT1),SW_HIDE);
	
	theApp.T_SW->m_vp[0].SetIndex(index);
	m_oglWindow.selectedVP[0] = index;		// ���õ� ����Ʈ ������ ����

	*pResult = 0;
}


// ������ Playback!
void CMainControlTab::OnBnClickedPlayback()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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

	////// playback�� ���� ������ ����
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
			/* Vision�� ������ �� �ʿ��� �κ�.. */
			//theApp.T_SW->bool_vision = false;
			if(theApp.T_SW->bool_vision) //vision ����Ǿ� ������ playback ȣ�� ������ snapshot ���� ��. �����ؾ� �� �κ�..
				theApp.T_SW->Snapshot(theApp.T_SW->snpshot);

			/* IK loop */
			// 1. ������ ������ ���ͼ� �������� IK goalT ����.
			Matrix4d _goalT_Gapproach, _goalT_Greach, _goalT_Gleave;
			Matrix4d _goalT_Rapproach, _goalT_Rreach, _goalT_Rleave;
			_goalT_Gapproach.setIdentity(), _goalT_Greach.setIdentity(), _goalT_Gleave.setIdentity();
			_goalT_Rapproach.setIdentity(), _goalT_Rreach.setIdentity(), _goalT_Rleave.setIdentity();
			for(int r=0; r<3; r++){
				for(int c=0; c<3; c++){
					//_goal_T(r, c) = GetRfrom_eXYZ(0.0, 0.0, snpshot(3))(r, c); // vision data������ yaw�� ����
					////// Vision ���� �� ���� ��.
					if(theApp.T_SW->bool_vision) //vision ����Ǿ� ������ playback ȣ�� ������ snapshot ���� ��. �����ؾ� �� �κ�..
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
				//_goal_T(r, 3) = snpshot(r); //snapshot���� RGBY������ ����Ǿ� ������ �� ���� �°� ����. ��ӵ� ���ڼ��� �Ф�
				////// Vision ���� �� ���� ��.
				if(theApp.T_SW->bool_vision) //vision ����Ǿ� ������ playback ȣ�� ������ snapshot ���� ��. �����ؾ� �� �κ�..
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
	
			// 2. IK loop�� ���� parameter ����.
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
					//// approach ������ via point ����.. �켱 joint space ����..
					theApp.T_SW->m_vp[0].vp_Gapproach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					theApp.T_SW->m_vp[0].vp_Gapproach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK �� Ǯ�� ���� �������� ����
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
					//// reach ������ via point ����.. �켱 joint space ����..
					theApp.T_SW->m_vp[0].vp_Greach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					theApp.T_SW->m_vp[0].vp_Greach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK �� Ǯ�� ���� �������� ����
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
					//// reach ������ via point ����.. �켱 joint space ����..
					theApp.T_SW->m_vp[0].vp_Rapproach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					theApp.T_SW->m_vp[0].vp_Rapproach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK �� Ǯ�� ���� �������� ����
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
					//// reach ������ via point ����.. �켱 joint space ����..
					theApp.T_SW->m_vp[0].vp_Rreach_q.resize(theApp.T_SW->m_robot[0].GetDOF()); 
					theApp.T_SW->m_vp[0].vp_Rreach_q = theApp.T_SW->m_robot[0].GetQ(); /// IK �� Ǯ�� ���� �������� ����
					cout<<"IK for vision data reach to release finished"<<theApp.T_SW->m_vp[0].vp_Rreach_q<<endl;
					cout<<"**********************"<<endl;
				}
			}

			//// vp_leave_q �� vp_approach_q�� ���ٰ� ����..
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




// Via-point �߰�, Right Arm
void CMainControlTab::OnBnClickedAddvia()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.	
	
	int index = theApp.T_SW->m_vp[0].GetIndex()+1;	// ������ index�� �Ʒ��� ���� ���� �ϱ� ����
	theApp.T_SW->m_vp[0].InsertPoint(index);

	theApp.T_SW->m_vp[0].ShowList();

	// List ��Ʈ�� number ������Ʈ
	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];

	m_vpList.InsertItem(index, " ");
	
	for(int i=0; i<DIM; i++)	// column ���� �߰�
	{	
		m_vpList.SetItem(index, i+1, LVIF_TEXT, gcvt(theApp.T_SW->m_vp[0].GetVal(index,i), 7, nullBuffer), NULL, NULL, NULL, NULL);
	}
	
	for(int j=0; j<theApp.T_SW->m_vp[0].GetNum(); j++)
	{
		// num ���
		item.iItem = j;
		item.iSubItem = 0;
		item.pszText = itoa(j+1, nullBuffer, 10);
		m_vpList.SetItem(&item);
	}
	
	vpShowUpdate();
}


// Via-point ����, Right Arm
void CMainControlTab::OnBnClickedDeletevia()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int selcount = m_vpList.GetSelectedCount();
	if(selcount <= 0) return;

	// ��������� ��ȭ���ڷ� �ѹ� �� ���
	if(IDYES == MessageBox("Are you Sure?", "Delete ViaPoint?",MB_YESNO)) {
		
		POSITION pos = m_vpList.GetFirstSelectedItemPosition();	// pos: 1-base

		for(int i=0; i<selcount; i++) {
			int temp = m_vpList.GetNextItem(-1, LVNI_SELECTED);
			int item_index = temp;	// index: 0-base
		
			theApp.T_SW->m_vp[0].DeleteArray(item_index);			// vp ��ü���� ����
			m_vpList.DeleteItem(item_index);	// List control���� ����
		}
	
		// List ��Ʈ�� number ������Ʈ
		LVITEM item;
		item.mask = LVIF_TEXT;
		char nullBuffer[50];
	
		for(int j=0; j<theApp.T_SW->m_vp[0].GetNum(); j++)
		{
			// num ���
			item.iItem = j;
			item.iSubItem = 0;
			item.pszText = itoa(j+1, nullBuffer, 10);
			m_vpList.SetItem(&item);
		}
	
		// View update
		vpShowUpdate();	
	}
}


// ���� �̵�, Right Arm
void CMainControlTab::OnBnClickedUp()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_vpList.GetItemCount() > 1) {
		int index = theApp.T_SW->m_vp[0].GetIndex();
		
		// 2��° �� ����..
		if(index > 0 && theApp.T_SW->m_vp[0].SetChange(index, index-1)) {

			// ����Ʈ�� �� ���
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
	
			for(int j=index-1; j<=index; j++)
			{
				// num ���
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				//m_vpList.InsertItem(&item);	// �ֻ� ��� �߰�
				for(int i=0; i<DIM; i++)
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[0].GetVal(j,i), 7, nullBuffer);
					m_vpList.SetItem(&item); // ���� ��� �߰��� SetItem ���
				}
			}

			// ������ ���õ� �Ķ��� ����
			m_vpList.SetSelectionMark(index);
			m_vpList.SetItemState(index, LVIS_FOCUSED, LVIS_SELECTED);

			// ���ο� �����ۿ� �Ķ��� ������
			m_vpList.SetSelectionMark(index-1);
			m_vpList.SetItemState(index-1, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

			
			vpShowUpdate();

		}
		// Focus ����
		m_vpList.SetFocus();
	}
}


// �Ʒ��� �̵�, Right Arm
void CMainControlTab::OnBnClickedDn()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	if(m_vpList.GetItemCount() > 1) {
		int index = theApp.T_SW->m_vp[0].GetIndex();
		
		// ������ ���� �� ������
		if(index < theApp.T_SW->m_vp[0].GetNum() && theApp.T_SW->m_vp[0].SetChange(index, index+1)) {
			
			// ����Ʈ�� �� ���
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
	
			for(int j=index; j<=index+1; j++)
			{
				// num ���
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				for(int i=0; i<DIM; i++)
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[0].GetVal(j,i), 7, nullBuffer);
					m_vpList.SetItem(&item); // ���� ��� �߰��� SetItem ���
				}
			}

			// ������ ���õ� �Ķ��� ����
			m_vpList.SetSelectionMark(index);
			m_vpList.SetItemState(index, LVIS_FOCUSED, LVIS_SELECTED);

			// ���ο� �����ۿ� �Ķ��� ������
			m_vpList.SetSelectionMark(index+1);
			m_vpList.SetItemState(index+1, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

			vpShowUpdate();
		}
		// Focus ����
		m_vpList.SetFocus();
	}
}



// ����Ʈ ��Ʈ�� ���� Ŭ���� ȣ�� ��. ����Ʈ â�� Ŭ���� �ش� ��ġ���� ȣ���Ͽ� 
// ����Ʈ ��Ʈ�� ���� ������ �� �ְԲ� �ϴ� �Լ�
void CMainControlTab::OnNMDblclkViapointlist(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	ViaPoint *vp = &theApp.T_SW->m_vp[0];	// Right Arm

	if(vp->GetIndex() < 0) vp->SetIndex(0);	// List control�� ���� ������ ��..
	

	// vp mem ���ε�
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

	// num ���
	item.iItem = vp->GetNum();
	item.iSubItem = 0;
	item.pszText = itoa(vp->GetNum(), nullBuffer, 10);
	m_vpList.InsertItem(&item);	// �ֻ� ��� �߰�
	

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
		
	// List control ���ε�
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

// ���� �̵�, Right Arm
void CMainControlTab::OnBnClickedUp2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_vpList.GetItemCount() > 1) {
		int index = theApp.T_SW->m_vp[1].GetIndex();
		
		// 2��° �� ����..
		if(index > 0 && theApp.T_SW->m_vp[1].SetChange(index, index-1)) {

			// ����Ʈ�� �� ���
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
	
			for(int j=index-1; j<=index; j++)
			{
				// num ���
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				//m_vpList.InsertItem(&item);	// �ֻ� ��� �߰�
				for(int i=0; i<DIM; i++)
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[1].GetVal(j,i), 7, nullBuffer);
					m_vpList2.SetItem(&item); // ���� ��� �߰��� SetItem ���
				}
			}

			// ������ ���õ� �Ķ��� ����
			m_vpList2.SetSelectionMark(index);
			m_vpList2.SetItemState(index, LVIS_FOCUSED, LVIS_SELECTED);

			// ���ο� �����ۿ� �Ķ��� ������
			m_vpList2.SetSelectionMark(index-1);
			m_vpList2.SetItemState(index-1, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

			
			vpShowUpdate();

		}
		// Focus ����
		m_vpList2.SetFocus();
	}
}

// �Ʒ��� �̵�, Left Arm
void CMainControlTab::OnBnClickedDn2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	if(m_vpList.GetItemCount() > 1) {
		int index = theApp.T_SW->m_vp[1].GetIndex();
		
		// ������ ���� �� ������
		if(index < theApp.T_SW->m_vp[1].GetNum() && theApp.T_SW->m_vp[1].SetChange(index, index+1)) {
			
			// ����Ʈ�� �� ���
			LVITEM item;
			item.mask = LVIF_TEXT;
			char nullBuffer[50];
	
			for(int j=index; j<=index+1; j++)
			{
				// num ���
				item.iItem = j;
				item.iSubItem = 0;
				item.pszText = itoa(j+1, nullBuffer, 10);
				for(int i=0; i<DIM; i++)
				{
					item.iSubItem = i+1;
					item.pszText = gcvt(theApp.T_SW->m_vp[1].GetVal(j,i), 7, nullBuffer);
					m_vpList2.SetItem(&item); // ���� ��� �߰��� SetItem ���
				}
			}

			// ������ ���õ� �Ķ��� ����
			m_vpList2.SetSelectionMark(index);
			m_vpList2.SetItemState(index, LVIS_FOCUSED, LVIS_SELECTED);

			// ���ο� �����ۿ� �Ķ��� ������
			m_vpList2.SetSelectionMark(index+1);
			m_vpList2.SetItemState(index+1, LVIS_SELECTED | LVIS_FOCUSED, LVIS_SELECTED | LVIS_FOCUSED);

			vpShowUpdate();
		}
		// Focus ����
		m_vpList2.SetFocus();
	}
}


void CMainControlTab::OnBnClickedAddvia2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int index = theApp.T_SW->m_vp[1].GetIndex()+1;	// ������ index�� �Ʒ��� ���� ���� �ϱ� ����
	theApp.T_SW->m_vp[1].InsertPoint(index);

	theApp.T_SW->m_vp[1].ShowList();

	// List ��Ʈ�� number ������Ʈ
	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];

	m_vpList2.InsertItem(index, " ");
	
	for(int i=0; i<DIM; i++)	// column ���� �߰�
	{	
		m_vpList2.SetItem(index, i+1, LVIF_TEXT, gcvt(theApp.T_SW->m_vp[1].GetVal(index,i), 7, nullBuffer), NULL, NULL, NULL, NULL);
	}
	
	for(int j=0; j<theApp.T_SW->m_vp[1].GetNum(); j++)
	{
		// num ���
		item.iItem = j;
		item.iSubItem = 0;
		item.pszText = itoa(j+1, nullBuffer, 10);
		m_vpList2.SetItem(&item);
	}
	
	vpShowUpdate();
}


void CMainControlTab::OnBnClickedDeletevia2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int selcount = m_vpList2.GetSelectedCount();
	if(selcount <= 0) return;

	if(IDYES == MessageBox("Are you Sure?", "Delete ViaPoint?",MB_YESNO)) {

		POSITION pos = m_vpList2.GetFirstSelectedItemPosition();	// pos: 1-base

		for(int i=0; i<selcount; i++) {
			int temp = m_vpList2.GetNextItem(-1, LVNI_SELECTED);
			int item_index = temp;	// index: 0-base

			theApp.T_SW->m_vp[1].DeleteArray(item_index);			// vp ��ü���� ����
			m_vpList2.DeleteItem(item_index);	// List control���� ����
		}
	
		// List ��Ʈ�� number ������Ʈ
		LVITEM item;
		item.mask = LVIF_TEXT;
		char nullBuffer[50];
	
		for(int j=0; j<theApp.T_SW->m_vp[1].GetNum(); j++)
		{
			// num ���
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	ViaPoint *vp = &theApp.T_SW->m_vp[1];	// Right Arm

	if(vp->GetIndex() < 0) vp->SetIndex(0);	// List control�� ���� ������ ��..
	

	// vp mem ���ε�
	vp->SetRobotVP(vp->GetNum(), theApp.T_SW->m_robot[1],"");
	vp->ShowList();	// TODO


	LVITEM item;
	item.mask = LVIF_TEXT;
	char nullBuffer[50];

	// num ���
	item.iItem = vp->GetNum();
	item.iSubItem = 0;
	item.pszText = itoa(vp->GetNum(), nullBuffer, 10);
	m_vpList2.InsertItem(&item);	// �ֻ� ��� �߰�
	
	
	// List control ���ε�
	vpListUpdate(1);	
	
	// Simulator update
	vpShowUpdate();
}

void CMainControlTab::OnLvnItemchangedViapointlist2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	int index = -1;
	
	if(pNMLV->iItem >= 0 && pNMLV->iItem < m_vpList2.GetItemCount())
	{
		index = pNMLV->iItem;
	}

	//cout << index << endl;
	

	::SendDlgItemMessageA(m_hWnd, IDC_EDIT2, WM_KILLFOCUS,0,0);
	::ShowWindow(::GetDlgItem(m_hWnd, IDC_EDIT2),SW_HIDE);
	
	theApp.T_SW->m_vp[1].SetIndex(index);
	m_oglWindow.selectedVP[1] = index;		// ���õ� ����Ʈ ������ ����

	*pResult = 0;
}


void CMainControlTab::OnNMDblclkViapointlist2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	
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
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
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
			cout<<"Omni Thread function terminated"<<endl;*/ // terminate�� �ȵǳ�..�Ǽ�����.
		
		if(theApp.T_SW->bool_Teleoperation)
			theApp.T_SW->bool_Teleoperation = false;
		
		GetDlgItem(IDC_INITOMNI)->SetWindowTextA("Init_Omni");
	}
}


void CMainControlTab::OnBnClickedTrajRecord()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

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


// ��� via-point clear(����)
void CMainControlTab::OnBnClickedClearvia()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	
	int num = theApp.T_SW->m_vp[0].GetNum();
	if(num <= 0) return;

	// ��������� ��ȭ���ڷ� �ѹ� �� ���
	if(IDYES == MessageBox("Are you Sure?", "Clear ViaPoint?",MB_YESNO)) {
		
		theApp.T_SW->m_vp[0].DeleteAllArray();
		m_vpList.DeleteAllItems();
	
		// View update
		vpShowUpdate();	
		theApp.T_SW->m_vp[0].SetIndex(-1);	// index �ʱ�ȭ
	}
}


// ��� via-point clear(������)
void CMainControlTab::OnBnClickedClearvia2()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	int num = theApp.T_SW->m_vp[1].GetNum();
	if(num <= 0) return;

	// ��������� ��ȭ���ڷ� �ѹ� �� ���
	if(IDYES == MessageBox("Are you Sure?", "Clear ViaPoint?",MB_YESNO)) {
		
		theApp.T_SW->m_vp[1].DeleteAllArray();
		m_vpList2.DeleteAllItems();
	
		// View update
		vpShowUpdate();	
		theApp.T_SW->m_vp[1].SetIndex(-1);	// index �ʱ�ȭ
	}
}


void CMainControlTab::OnBnClickedTopview()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_oglWindow.setTopView();
}


void CMainControlTab::OnBnClickedFrontview()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_oglWindow.setFrontView();
}


void CMainControlTab::OnBnClickedLeftview()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_oglWindow.setLeftView();
}


//// ---------------------------------------------------------
//// OpenGL View control
void CMainControlTab::OnBnClickedViewup()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_oglWindow.setViewUP();
}


void CMainControlTab::OnBnClickedViewdown()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_oglWindow.setViewDOWN();
}


void CMainControlTab::OnBnClickedViewleft()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_oglWindow.setViewLEFT();
}


void CMainControlTab::OnBnClickedViewright()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_oglWindow.setViewRIGHT();
}
