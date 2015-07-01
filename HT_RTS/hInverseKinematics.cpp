#include "hInverseKinematics.h"

int joint =7;
double DH_alpha[7] = {M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, 0};
double DH_a[7] = {145, 0, 0, 0, 0, 0, 0};
double DH_d[7] = {555, 0, 839, 0, 627, 0, 356};
double Initial_theta[7] = {0, M_PI/2+M_PI/2, 0, 0+M_PI/2, 0, -M_PI/2, 0};

double tmp_P[6];

hMatrix Cur_Robot_ROTATION(3,3), Next_Robot_ROTATION(3,3);
hMatrix Cur_Robot_POSITION(3,1), Next_Robot_POSITION(3,1);

double Pre_R3 = 0, Cur_R3 = 0, R3 = 0;

///////////////// 역기구학을 위한 함수 ///////////////////////////////
int Inv_solution(){
	int cnt = 0;
	hMatrix Goal_T(4,4),Initial_T(4,4);
	Next_Robot_ROTATION = Cur_Robot_ROTATION*Tool_Position(tmp_P[3], tmp_P[4], tmp_P[5]);
	double Next_R_POS[3] = {Cur_Robot_POSITION.element(0,0) + tmp_P[0], Cur_Robot_POSITION.element(1,0)+tmp_P[1], Cur_Robot_POSITION.element(2,0)+tmp_P[2]};
	Next_Robot_POSITION.SET(3,1,&Next_R_POS[0]);
	double Robot_Goal_T[16] = {Next_Robot_ROTATION.element(0,0), Next_Robot_ROTATION.element(0,1), Next_Robot_ROTATION.element(0,2), Next_Robot_POSITION.element(0,0),
							   Next_Robot_ROTATION.element(1,0), Next_Robot_ROTATION.element(1,1), Next_Robot_ROTATION.element(1,2), Next_Robot_POSITION.element(1,0),
							   Next_Robot_ROTATION.element(2,0), Next_Robot_ROTATION.element(2,1), Next_Robot_ROTATION.element(2,2), Next_Robot_POSITION.element(2,0),
							   0, 0, 0, 1};

	Goal_T.SET(4,4,&Robot_Goal_T[0]);
	
	
	double error_position[3]= {Goal_T.element(0,3)-Initial_T.element(0,3),Goal_T.element(1,3)-Initial_T.element(1,3),
		Goal_T.element(2,3)-Initial_T.element(2,3)};

	double norm= 10;
	while(norm > 0.001){
		hMatrix Solution(7,1);
		Initial_T = T_hMatrix(&Initial_theta[0], &DH_alpha[0], &DH_a[0], &DH_d[0], joint);
		for (int i=0; i<3; i++){
			error_position[i] = (Goal_T.element(i,3)-Initial_T.element(i,3)); 
		}
		norm = sqrt((error_position[0])*(error_position[0])+(error_position[1])*(error_position[1])+
			(error_position[2])*(error_position[2]));
		Solution = Inverse_Kinematics(Initial_T, Goal_T, &Initial_theta[0], &DH_alpha[0], &DH_a[0], &DH_d[0], joint);
		//Solution.displayhMatrix();
		for(int i=0; i<joint;i++)
			Initial_theta[i] = Solution.element(i,0);
	}
	return 0;
}

hMatrix Inverse_Kinematics(hMatrix Initial_T,hMatrix Goal_T,double *Initial_t, double *DH_alpha, double *DH_a, double *DH_d, int joint){

	for(int i=0; i<joint; i++){
		Initial_theta[i] = *Initial_t;
		Initial_t++;
	}

	hMatrix Initial_Theta(7,1);
	hMatrix J(6,7), Pinv_J(7,6);
	hMatrix n_a(3,1),s_a(3,1),a_a(3,1),n_t(3,1),s_t(3,1),a_t(3,1),p_del(3,1);
	double x,y,z,rx,ry,rz;
	double error_position[3]= {Goal_T.element(0,3)-Initial_T.element(0,3),Goal_T.element(1,3)-Initial_T.element(1,3),Goal_T.element(2,3)-Initial_T.element(2,3)};
	hMatrix P(3,1),R(3,1),Rotation(3,3),dx_temp1(3,1),dx_temp2(3,1),dX(6,1),del_Theta(7,1),Temp(7,1);


	Initial_Theta.SET(7,1,Initial_theta);

			Initial_T = T_hMatrix(&Initial_theta[0], &DH_alpha[0], &DH_a[0], &DH_d[0], joint);
			J = Jacobian_hMatrix(&Initial_theta[0], &DH_alpha[0], &DH_a[0], &DH_d[0]);
			Pinv_J = Pseudo_Inverse(J);

			for(int i = 0; i<3; i++){
				n_a.SetElement(i,0,Initial_T.element(i,0));
				s_a.SetElement(i,0,Initial_T.element(i,1));
				a_a.SetElement(i,0,Initial_T.element(i,2));
				n_t.SetElement(i,0,Goal_T.element(i,0));
				s_t.SetElement(i,0,Goal_T.element(i,1));
				a_t.SetElement(i,0,Goal_T.element(i,2));
				p_del.SetElement(i,0,Goal_T.element(i,3)-Initial_T.element(i,3));
			}
			
			x = dot(n_a, p_del); 
			y = dot(s_a, p_del); 
			z = dot(a_a, p_del); ;
			rx = (dot(a_a,s_t)-dot(a_t,s_a))/2;
			ry = (dot(n_a,a_t)-dot(n_t,a_a))/2;
			rz = (dot(s_a,n_t)-dot(s_t,n_a))/2;

			double dx_P[3] = {x,y,z},dx_R[3] = {rx,ry,rz};

			P.SET(3,1,&dx_P[0]);
			R.SET(3,1,&dx_R[0]);

			Rotation = T_Rotation(Initial_T);
			dx_temp1 = Rotation*P;
			dx_temp2 = Rotation*R;

			for(int i =0; i<3; i++){
				dX.SetElement(i,0,dx_temp1.element(i,0));
				dX.SetElement(i+3,0,dx_temp2.element(i,0));
			}
			
			del_Theta = Pinv_J*dX;

			for(int i=0; i<joint; i++)
				Temp.SetElement(i,0,Initial_Theta.element(i,0) + del_Theta.element(i,0));
			Initial_Theta = Temp;
	
	return Initial_Theta;
}

hMatrix A_hMatrix(double theta, double alpha, double a, double d){
	
	double value[16] = {cos(theta),  -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta),
      sin(theta),  cos(alpha)*cos(theta),  -sin(alpha)*cos(theta),  a*sin(theta),
      0,           sin(alpha),             cos(alpha),              d,
	  0,           0,                      0,                       1};

	hMatrix A(4,4);
	A.SET(4,4,&value[0]);

return A;
}

hMatrix T_hMatrix(double *theta, double *alpha, double *a, double *d, int joint){
	
	double Theta[7], Alpha[7], A[7], D[7];
	for(int i=0; i<joint; i++){
		Theta[i] = *theta;
		Alpha[i] = *alpha;
		A[i] = *a;
		D[i] = *d;
		theta++;
		alpha++;
		a++;
		d++;
	}

	hMatrix Temp1(4,4), T(4,4);
	double value[16] = {1,0,0,0,
						0,1,0,0,
						0,0,1,0,
						0,0,0,1};
	T.SET(4,4,&value[0]);	
	for(int i=0; i<joint; i++){
		Temp1 = A_hMatrix(Theta[i],Alpha[i],A[i],D[i]);
		T = T*Temp1;
	}
	
return T;
}

hMatrix Jacobian_hMatrix(double *theta, double *alpha, double *a, double *d) {
	hMatrix T01(4,4), T02(4,4), T03(4,4), T04(4,4), T05(4,4), T06(4,4), T07(4,4);	
	T01 = T_hMatrix(&theta[0], &alpha[0], &a[0], &d[0], 1);
	T02 = T_hMatrix(&theta[0], &alpha[0], &a[0], &d[0], 2);
	T03 = T_hMatrix(&theta[0], &alpha[0], &a[0], &d[0], 3);
	T04 = T_hMatrix(&theta[0], &alpha[0], &a[0], &d[0], 4);
	T05 = T_hMatrix(&theta[0], &alpha[0], &a[0], &d[0], 5);
	T06 = T_hMatrix(&theta[0], &alpha[0], &a[0], &d[0], 6);
	T07 = T_hMatrix(&theta[0], &alpha[0], &a[0], &d[0], 7);

	double k[3] = {0,0,1};
	double z1[3] = { T01.element(0,2),T01.element(1,2), T01.element(2,2)};
	double z2[3] = { T02.element(0,2),T02.element(1,2), T02.element(2,2)};
	double z3[3] = { T03.element(0,2),T03.element(1,2), T03.element(2,2)};
	double z4[3] = { T04.element(0,2),T04.element(1,2), T04.element(2,2)};
	double z5[3] = { T05.element(0,2),T05.element(1,2), T05.element(2,2)};
	double z6[3] = { T06.element(0,2),T06.element(1,2), T06.element(2,2)};

	double o1[3] = {T01.element(0,3), T01.element(1,3), T01.element(2,3)};
	double o2[3] = {T02.element(0,3), T02.element(1,3), T02.element(2,3)};
	double o3[3] = {T03.element(0,3), T03.element(1,3), T03.element(2,3)};
	double o4[3] = {T04.element(0,3), T04.element(1,3), T04.element(2,3)};
	double o5[3] = {T05.element(0,3), T05.element(1,3), T05.element(2,3)};
	double o6[3] = {T06.element(0,3), T06.element(1,3), T06.element(2,3)};
	double o7[3] = {T07.element(0,3), T07.element(1,3), T07.element(2,3)};
 
	double O1[3] ={o7[0],o7[1],o7[2]};
	double O2[3] ={o7[0]-o1[0],o7[1]-o1[1],o7[2]-o1[2]};
	double O3[3] ={o7[0]-o2[0],o7[1]-o2[1],o7[2]-o2[2]};
	double O4[3] ={o7[0]-o3[0],o7[1]-o3[1],o7[2]-o3[2]};
	double O5[3] ={o7[0]-o4[0],o7[1]-o4[1],o7[2]-o4[2]};
	double O6[3] ={o7[0]-o5[0],o7[1]-o5[1],o7[2]-o5[2]};
	double O7[3] ={o7[0]-o6[0],o7[1]-o6[1],o7[2]-o6[2]};
	
	hMatrix c1(1,3),c2(1,3),c3(1,3),c4(1,3),c5(1,3),c6(1,3),c7(1,3);
	c1 =  cross(&k[0],&O1[0]);
	c2 =  cross(&z1[0],&O2[0]);
	c3 =  cross(&z2[0],&O3[0]);
	c4 =  cross(&z3[0],&O4[0]);
	c5 =  cross(&z4[0],&O5[0]);
	c6 =  cross(&z5[0],&O6[0]);
	c7 =  cross(&z6[0],&O7[0]);

	double J[42] = { c1.element(0,0), c2.element(0,0), c3.element(0,0), c4.element(0,0), c5.element(0,0), c6.element(0,0), c7.element(0,0),
					c1.element(0,1), c2.element(0,1), c3.element(0,1), c4.element(0,1), c5.element(0,1), c6.element(0,1), c7.element(0,1),
					c1.element(0,2), c2.element(0,2), c3.element(0,2), c4.element(0,2), c5.element(0,2), c6.element(0,2), c7.element(0,2),
					k[0],z1[0],z2[0],z3[0],z4[0],z5[0],z6[0],
					k[1],z1[1],z2[1],z3[1],z4[1],z5[1],z6[1],
					k[2],z1[2],z2[2],z3[2],z4[2],z5[2],z6[2]};
	hMatrix Jacobian(6,7);
	Jacobian.SET(6,7,&J[0]);
	return Jacobian;
}

hMatrix cross(double *row, double *col){
	double Up[3],Down[3],result[3];
	for(int i=0; i<3; i++){
		Up[i] = *row;
		Down[i] = *col;
		row++;
		col++;
	}
	
	result[0] = Up[1]*Down[2] - Up[2]*Down[1];
	result[1] = Up[2]*Down[0] - Up[0]*Down[2];
	result[2] = Up[0]*Down[1] - Up[1]*Down[0];

	hMatrix Cross(1,3);
	Cross.SET(1,3,&result[0]);

	return Cross;
}

hMatrix Inverse(hMatrix Mat){
	double ratio,a,temp1,temp2;    
	int i, j, k;    
	
	int row = Mat.GetRow(),col = Mat.GetCol(); 
	hMatrix Inv(row,col*2);
	for(i=0;i<row; i++){
		for(j=0;j<col; j++)
			Inv.SetElement(i,j,Mat.element(i,j));
	}
	for(i =0;i<row; i++)
		for(j=col;j<2*col;j++){
			if(i==(j-row))
				Inv.SetElement(i,j,1);
			else
				Inv.SetElement(i,j,0);
		}

	for(i = 0; i < row; i++){        
		for(j = 0; j < col; j++){            
			if(i!=j){                
				ratio = Inv.element(j,i)/Inv.element(i,i);
			
				for(k = 0; k < 2*col; k++){     
					temp1 = Inv.element(j,k) - ratio * Inv.element(i,k);
					Inv.SetElement(j,k,temp1);   
				}            
			}        
		}    
	}

	for(i = 0; i < row; i++){        
		a = Inv.element(i,i);       
		for(j = 0; j < 2*col; j++){
			temp2 = Inv.element(i,j)/a;
			Inv.SetElement(i,j,temp2);     
		}    
	}     
	hMatrix Result(row,col);
	for(i = 0; i < row; i++){        
		for(j = col; j < 2*col; j++){            
			Result.SetElement(i,j-col,Inv.element(i,j));
		}
	}
	return Result;
}

hMatrix Pseudo_Inverse(hMatrix Mat){
	int row = Mat.GetRow(), col = Mat.GetCol();
	hMatrix trans(col,row);
	trans =Transpose(Mat);
	hMatrix temp1(row,row);
	temp1 = Inverse(Mat*trans);
	hMatrix temp2(col,row);
	temp2 = trans*temp1;

	return temp2;
}


hMatrix Transpose(hMatrix Mat){
	int row = Mat.GetRow(), col = Mat.GetCol();
	hMatrix Trans(col,row);
	for(int i =0; i<row; i++){
		for(int j=0; j<col; j++){
			Trans.SetElement(j,i,Mat.element(i,j));
		}
	}

	return Trans;
}

double dot(hMatrix A,hMatrix B){
	double result=0;
	
	hMatrix temp(1,1);
	temp = Transpose(A)*B;
	result = temp.element(0,0);
	
	return result;
}

hMatrix T_Rotation(hMatrix Mat){
	hMatrix Rotation(3,3);
	
	for(int i = 0; i<3; i++){
		for(int j= 0; j<3; j++){
			Rotation.SetElement(i,j,Mat.element(i,j));
		}
		
	}
	return Rotation;
}

hMatrix Tool_Position(double roll, double pitch, double yaw){
	hMatrix Yaw(3,3), Pitch(3,3), Roll(3,3), temp(3,3);
	double roll_data[9] =  {1,	0,	0,
							0,cos(roll),-sin(roll),
							0,sin(roll),cos(roll)};

	double pitch_data[9] =  {cos(pitch),0,sin(pitch),
							0,	1,	0,
							-sin(pitch),0,cos(pitch)};
	double yaw_data[9] =  {cos(yaw),-sin(yaw),	0,
							sin(yaw),cos(yaw),0,
							0,0,1};
	Roll.SET(3,3,&roll_data[0]);
	Pitch.SET(3,3,&pitch_data[0]);
	Yaw.SET(3,3,&yaw_data[0]);

	temp = Yaw*Pitch;

	return temp*Roll;
}