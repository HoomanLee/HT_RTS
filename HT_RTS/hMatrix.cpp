#include "hMatrix.h"

hMatrix::hMatrix()
{
	row = 0;
	col = 0;
	M_pData=NULL;
}

hMatrix::hMatrix(int n, int m)
{
	row = n;
	col = m;

	M_pData=new double*[n];
	memset(M_pData,0,sizeof(double*)*n);

	for(int i=0;i<n;i++){
		M_pData[i]=new double[m];
		memset(M_pData[i],0,sizeof(double)*m);
	}
}


hMatrix::~hMatrix()
{
	if(M_pData)
	{
		for(int i=0;i<row;i++){
			delete[] M_pData[i];
			M_pData[i]=NULL;
		}
		delete[] M_pData;
	}
	M_pData=NULL;
}

hMatrix::hMatrix(hMatrix &opr)
{
	int i,j;
	row=opr.GetRow();
	col=opr.GetCol();

	M_pData=new double*[row];
	memset(M_pData,0,sizeof(double*)*row);

	for(i=0;i<row;i++){
		M_pData[i]=new double[col];
		for(j=0;j<col;j++){
			M_pData[i][j]=opr.element(i,j);
		}
	}
}


hMatrix hMatrix::operator*(const hMatrix& opr) const
{
	hMatrix ret(row,opr.GetCol());

	if(col != opr.GetRow()) return ret;
	
	int i,j,k;
	double sum=0;
	for(i=0;i<row;i++)
	{
		for(j=0;j<opr.GetCol();j++)
		{
			for(k=0;k<opr.GetRow();k++)
				{
					sum+=M_pData[i][k]*opr.element(k,j);
				}
					ret.SetElement(i,j,sum);
					sum=0;	
		}
			
	}	
	return ret;
}

hMatrix& hMatrix::operator=(const hMatrix& opr)
{
	int i,j;
	
	/*if(row!=opr.GetRow()||col!=opr.GetCol())
	{
		row=col=0;
		
		if(M_pData)
		{
			for(i=0;i<row;i++)
			{
				delete[] M_pData[i];
				M_pData[i]=NULL;
			}
			delete[] M_pData;
		}
		M_pData=NULL;*/

		if(M_pData==NULL){
			row=opr.GetRow();
			col=opr.GetCol();

			M_pData=new double*[row];
			memset(M_pData,0,sizeof(double*)*row);

			for(i=0;i<row;i++){
				M_pData[i]=new double[col];
				memset(M_pData[i],0,sizeof(double)*col);
			}
		}
		
		for(i=0;i<row;i++){
			for(j=0;j<col;j++){
				M_pData[i][j]=opr.element(i,j);
			}
		}
		return *this;
	
}

// 전치행렬을 만드는 함수
hMatrix& hMatrix::transpose()
{
// 만약 행렬의 크기가 정상적이지 않다면 리턴
if(row < 1 || col < 1) return *this;
// 만약 행과 열의 크기가 다르다면 리턴
if(row != col) return *this;
// 만약 메모리가 생성되지 않았다면 리턴
if(M_pData == NULL) return *this;

// 행과 열의 값을 바꾸어 전치행렬을 구성
int i, j;
double nTemp;
for(i = 0; i < row; i++)
{
for(j = i + 1; j < row; j++)
{
nTemp = M_pData[i][j];
M_pData[i][j] = M_pData[j][i];
M_pData[j][i] = nTemp;
}
}
return *this;
}


double hMatrix::element(int n, int m) const
{
	if(row<1 || col<1) return 0;
	if (M_pData==NULL) return 0;
	if (n<0 || n>=row) return 0;
	if (m<0 || m>=col) return 0;

	return M_pData[n][m];
}

void hMatrix::displayhMatrix()
{
	if(row<1 || col<1) return;
	if(M_pData==NULL) return;

	int i,j;
	cout<<"행 : "<<row<<"   /  열 : "<<col<<endl;

	for(i=0;i<row;i++)
	{
		for(j=0;j<col;j++)
		{
			cout<<M_pData[i][j]<<"\t\t";
		}
		cout<<endl;
	}
	cout<<endl<<endl;
}


void hMatrix::SetElement(int n,int m,double value)
{
	if(row<1 || col<1) return;
	if(M_pData==NULL) return;
	if(n<0 || n>=row) return;
	if(m<0 || m>=col) return;

	M_pData[n][m]=value;
}

void hMatrix::ReadhMatrix()
{
	cout<<"각 항목들을 입력하세요.";
	for(int i=0;i<row;i++){
		for(int j=0;j<col;j++)
			cin>>this->M_pData[i][j];
	}
}
void hMatrix::SET(int n, int m, double *val)
{
	for(int i=0;i<n;i++){
		for(int j=0;j<m;j++){
			this->M_pData[i][j]=*val;
			val++;}
	}
}

/////////////////////////////////////////////좌표변환/////////////////////////////////////////
hMatrix hMatrix::Set_T(hMatrix R, hMatrix T){
	hMatrix T_R(3,3);//T의 Rotation 부분
	hMatrix T_P(3,1);//T의 Position 부분 
	hMatrix R_R(3,3);
	hMatrix R_P(3,1);
	hMatrix Result(4,4);
	double value_R[9]={T.element(0,0),T.element(0,1),T.element(0,2),T.element(1,0),T.element(1,1),T.element(1,2),T.element(2,0),T.element(2,1),T.element(2,2)};
	double value_P[3]={T.element(0,3),T.element(1,3),T.element(2,3)};
	T_R.SET(3,3,&value_R[0]);
	T_P.SET(3,1,&value_P[0]);
	R_R=R*T_R;
	R_P=R*T_P;

	double value_Result[16]={R_R.element(0,0),R_R.element(0,1),R_R.element(0,2),R_P.element(0,0),R_R.element(1,0),R_R.element(1,1),R_R.element(1,2),
		R_P.element(1,0),R_R.element(2,0),R_R.element(2,1),R_R.element(2,2),R_P.element(2,0),0,0,0,1};
	Result.SET(4,4,&value_Result[0]);
	return Result;
}