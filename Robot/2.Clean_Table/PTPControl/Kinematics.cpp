
#include "Kinematics.h"


double d1 = 345;
double d2 = 0;
double d3 = 0;
double d4 = 295;
double d5 = 0;
double d6 = 102+130;

double alpha1 = PI /2;//90度
double alpha2 = 0;
double alpha3 = PI /2;//90度
double alpha4 = -PI /2;//-90度
double alpha5 = PI /2;//90度
double alpha6 = 0;


double a1 = 75;
double a2 = 270;
double a3 = 90;
double a4 = 0;
double a5 = 0;
double a6 = 0;



double d_new1 = 200;
double d_new2 = -150;
double d_new3 = -150;
double d_new4 = -150;
double d_new5 = (295)/2;
double d_new6 = 102 + 100;
double d_new7 = 102 + 175;




double a_new1 = 0;
double a_new2 = 0;
double a_new3 = (270)/2;
double a_new4 = 270;
double a_new5 = 0;
double a_new6 = 0;
double a_new7 = 0;





double Kinematics::over180_rad(double input)
{
	double mod=input;
	if(input>PI)
		mod=input-2* PI;
	if(input<-PI)
		mod=input+2* PI;
	return mod;
}
void Kinematics::calculateT(double theta,double alpha,double a,double d, Mat *output)
{
	Mat RotZ=Mat::zeros(4,4,CV_64FC1);
	RotZ.at<double>(0,0)=1;
	RotZ.at<double>(0,1)=0;
	RotZ.at<double>(0,2)=0;
	RotZ.at<double>(0,3)=0;

	RotZ.at<double>(1,0)=0;
	RotZ.at<double>(1,1)=cos(alpha);
	RotZ.at<double>(1,2)=-sin(alpha);
	RotZ.at<double>(1,3)=0;

	RotZ.at<double>(2,0)=0;
	RotZ.at<double>(2,1)=sin(alpha);
	RotZ.at<double>(2,2)=cos(alpha);
	RotZ.at<double>(2,3)=0;

	RotZ.at<double>(3,0)=0;
	RotZ.at<double>(3,1)=0;
	RotZ.at<double>(3,2)=0;
	RotZ.at<double>(3,3)=1;

	Mat TransZ=Mat::zeros(4,4,CV_64FC1);
	TransZ.at<double>(0,0)=1;
	TransZ.at<double>(0,1)=0;
	TransZ.at<double>(0,2)=0;
	TransZ.at<double>(0,3)=a;

	TransZ.at<double>(1,0)=0;
	TransZ.at<double>(1,1)=1;
	TransZ.at<double>(1,2)=0;
	TransZ.at<double>(1,3)=0;

	TransZ.at<double>(2,0)=0;
	TransZ.at<double>(2,1)=0;
	TransZ.at<double>(2,2)=1;
	TransZ.at<double>(2,3)=0;

	TransZ.at<double>(3,0)=0;
	TransZ.at<double>(3,1)=0;
	TransZ.at<double>(3,2)=0;
	TransZ.at<double>(3,3)=1;

	Mat TransX=Mat::zeros(4,4,CV_64FC1);
	TransX.at<double>(0,0)=1;
	TransX.at<double>(0,1)=0;
	TransX.at<double>(0,2)=0;
	TransX.at<double>(0,3)=0;

	TransX.at<double>(1,0)=0;
	TransX.at<double>(1,1)=1;
	TransX.at<double>(1,2)=0;
	TransX.at<double>(1,3)=0;

	TransX.at<double>(2,0)=0;
	TransX.at<double>(2,1)=0;
	TransX.at<double>(2,2)=1;
	TransX.at<double>(2,3)=d;

	TransX.at<double>(3,0)=0;
	TransX.at<double>(3,1)=0;
	TransX.at<double>(3,2)=0;
	TransX.at<double>(3,3)=1;

	Mat RotX=Mat::zeros(4,4,CV_64FC1);
	RotX.at<double>(0,0)=cos(theta);
	RotX.at<double>(0,1)=-sin(theta);
	RotX.at<double>(0,2)=0;
	RotX.at<double>(0,3)=0;

	RotX.at<double>(1,0)=sin(theta);
	RotX.at<double>(1,1)=cos(theta);
	RotX.at<double>(1,2)=0;
	RotX.at<double>(1,3)=0;

	RotX.at<double>(2,0)=0;
	RotX.at<double>(2,1)=0;
	RotX.at<double>(2,2)=1;
	RotX.at<double>(2,3)=0;

	RotX.at<double>(3,0)=0;
	RotX.at<double>(3,1)=0;
	RotX.at<double>(3,2)=0;
	RotX.at<double>(3,3)=1;

	*output=RotX*TransX*TransZ*RotZ;

}
void Kinematics::calculateA1(double input1, Mat *output)
{
	Mat T1;
	calculateT(input1,alpha1,a1,d1,&T1);
	*output = T1;
}
void Kinematics::calculateA2(double theta1, double theta2, Mat *output)
{
	Mat T1,T2; 
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	*output = T1*T2;
}
void Kinematics::calculateA3(double theta1, double theta2, double theta3, Mat *output)
{
	Mat T1,T2,T3;
	calculateT(theta1,alpha1,a1,d1,&T1); 
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	*output = T1*T2*T3;
}
void Kinematics::calculateA4(double theta1, double theta2, double theta3, double theta4, Mat *output)
{
	Mat T1,T2,T3,T4;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	calculateT(theta4,alpha4,a4,d4,&T4);
	*output = T1*T2*T3*T4;
}
void Kinematics::calculateA5(double theta1, double theta2, double theta3, double theta4, double theta5, Mat *output)
{
	Mat T1,T2,T3,T4,T5;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	calculateT(theta4,alpha4,a4,d4,&T4);
	calculateT(theta5,alpha5,a5,d5,&T5);
	*output = T1*T2*T3*T4*T5;
}
void Kinematics::calculateA6(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, Mat *output)
{
	Mat T1,T2,T3,T4,T5,T6;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	calculateT(theta4,alpha4,a4,d4,&T4);
	calculateT(theta5,alpha5,a5,d5,&T5);
	calculateT(theta6,alpha6,a6,d6,&T6);
	*output = T1*T2*T3*T4*T5*T6;
}
void Kinematics::calculateJ1(double EEx,double EEy,double EEz,double z1,double z2,double z3,double *J1,double *J1p)
{
	double Ex = EEx - d6*z1;
	double Ey = EEy - d6*z2;
	double Ez = EEz - d6*z3;
	*J1 = atan2(Ey,Ex);
	*J1p = atan2(Ey,Ex) + PI;
	//增加一個機制 若超過180度 自動變負號
	*J1 = over180_rad(*J1);
	*J1p = over180_rad(*J1p);
}
void Kinematics::calculateJ2J3(double Ex,double Ey,double Ez,double J1,double *J2,double *J2p,double *J3,double *J3p)
{
	Mat A1=Mat::zeros(4,4,CV_64FC1);
	Kinematics::calculateA1(J1,&A1);
	double Bx=A1.at<double>(0,3);
	double By=A1.at<double>(1,3);
	double Bz=A1.at<double>(2,3);
	//printf("Bx %f \n",Bx);
	//printf("By %f \n",By);
	double P = 0;
	double AB = a1;
	if(abs(Bx)>1){
		if (Ex*Bx < 0)
		{
			P = -sqrt(Ex*Ex+Ey*Ey) - AB;
		}
		else 
		{
			P = sqrt(Ex*Ex+Ey*Ey) - AB;
		}
	}
	else if(abs(By)>1){
		if (Ey*By < 0)
		{
			P = -sqrt(Ex*Ex+Ey*Ey) - AB;
		}
		else 
		{
			P = sqrt(Ex*Ex+Ey*Ey) - AB;
		}
	}
	else 
	{
		P = sqrt(Ex*Ex+Ey*Ey) - AB;
	}
	double OA = d1;
	double Q = Ez - OA;
	double BC = a2;
	double CD = a3;
	double DE = d4;
	double BE = sqrt(P*P+Q*Q);//sqrt((Ex-Bx)*(Ex-Bx)+(Ey-By)*(Ey-By)+(Ez-Bz)*(Ez-Bz));// 
	double CE = sqrt(CD*CD+DE*DE);
	double angleCBE = acos((BC*BC+BE*BE-CE*CE)/(2*BC*BE));
	//printf("CBE %f \n",angleCBE*180/pi);
	//printf("%f \n",(BC*BC+BE*BE-CE*CE)/(2*BC*BE));
	double angleEBU = atan2(Q,P);//使用arctan2函數以修正arctan給出的角度值
	//printf("EBU %f \n",angleEBU*180/pi);
	*J2=- (PI /2-(angleCBE+angleEBU));
	*J2p=- (PI /2-(angleEBU-angleCBE) );
	//增加一個機制 若超過180度 自動變負號
	*J2 = over180_rad(*J2);
	*J2p = over180_rad(*J2p);
	//printf("P %f \n",P);
	//printf("Q %f \n",Q);
	double angleBCE = acos((BC*BC+CE*CE-BE*BE)/(2*BC*CE));
	double angleECD = acos(CD/CE);//acosd((CD^2+CE^2-DE^2)/(2*CD*CE));
	*J3 = angleBCE + angleECD - PI; //elbow up
	*J3p = PI -(angleBCE - angleECD); //elbow down
	//增加一個機制 若超過180度 自動變負號
	*J3 = over180_rad(*J3);
	*J3p = over180_rad(*J3p);

}
void Kinematics::calculateJ4J5J6(double EEx,double EEy,double EEz,double x1,double x2,double x3,double J1,double J2,double J3,double *J4, double *J5, double *J6,double *J4p, double *J5p, double *J6p)
{
	Mat A3 = Mat::zeros(4,4,CV_64FC1);
	calculateA3(J1,J2+(PI /2),J3,&A3);
	double newEEx = EEx-A3.at<double>(0,3);
	double newEEy = EEy-A3.at<double>(1,3);
	double newEEz = EEz-A3.at<double>(2,3);
	/*
	printf("%f \n",newEEx);
	printf("%f \n",newEEy);
	printf("%f \n",newEEz);
	*/

	double xProjection = (newEEx*A3.at<double>(0,0)) + (newEEy*A3.at<double>(1,0)) + (newEEz*A3.at<double>(2,0));
	double yProjection = (newEEx*A3.at<double>(0,1)) + (newEEy*A3.at<double>(1,1)) + (newEEz*A3.at<double>(2,1));
	double zProjection = newEEx*A3.at<double>(0,2) + newEEy*A3.at<double>(1,2) + newEEz*A3.at<double>(2,2);
	
	*J4=atan2(yProjection,xProjection);
	*J4p=atan2(yProjection,xProjection) + PI;
	//增加一個機制 若超過180度 自動變負號
	*J4 = over180_rad(*J4);
	*J4p = over180_rad(*J4p);
	/*printf("%f \n",RAD2DEG(*J4));
	printf("%f \n",RAD2DEG(*J4p));*/
	double EExdir1 = xProjection*A3.at<double>(0,0);
	double EExdir2 = xProjection*A3.at<double>(1,0);
	double EExdir3 = xProjection*A3.at<double>(2,0);

	double EEydir1 = yProjection*A3.at<double>(0,1);
	double EEydir2 = yProjection*A3.at<double>(1,1);
	double EEydir3 = yProjection*A3.at<double>(2,1);

	double EEinXYprojection1 = EExdir1 + EEydir1;
	double EEinXYprojection2 = EExdir2 + EEydir2;
	double EEinXYprojection3 = EExdir3 + EEydir3;

	double EEinXYprojectionValue=sqrt(EEinXYprojection1*EEinXYprojection1 + EEinXYprojection2*EEinXYprojection2 + EEinXYprojection3*EEinXYprojection3);


	// 會用102 是因為末端長度是102，如果用newEEvalue，那是末端長度和前一軸長度相加
	double J5temp1cos = (zProjection - d4)/d6;
	double J5temp2cos = (newEEx*EEinXYprojection1+newEEy*EEinXYprojection2+newEEz*EEinXYprojection3)/(d6*EEinXYprojectionValue);
	if(J5temp1cos>1)
	J5temp1cos=1;

	if(J5temp2cos>1)
	J5temp2cos=1;

	if(J5temp1cos<-1)
	J5temp1cos=-1;

	if(J5temp2cos<-1)
	J5temp2cos=-1;
	//printf("(zProjection - d4)/d6 %f \n",J5temp1cos);
	/*如果有算術誤差，將其消除*/
	double J5temp1 = acos(J5temp1cos);
	double J5temp2 = acos(J5temp2cos);
	
	if(J5temp2 > PI /2)
		*J5 = -1*J5temp1;
	else
		*J5 = J5temp1;
	*J5p = -*J5;
	//增加一個機制 若超過180度 自動變負號
	*J5 = over180_rad(*J5);
	*J5p = over180_rad(*J5p);

	Mat A5 = Mat::zeros(4,4,CV_64FC1);
	calculateA5(J1,J2+(PI /2),J3,*J4,*J5,&A5);
	double J6temp1cos = x1*A5.at<double>(0,0)+x2*A5.at<double>(1,0)+x3*A5.at<double>(2,0);
	double J6temp2cos = x1*A5.at<double>(0,1)+x2*A5.at<double>(1,1)+x3*A5.at<double>(2,1);
	if(J6temp1cos>1)
	J6temp1cos=1;

	if(J6temp2cos>1)
	J6temp2cos=1;

	if(J6temp1cos<-1)
	J6temp1cos=-1;

	if(J6temp2cos<-1)
	J6temp2cos=-1;
	/*如果有算術誤差，將其消除*/

	double J6temp1=acos(J6temp1cos);
	double J6temp2=acos(J6temp2cos);
	
	if(J6temp2>PI /2)
	{
		*J6=-1*J6temp1;
		*J6p = -1*J6temp1 + PI;
	}
	else
	{
		*J6=J6temp1;
		*J6p=J6temp1 + PI;
	}
	//增加一個機制 若超過180度 自動變負號
	*J6 = over180_rad(*J6);
	*J6p = over180_rad(*J6p);

}
Mat Kinematics::FindOptimalSol(Mat input,Mat compare)
{
	Mat Sol = Mat::zeros(8,6,CV_64FC1);
	Sol = input;
	int IsSolution[8];
	double distance[8];
	double buffer=0;
	int numofsol = 0;

	for(int i = 0;i < 8;i++)
	{
		IsSolution[i] = 0;
		distance[i] = 0;
	}
	
	for(int i = 0;i < 8;i++)
	{
		for(int j = 0;j < 6;j++)
		{
			if( _isnan( input.at<double>(i,j) ) )
			{
				IsSolution[i] = 0;
				//printf("%f ",RAD2DEG(input.at<double>(i,j)) );
			}
		}
		/*if(IsSolution[i] == 1)
		{
			numofsol++;
		}
		//printf("\n ");
		*/
		if(IsSolution[i] == 1)
		{
			numofsol++;
		}
	}

	if(numofsol == 0)
	{
		printf("此點在機械手臂工作範圍外，請關閉程式 \n");
		Mat OptimalSol = Mat::zeros(1,6,CV_64FC1);
		OptimalSol.at<double>(0,0) = 0.0;
		OptimalSol.at<double>(0,1) = 0.0;
		OptimalSol.at<double>(0,2) = 0.0;
		OptimalSol.at<double>(0,3) = 0.0;
		OptimalSol.at<double>(0,4) = 0.0;
		OptimalSol.at<double>(0,5) = 0.0;

		return OptimalSol;

	}
	else
	{
		int OptimalSolutionNumber = 0;
		for(int i = 0;i < 8;i++)
		{
			if(IsSolution[i] == 1)
			{
				distance[i]=0;
				for(int j = 0;j < 6;j++)
				{
					buffer = input.at<double>(i,j) - compare.at<double>(0,j);
					distance[i] = distance[i] + buffer*buffer;
					buffer = 0;
				}
				OptimalSolutionNumber=i;
			}
		}
		for(int counter = 0;counter < 8;counter++)
		{
			if(IsSolution[counter] == 1)
			{
				if(distance[OptimalSolutionNumber]>distance[counter])
				{
					OptimalSolutionNumber=counter;
				}
				else if(distance[OptimalSolutionNumber]<distance[counter])
				{
					OptimalSolutionNumber=OptimalSolutionNumber;
				}
				else 
				{
					OptimalSolutionNumber=OptimalSolutionNumber;
				}
			}
		}
		Mat OptimalSol = Mat::zeros(1,6,CV_64FC1);
		OptimalSol.at<double>(0,0) = Sol.at<double>(OptimalSolutionNumber,0);
		OptimalSol.at<double>(0,1) = Sol.at<double>(OptimalSolutionNumber,1);
		OptimalSol.at<double>(0,2) = Sol.at<double>(OptimalSolutionNumber,2);
		OptimalSol.at<double>(0,3) = Sol.at<double>(OptimalSolutionNumber,3);
		OptimalSol.at<double>(0,4) = Sol.at<double>(OptimalSolutionNumber,4);
		OptimalSol.at<double>(0,5) = Sol.at<double>(OptimalSolutionNumber,5);

		return OptimalSol;
	}
}
Mat Kinematics::FindOptimalSol_YA(Mat input,Mat compare,int Singularplace[Sol_Num],bool &Singular)
{
	Mat Sol = Mat::zeros(8,6,CV_64FC1);
	Sol = input;
	int IsSolution[8];
	double distance[8];
	double buffer=0;
	int numofsol = 0;
	double max_pos[6] = {DEG2RAD(170) ,DEG2RAD(130), DEG2RAD(170), DEG2RAD(190), DEG2RAD(125), DEG2RAD(360) };
	double min_pos[6] = {DEG2RAD(-170) ,DEG2RAD(-85), DEG2RAD(-110), DEG2RAD(-190), DEG2RAD(-125), DEG2RAD(-360) };

	for(int i = 0;i < 8;i++)
	{
		IsSolution[i] = 0;
		distance[i] = 0;
	}
	
	for(int i = 0;i < 8;i++)
	{
		for(int j = 0;j < 6;j++)
		{
			if( _isnan( input.at<double>(i,j) ) )
			{
				IsSolution[i] = 0;
				//printf("%f ",RAD2DEG(input.at<double>(i,j)) );
			}
		}
		/*if(IsSolution[i] == 1)
		{
			numofsol++;
		}
		//printf("\n ");
		*/
	}

	for (int i = 0; i < Sol_Num ; i++)
	{
		if (abs(input.at<double>(i,3)) < 3.0 && abs(input.at<double>(i,0)) < 1.6 && Singularplace[i] ==1 )
		{
			IsSolution[i] = 1;
		}
		else
		{
			IsSolution[i] = 0;
			
		}
		// 檢查joint是否超過限制
		if (IsSolution[i] == 1)
		{
		
			for (int j = 0; j<AXIS; j++)
			{   
				if (max_pos[j] < input.at<double>(i,j) || input.at<double>(i,j) < min_pos[j])
				{
					IsSolution[i] = 0;
				}
			}
		}

		// 檢查joint是否超過限制
		
		if(IsSolution[i] == 1)
		{
			numofsol++;
		}
	}
	if(numofsol == 0)
	{
		printf("此點在機械手臂工作範圍外，請關閉程式 \n");
		Mat OptimalSol = Mat::zeros(1,6,CV_64FC1);
		OptimalSol.at<double>(0,0) = 0.0;
		OptimalSol.at<double>(0,1) = 0.0;
		OptimalSol.at<double>(0,2) = 0.0;
		OptimalSol.at<double>(0,3) = 0.0;
		OptimalSol.at<double>(0,4) = 0.0;
		OptimalSol.at<double>(0,5) = 0.0;
		Singular = true;
		return OptimalSol;

	}
	else
	{
		int OptimalSolutionNumber = 0;
		for(int i = 0;i < 8;i++)
		{
			if(IsSolution[i] == 1)
			{
				distance[i]=0;
				for(int j = 0;j < 6;j++)
				{
					buffer = input.at<double>(i,j) - compare.at<double>(0,j);
					distance[i] = distance[i] + buffer*buffer;
					buffer = 0;
				}
				OptimalSolutionNumber=i;
			}
		}
		for(int counter = 0;counter < 8;counter++)
		{
			if(IsSolution[counter] == 1)
			{
				if(distance[OptimalSolutionNumber]>distance[counter])
				{
					OptimalSolutionNumber=counter;
				}
				else if(distance[OptimalSolutionNumber]<distance[counter])
				{
					OptimalSolutionNumber=OptimalSolutionNumber;
				}
				else 
				{
					OptimalSolutionNumber=OptimalSolutionNumber;
				}
			}
		}
		Mat OptimalSol = Mat::zeros(1,6,CV_64FC1);
		OptimalSol.at<double>(0,0) = Sol.at<double>(OptimalSolutionNumber,0);
		OptimalSol.at<double>(0,1) = Sol.at<double>(OptimalSolutionNumber,1);
		OptimalSol.at<double>(0,2) = Sol.at<double>(OptimalSolutionNumber,2);
		OptimalSol.at<double>(0,3) = Sol.at<double>(OptimalSolutionNumber,3);
		OptimalSol.at<double>(0,4) = Sol.at<double>(OptimalSolutionNumber,4);
		OptimalSol.at<double>(0,5) = Sol.at<double>(OptimalSolutionNumber,5);
		Singular = false;
		return OptimalSol;
	}
}
Mat Kinematics::Solution(double Target_Joint[Sol_Num][AXIS],double nowjointpos[AXIS],int Singularplace[Sol_Num],bool &Singular)
{
	Mat output = Mat::zeros(8,6,CV_64FC1);
	output.at<double>(0,0) = Target_Joint[0][0];
	output.at<double>(0,1) = Target_Joint[0][1];
	output.at<double>(0,2) = Target_Joint[0][2];
	output.at<double>(0,3) = Target_Joint[0][3];
	output.at<double>(0,4) = Target_Joint[0][4];
	output.at<double>(0,5) = Target_Joint[0][5];

	output.at<double>(1,0) = Target_Joint[1][0];
	output.at<double>(1,1) = Target_Joint[1][1];
	output.at<double>(1,2) = Target_Joint[1][2];
	output.at<double>(1,3) = Target_Joint[1][3];
	output.at<double>(1,4) = Target_Joint[1][4];
	output.at<double>(1,5) = Target_Joint[1][5];

	output.at<double>(2,0) = Target_Joint[2][0];
	output.at<double>(2,1) = Target_Joint[2][1];
	output.at<double>(2,2) = Target_Joint[2][2];
	output.at<double>(2,3) = Target_Joint[2][3];
	output.at<double>(2,4) = Target_Joint[2][4];
	output.at<double>(2,5) = Target_Joint[2][5];

	output.at<double>(3,0) = Target_Joint[3][0];
	output.at<double>(3,1) = Target_Joint[3][1];
	output.at<double>(3,2) = Target_Joint[3][2];
	output.at<double>(3,3) = Target_Joint[3][3];
	output.at<double>(3,4) = Target_Joint[3][4];
	output.at<double>(3,5) = Target_Joint[3][5];

	output.at<double>(4,0) = Target_Joint[4][0];
	output.at<double>(4,1) = Target_Joint[4][1];
	output.at<double>(4,2) = Target_Joint[4][2];
	output.at<double>(4,3) = Target_Joint[4][3];
	output.at<double>(4,4) = Target_Joint[4][4];
	output.at<double>(4,5) = Target_Joint[4][5];

	output.at<double>(5,0) = Target_Joint[5][0];
	output.at<double>(5,1) = Target_Joint[5][1];
	output.at<double>(5,2) = Target_Joint[5][2];
	output.at<double>(5,3) = Target_Joint[5][3];
	output.at<double>(5,4) = Target_Joint[5][4];
	output.at<double>(5,5) = Target_Joint[5][5];

	output.at<double>(6,0) = Target_Joint[6][0];
	output.at<double>(6,1) = Target_Joint[6][1];
	output.at<double>(6,2) = Target_Joint[6][2];
	output.at<double>(6,3) = Target_Joint[6][3];
	output.at<double>(6,4) = Target_Joint[6][4];
	output.at<double>(6,5) = Target_Joint[6][5];

	output.at<double>(7,0) = Target_Joint[7][0];
	output.at<double>(7,1) = Target_Joint[7][1];
	output.at<double>(7,2) = Target_Joint[7][2];
	output.at<double>(7,3) = Target_Joint[7][3];
	output.at<double>(7,4) = Target_Joint[7][4];
	output.at<double>(7,5) = Target_Joint[7][5];

	Mat currentpos = Mat::zeros(1,6,CV_64FC1);
	currentpos.at<double>(0,0) = nowjointpos[0];
	currentpos.at<double>(0,1) = nowjointpos[1];
	currentpos.at<double>(0,2) = nowjointpos[2];
	currentpos.at<double>(0,3) = nowjointpos[3];
	currentpos.at<double>(0,4) = nowjointpos[4];
	currentpos.at<double>(0,5) = nowjointpos[5];


	Mat Solution = Mat::zeros(1,6,CV_64FC1);
	
	Solution = Kinematics::FindOptimalSol_YA(output,currentpos,Singularplace,Singular);

	/*
	static double SolPos[AXIS];
	SolPos[0] = Solution.at<double>(0,0);
	SolPos[1] = Solution.at<double>(0,1);
	SolPos[2] = Solution.at<double>(0,2);
	SolPos[3] = Solution.at<double>(0,3);
	SolPos[4] = Solution.at<double>(0,4);
	SolPos[5] = Solution.at<double>(0,5);
	*/

	return Solution;

	

}

Mat Kinematics::Reverse(Mat nsdt,Mat currentpos)
{
	double z1 = nsdt.at<double>(0,2);
	double z2 = nsdt.at<double>(1,2);
	double z3 = nsdt.at<double>(2,2);
	double x1 = nsdt.at<double>(0,0);
	double x2 = nsdt.at<double>(1,0);
	double x3 = nsdt.at<double>(2,0);
	double EEx = nsdt.at<double>(0,3);
	double EEy = nsdt.at<double>(1,3);
	double EEz = nsdt.at<double>(2,3);
	double J1s[2];
	double J2s[4],J3s[4];
	double J4s[8],J5s[8],J6s[8];
	Kinematics::calculateJ1(EEx,EEy,EEz,z1,z2,z3,&J1s[0],&J1s[1]);
	double Ex = EEx - d6*z1;
	double Ey = EEy - d6*z2;
	double Ez = EEz - d6*z3;
	Kinematics::calculateJ2J3(Ex,Ey,Ez,J1s[0],&J2s[0],&J2s[1],&J3s[0],&J3s[1]);
	Kinematics::calculateJ2J3(Ex,Ey,Ez,J1s[1],&J2s[2],&J2s[3],&J3s[2],&J3s[3]);
	Kinematics::calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1s[0],J2s[0],J3s[0],&J4s[0],&J5s[0],&J6s[0],&J4s[1],&J5s[1],&J6s[1]);
	Kinematics::calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1s[0],J2s[1],J3s[1],&J4s[2],&J5s[2],&J6s[2],&J4s[3],&J5s[3],&J6s[3]);
	Kinematics::calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1s[1],J2s[2],J3s[2],&J4s[4],&J5s[4],&J6s[4],&J4s[5],&J5s[5],&J6s[5]);
	Kinematics::calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1s[1],J2s[3],J3s[3],&J4s[6],&J5s[6],&J6s[6],&J4s[7],&J5s[7],&J6s[7]);
	Mat output = Mat::zeros(8,6,CV_64FC1);
	output.at<double>(0,0) = J1s[0];
	output.at<double>(0,1) = J2s[0];
	output.at<double>(0,2) = J3s[0];
	output.at<double>(0,3) = J4s[0];
	output.at<double>(0,4) = J5s[0];
	output.at<double>(0,5) = J6s[0];

	output.at<double>(1,0) = J1s[0];
	output.at<double>(1,1) = J2s[0];
	output.at<double>(1,2) = J3s[0];
	output.at<double>(1,3) = J4s[1];
	output.at<double>(1,4) = J5s[1];
	output.at<double>(1,5) = J6s[1];

	output.at<double>(2,0) = J1s[0];
	output.at<double>(2,1) = J2s[1];
	output.at<double>(2,2) = J3s[1];
	output.at<double>(2,3) = J4s[2];
	output.at<double>(2,4) = J5s[2];
	output.at<double>(2,5) = J6s[2];

	output.at<double>(3,0) = J1s[0];
	output.at<double>(3,1) = J2s[1];
	output.at<double>(3,2) = J3s[1];
	output.at<double>(3,3) = J4s[3];
	output.at<double>(3,4) = J5s[3];
	output.at<double>(3,5) = J6s[3];

	output.at<double>(4,0) = J1s[1];
	output.at<double>(4,1) = J2s[2];
	output.at<double>(4,2) = J3s[2];
	output.at<double>(4,3) = J4s[4];
	output.at<double>(4,4) = J5s[4];
	output.at<double>(4,5) = J6s[4];

	output.at<double>(5,0) = J1s[1];
	output.at<double>(5,1) = J2s[2];
	output.at<double>(5,2) = J3s[2];
	output.at<double>(5,3) = J4s[5];
	output.at<double>(5,4) = J5s[5];
	output.at<double>(5,5) = J6s[5];

	output.at<double>(6,0) = J1s[1];
	output.at<double>(6,1) = J2s[3];
	output.at<double>(6,2) = J3s[3];
	output.at<double>(6,3) = J4s[6];
	output.at<double>(6,4) = J5s[6];
	output.at<double>(6,5) = J6s[6];

	output.at<double>(7,0) = J1s[1];
	output.at<double>(7,1) = J2s[3];
	output.at<double>(7,2) = J3s[3];
	output.at<double>(7,3) = J4s[7];
	output.at<double>(7,4) = J5s[7];
	output.at<double>(7,5) = J6s[7];
	/*
	for(int i=0;i<8;i++){
		for(int j = 0;j<6;j++){
			printf("%f ",output.at<double>(i,j)*180/pi);
		}
		printf("\n");
	}
	*/
	Mat Solution = Mat::zeros(1,6,CV_64FC1);
	Solution = Kinematics::FindOptimalSol(output,currentpos);
	return Solution;

}
Mat Kinematics::Forward(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6)
{
	Mat T1,T2,T3,T4,T5,T6;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	calculateT(theta4,alpha4,a4,d4,&T4);
	calculateT(theta5,alpha5,a5,d5,&T5);
	calculateT(theta6,alpha6,a6,d6,&T6);
	Mat output = Mat::zeros(4,4,CV_64FC1);
	output = T1*T2*T3*T4*T5*T6;
	return output;
}
void Kinematics::ForwardJ6(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6 , double* C_NowPos )
{
	Mat T1,T2,T3,T4,T5,T6;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	calculateT(theta4,alpha4,a4,d4,&T4);
	calculateT(theta5,alpha5,a5,d5,&T5);
	calculateT(theta6,alpha6,a6,d6,&T6);
	Mat output = Mat::zeros(4,4,CV_64FC1);
	output = T1*T2*T3*T4*T5*T6;

	for(int i = 0 ; i < TASKSPACE ; i++ )
	{
		C_NowPos[i] = output.at<double>(i,0) ;
	}
	
}
Mat Kinematics::ForwardJ5(double theta1, double theta2, double theta3, double theta4, double theta5)
{
	Mat T1,T2,T3,T4,T5;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	calculateT(theta4,alpha4,a4,d4,&T4);
	calculateT(theta5,alpha5,a5,d5,&T5);
	Mat output_temp = Mat::zeros(4,4,CV_64FC1);
	output_temp = T1*T2*T3*T4*T5;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::ForwardJ4(double theta1, double theta2, double theta3, double theta4)
{
	Mat T1,T2,T3,T4;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	calculateT(theta4,alpha4,a4,d4,&T4);
	Mat output_temp = Mat::zeros(4,4,CV_64FC1);
	output_temp = T1*T2*T3*T4;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::ForwardJ3(double theta1, double theta2, double theta3)
{
	Mat T1,T2,T3;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	calculateT(theta3,alpha3,a3,d3,&T3);
	Mat output_temp = Mat::zeros(4,4,CV_64FC1);
	output_temp = T1*T2*T3;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::ForwardJ2(double theta1, double theta2)
{
	Mat T1,T2;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a2,d2,&T2);
	Mat output_temp = Mat::zeros(4,4,CV_64FC1);
	output_temp = T1*T2;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::ForwardJ1(double theta1)
{
	Mat T1;
	calculateT(theta1, alpha1,a1,d1,&T1);
	Mat output_temp = Mat::zeros(4,4,CV_64FC1);
	output_temp = T1;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::Forward_Jnew1(double theta1)
{
	Mat T1;
	calculateT(theta1, alpha1, a_new1, d_new1, &T1);
	Mat output_temp = Mat::zeros(4, 4, CV_64FC1);
	output_temp = T1;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::Forward_Jnew2(double theta1, double theta2)
{
	Mat T1,T2;
	calculateT(theta1,alpha1,a1,d1,&T1);
	calculateT(theta2,alpha2,a_new2,d_new2,&T2);
	Mat output_temp = Mat::zeros(4,4,CV_64FC1);
	output_temp = T1*T2;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::Forward_Jnew3(double theta1, double theta2)
{
	Mat T1, T2;
	calculateT(theta1, alpha1, a1, d1, &T1);
	calculateT(theta2, alpha2, a_new3, d_new3, &T2);
	Mat output_temp = Mat::zeros(4, 4, CV_64FC1);
	output_temp = T1*T2;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::Forward_Jnew4(double theta1, double theta2)
{
	Mat T1, T2;
	calculateT(theta1, alpha1, a1, d1, &T1);
	calculateT(theta2, alpha2, a_new4, d_new4, &T2);
	Mat output_temp = Mat::zeros(4, 4, CV_64FC1);
	output_temp = T1*T2;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::Forward_Jnew5(double theta1, double theta2, double theta3, double theta4)
{
	Mat T1, T2, T3, T4;
	calculateT(theta1, alpha1, a1, d1, &T1);
	calculateT(theta2, alpha2, a2, d2, &T2);
	calculateT(theta3, alpha3, a3, d3, &T3);
	calculateT(theta4, alpha4, a_new5, d_new5, &T4);
	Mat output_temp = Mat::zeros(4, 4, CV_64FC1);
	output_temp = T1*T2*T3*T4;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}
Mat Kinematics::Forward_Jnew6(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6)
{
	Mat T1, T2, T3, T4, T5, T6;
	calculateT(theta1, alpha1, a1, d1, &T1);
	calculateT(theta2, alpha2, a2, d2, &T2);
	calculateT(theta3, alpha3, a3, d3, &T3);
	calculateT(theta4, alpha4, a4, d4, &T4);
	calculateT(theta5, alpha5, a5, d5, &T5);
	calculateT(theta6, alpha6, a_new6, d_new6, &T6);
	Mat output_temp = Mat::zeros(4, 4, CV_64FC1);
	output_temp = T1*T2*T3*T4*T5*T6;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}

Mat Kinematics::Forward_Jnew7(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6)
{
	Mat T1, T2, T3, T4, T5, T6;
	calculateT(theta1, alpha1, a1, d1, &T1);
	calculateT(theta2, alpha2, a2, d2, &T2);
	calculateT(theta3, alpha3, a3, d3, &T3);
	calculateT(theta4, alpha4, a4, d4, &T4);
	calculateT(theta5, alpha5, a5, d5, &T5);
	calculateT(theta6, alpha6, a_new7, d_new7, &T6);
	Mat output_temp = Mat::zeros(4, 4, CV_64FC1);
	output_temp = T1*T2*T3*T4*T5*T6;
	Mat output = Mat::zeros(3, 1, CV_64FC1);
	output.at<double>(0, 0) = output_temp.at<double>(0, 3);
	output.at<double>(1, 0) = output_temp.at<double>(1, 3);
	output.at<double>(2, 0) = output_temp.at<double>(2, 3);
	return output;
}

void Kinematics::SetPose(double x,double y,double z,double roll,double pitch,double yall,Mat &output)
{
	cv::Mat Yaw=cv::Mat::zeros(3,3,CV_64FC1);
	cv::Mat Pitch=cv::Mat::zeros(3,3,CV_64FC1);
	cv::Mat Roll=cv::Mat::zeros(3,3,CV_64FC1);

	cv::Mat RotationMatrix =cv::Mat::zeros(3,3,CV_64FC1);

	Yaw.at<double>(0,0)=cos(yall);
	Yaw.at<double>(0,1)=-sin(yall);
	Yaw.at<double>(0,2)=0;

	Yaw.at<double>(1,0)=sin(yall);
	Yaw.at<double>(1,1)=cos(yall);
	Yaw.at<double>(1,2)=0;

	Yaw.at<double>(2,0)=0;
	Yaw.at<double>(2,1)=0;
	Yaw.at<double>(2,2)=1;
	
	Pitch.at<double>(0,0)=cos(pitch);
	Pitch.at<double>(0,1)=0;
	Pitch.at<double>(0,2)=sin(pitch);

	Pitch.at<double>(1,0)=0;
	Pitch.at<double>(1,1)=1;
	Pitch.at<double>(1,2)=0;

	Pitch.at<double>(2,0)=-sin(pitch);
	Pitch.at<double>(2,1)=0;
	Pitch.at<double>(2,2)=cos(pitch);
	
	Roll.at<double>(0,0)=1;
	Roll.at<double>(0,1)=0;
	Roll.at<double>(0,2)=0;

	Roll.at<double>(1,0)=0;
	Roll.at<double>(1,1)=cos(roll);
	Roll.at<double>(1,2)=-sin(roll);

	Roll.at<double>(2,0)=0;
	Roll.at<double>(2,1)=sin(roll);
	Roll.at<double>(2,2)=cos(roll);
	
	RotationMatrix=Yaw*Pitch*Roll;

	output.at<double>(0,0)=RotationMatrix.at<double>(0,0);
	output.at<double>(0,1)=RotationMatrix.at<double>(1,0);
	output.at<double>(0,2)=RotationMatrix.at<double>(2,0);

	output.at<double>(1,0)=RotationMatrix.at<double>(0,1);
	output.at<double>(1,1)=RotationMatrix.at<double>(1,1);
	output.at<double>(1,2)=RotationMatrix.at<double>(2,1);

	output.at<double>(2,0)=RotationMatrix.at<double>(0,2);
	output.at<double>(2,1)=RotationMatrix.at<double>(1,2);
	output.at<double>(2,2)=RotationMatrix.at<double>(2,2);


	output.at<double>(0,3)=x;
	output.at<double>(1,3)=y;
	output.at<double>(2,3)=z;
}
void Kinematics::ReverseArray(double nsdt[6],double currentpos[6],double (&output)[6])
{
	Mat input1 = Mat::zeros(3,4,CV_64FC1);
	Mat input2 = Mat::zeros(1,6,CV_64FC1);
	Kinematics::SetPose(nsdt[0], nsdt[1], nsdt[2], nsdt[3], nsdt[4], nsdt[5],input1);
	input2.at<double>(0,0) = currentpos[0];
	input2.at<double>(0,1) = currentpos[1];
	input2.at<double>(0,2) = currentpos[2];
	input2.at<double>(0,4) = currentpos[3];
	input2.at<double>(0,3) = currentpos[4];
	input2.at<double>(0,5) = currentpos[5];
	Mat Solutions  = Mat::zeros(1,6,CV_64FC1);
	Solutions  = Kinematics::Reverse(input1,input2);
	output[0] = Solutions.at<double>(0,0);
	output[1] = Solutions.at<double>(0,1);
	output[2] = Solutions.at<double>(0,2);
	output[3] = Solutions.at<double>(0,3);
	output[4] = Solutions.at<double>(0,4);
	output[5] = Solutions.at<double>(0,5);

}