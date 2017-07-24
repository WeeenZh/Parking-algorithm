/***************************************************************************

    file                 : user3.cpp
    author            : Xuangui Huang
    email              : stslxg@gmail.com
    description    :  user module for CyberParking

 ***************************************************************************/

/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_parking";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberParking" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	printf("OK!\n");
	return 0;
}

/*
     WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

/* 
    define your variables here.
    following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	//printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
}

static int flag = 0;
static float k,b,dist,dist_C,dist_S,dist_Ang,dist_Angto,dist_Anggo;
static float d_Sp,d_Si,d_Sd,d_Ap,d_Ai,d_Ad,d_S,d_A,d_Sp_,d_C,d_Ap_;
static int hehe1=1;	
static int hehe2=1;
static int flagt = 0;

static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){   //！！！例程为车头入库，需编写车尾入库程序！！！
	/* write your own code here */
	int i = 0; //i修正二三象限角度dist_Angle
	if(_carX - _lotX<0){if(_carY - _lotY<0) i = -1;else i = 1;}
	else{i = 0;}
	dist = sqrt((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY)); //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
	dist_Angto = (atan((_carY - _lotY)/(_carX - _lotX))+PI*i);		//车位指向车辆向量，-PI到PI

	int m =0,n =0;//同i为角度转换系数，计算时输入为0到2*PI
	if(dist_Angto<0)n = 1;else n = 0;
	if(_lotAngle<0)m = 1;else m =0;
	dist_Ang = (_lotAngle+2*PI*m) - (dist_Angto+2*PI*n);//if(dist_Ang>PI){dist_Ang -= PI;}		//车位指向车辆向量与车位朝向夹角,-2*PI到2*PI
	if(_caryaw<0)n=1;else n =0;
	dist_Anggo =(_lotAngle+2*PI*m)-(_caryaw+2*PI*n);		//车位与车辆指向夹角（有向）,-2*PI到2*PI

	dist_C = dist * sin(dist_Ang);		//纵向距离
	dist_S = dist * cos(dist_Ang);		//横向距离



	printf("dist:%.2f, dist_Ang:%.2f, dist_Angto:%.2f, dist_C:%.2f, dist_S:%.2f\nd_Sp:%.2f, d_Si:%.2f, d_Sd:%.2f, d_Ap:%.2f,d_Ai:%.2f,d_Ad:%.2f\n",dist,dist_Ang,dist_Angto,dist_C,dist_S,d_Sp,d_Si,d_Sd,d_Ap,d_Ai,d_Ad);
	printf("dist_Anggo:%.2f, lotX:%.2f, lotY:%.2f, lotAngle:%.2f, carX:%.2f, carY:%.2f, carYaw:%.2f \n",dist_Anggo,_lotX,_lotY,_lotAngle,_carX,_carY,_caryaw);

	if ( flag == -1){}//befinish设置完毕
	else
	{
		if (flag ==1)    //用车位横向坐标，车位角度差值判断是否完成泊车
		{	static float CAac,CAsr,CA_x;//PID计算acc，brake，steer
				d_Sp = sqrt(pow(dist_S,2)+pow(dist_C,2))-sqrt(pow(d_S,2)+pow(d_C,2));	//油门控制p为相对车位的速度
				d_Si = sqrt(pow(dist_S,2)+pow(dist_C,2));
				d_Sd = d_Sp - d_Sp_; 

				
				CA_x = -pow(fabs(dist_C/2.5),15)*(fabs(dist_C)/dist_C);//预瞄点的坐标
				d_Ap = ((atan(CA_x/(dist_S+6))+dist_Anggo)-(atan((-50*pow(d_C/3,31)))+d_A));	//方向控制p为车辆相对预瞄点的角速度
				d_A = dist_Ang;
				d_Ai = (atan((CA_x)/(dist_S+6))+dist_Anggo);
				d_Ad = d_Ap - d_Ap_; 
				
				d_Sp_ = d_Sp;
				d_S = dist_S;
				d_C = dist_C;
				d_Ap_ = d_Ap;

				CAac = d_Sp +d_Si+d_Sd;		//acc,brake计算值
				CAsr = d_Ap/100 + d_Ai + d_Ad;		//steer计算值

			if(((dist_S) * (dist_S) < 0.1||dist_S<0)&&abs(_speed)<0.2){*bFinished = true;flag = -1;}	//设置befinfshed，速度以及纵向距离足够小
			else{	if(fabs(CAsr)>0.2&&dist_S*dist_S>fabs(_speed)){	*cmdAcc =(13/(abs(CAsr)/0.2)+5+_speed);
										*cmdBrake = -(13/(abs(CAsr)/0.2)+5+_speed);
										printf("调整方向");}
					else{	*cmdAcc =(pow((dist_S),2)+3*dist_S+_speed);
							*cmdBrake = -(pow((dist_S),2)+3*dist_S+_speed);
							printf("加速入库");}
					*cmdSteer = -(CAsr)/0.5;
					*cmdGear = -1;
					printf("##CA_x:%.2f/t可能要设置bfinished了\n",CA_x);}}

		else if( ( flag ==2 )) //右转向，完成漂移
		{	printf("##右转向，完成漂移~\n");
			*cmdSteer = 1 ;
			*cmdGear = 1;
			*cmdAcc =0;
			*cmdBrake = 1;
			if(fabs(dist_Anggo) < 0.25 && dist <20||abs(_speed)<5){flag = 1;}}

		else if (flag==3)//向左大转向
		{   *cmdSteer = -1;
			printf("##向左大转向~~~~\n");
			*cmdGear = 1;
			*cmdAcc =0;
			*cmdBrake = 0;
			if(dist_Anggo<PI){if (fabs(fabs(dist_Anggo)-PI/2)>0.3){flag = 2;}}		
			else {if (fabs(fabs(dist_Anggo-2*PI)-PI/2)>0.3){flag = 2;}}}	//车辆指向车位中心时摆正位置

		else if (flag==4) //到一定距离范围并且车子指向车位，上一状态巡线时，将车子调整至与车位90度方向行驶，增加转弯半径
		{	if(abs(dist_Anggo)>PI){*cmdSteer = -(atan2(_width/1.7-dist_S,30)-(PI/2-(2*PI-dist_Anggo)))/3.14/0.2;}//anggo钝角
			else{*cmdSteer = -(atan2(_width/1.5-dist_S,30)-(PI/2+dist_Anggo))/3.14/0.2;}
			printf("β角：%.2f",atan2(_width/1.5-dist_S,30));
			printf("##即将到车位控制速度准备漂移贴左边\n");
			*cmdGear = 1;
			*cmdAcc =-(_speed-140)*dist/50;		//此路段结尾控制速度50到140
			*cmdBrake = (_speed-140)*dist/50;
			flag = 4;
			if((dist<50&&dist_C <13)){flag = 3;}}

		else //其它路段按巡线方式行驶
		{	printf("##巡线行驶\n");
			*cmdAcc = 1;//油门给100%
		    *cmdBrake = 0;//无刹车
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;//设定舵机方向
		    *cmdGear = 1;//档位始终挂1
			flag = 5;
			if(dist<80&&fabs(fabs(dist_Ang)-PI/2)<0.3){flag = 4;}
	    }
	}

	
	if(*bFinished)		//出库
	{*cmdGear = 1;
		if (dist_S<2) //接近停车位时，控制车的朝向与车位一致，速度控制在2 
		{	printf("##直线出库\n");
			*cmdSteer = 20*(dist_Anggo)/3.14 ;
			*cmdAcc =-(_speed-(25*abs(dist_S)+2))/0.3;//控制速度为2，随出库距离加速
			*cmdBrake = (_speed-(25*abs(dist_S)+2))/0.3;}

		else  //其它路段按巡线方式行驶
		{	printf("##出库巡线\n");
			*cmdAcc =-(_speed-30-pow(32,1+(-(_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14)));//控制速度为12，随出库距离加速,随转向减速
			*cmdBrake =(_speed-30-pow(32,1+(-(_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14)));
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14;}//设定舵机方向
		

	}
	


	printf("\tflag:%d\tspeed:%.2f\tdist:%.2f\tflagt:%d\n",flag,_speed,dist,flagt);
	printf("Acc:%.2f\tBrake:%.2f\tSteer:%.2f\n",*cmdAcc,*cmdBrake,*cmdSteer);
	
	/*		printf("XY%.2f\t",(_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
			printf("dY^2%.2f\t",(_carY - _lotY) * (_carY - _lotY));
			printf("dX^2%.2f\n",(_carX-_lotX) * (_carX - _lotX));*/
	printf("-----------------------------------------------------------------------\n");
}
