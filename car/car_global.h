#ifndef _CAR_GLOBAL_H_
#define _CAR_GLOBAL_H_

#include "common.h"
#include "ymath.h"
#include "car_isr.h"
#include "car_init.h"
#include "car_generalfunc.h"
#include "data_process.h"

//临时变量！不用的及时删除！


#define PIT0_MS 5
#define MAXMOTOR 1000
#define FLASH_SAVE_SECTOR 255
#define FLASH_SAVE_SECTOR2 254
#define CAR_SPEED_CONSTANT 0.0390625  //单位转为：转/秒
#define SpeedControl_Period_Const  10       //速度控制周期的时间常量
#define SPEED_CONTROL_COUNT 10
#define DIRECTION_CONTROL_COUNT 2    
#define SDBase 230
#define SpeedMax 800


//#if 1 //全局变量与函数声明★★★★★★★★★★★★★★★★★★★★★★★★★★★

/*
*  结构体定义
*/

typedef enum STATUS_BUTTON
{
    NONE=0,
    PRESS,
    CW,
    CCW,
    UP,
    DOWN,
    PUSH
}STATUS_BUTTON; 

typedef enum STATUS_TRACK 
{
    DoubleLineRoad=0,
    LeftTurnRoad,
    RightTurnRoad,
    CrossRoad,
    RightAngleRoad,
    RATurnDetect,
    RightAngleTurnLeft,
    RightAngleTurnRight,
    LeftBarrier,
    RightBarrier,
    SingleLineRoad,
    RampRoad
}STATUS_TRACK;

typedef struct STATUS_CAR_STRUCT
{
    uint8 g_u8status;         //小车运行状态
    float g_fbatt_volt;     //电池电压
    uint16 g_u16ActiveT;
}STATUS_CAR_STRUCT;

typedef struct indata_STRUCT
{
    uint8 Pixel[128];
    uint8 Pixel2[128];
    //uint8 Pixel3[128];
    MPU6050_DATA_STRUCT mpu6050;
    int32 g_s32LSpeed;
    int32 g_s32RSpeed;
    int32 g_s32LSpeedSum;
    int32 g_s32RSpeedSum;
    int32 g_s32LSpeedSumLast;
    int32 g_s32RSpeedSumLast;
    float g_fAngleAcc;
    float g_fAngleDotBal;
    float g_fAngleDotDir;
    float g_fCarSpeed;                //反馈的现在车速
    float g_fCarAngle;
}indata_STRUCT;

typedef struct setpara_STRUCT
{
    //巡迹
    int CCD1EdgeTH;
    int CCD2EdgeTH;
    //int CCD3EdgeTH;
    //十字
    int WhitePixelMin;
    //直角
    int RALeftBias1;
    int RARightBias1;
    int RALeftBias2;
    int RARightBias2;
    int g_s16RAAngleT;
    int RALD1;
    int RALD2;
    int RARD1;
    int RARD2;
    int RAGyroP1;
    int RAGyroP2;    
    //障碍
    int BarrierLBias;
    int BarrierRBias;
    int BarrierPNum1;
    int BarrierPNum2;
    int BarrierGyroP;
    int BarrGyReT;
    //坡道
    int RampF;
    int RampW;
    int RampRAD;
    int RampSLD;
    int RampBarriD;
    int BiasLim;
    int RampBrakeN;
    int RampSpeed;
    int RampSLCountN; 
    int RampGyroP; 
    int RampSLNum; 
    int RampBarriNum; 
    //过坡道时间
    int RampDNum;    
    //灯塔
    int TowerStopNum;
    //控制
    int RunAngle;
    int BalanceAngle;
    int ACCManualCounterNum;
    int TargetSpeed;
    int DirBaseIni;
    int DirIniT;
    int DirBase;
    int DirCoef;
    int DirKdDifK;
    int LeftTurnLimit;
    int RightTurnLimit;
    int ACCT;
    struct
    {
        int Kp;
        int Kd;
        int KpS;
        int KdS;
    }BalancePD;
    struct 
    {
        int KpStart;
        int KpSteady;
        int Ki;
    }SpeedPI;
    struct
    {
        int KdGyro;
    }DirPD;
    //调试
    int DutyTest;
    int MotorNum;
    int ToScope;
    int SetTime;
    int SendAllPara;
    int AccidentProtect;
    int Expo;
    int GyroAdj;
    int Race;
    int SaveSelect;
    int StartT;
}setpara_STRUCT;

typedef struct track_STRUCT
{
    int g_s16DycMidP;
    int g_s16DycMidP2;
    int g_s16LEdge;
    int g_s16REdge;
    int g_s16LEdge2;
    int g_s16REdge2;
    uint8 g_u8LMissP;
    uint8 g_u8RMissP;
    int g_s16DirBiasNew;
    int g_s16DirBiasNew2;
    int g_s16DirBiasOld;
    int g_s16DirBiasDiff;
    int Edge[2];
    //    int Wide1[10];
    //    int Wide2[10];
    int RoadWidth;
    int RoadWidth2;
    //int g_s16CCD2EdgeP;
    uint8 g_u8LineContinueDetectPause;
    uint8 g_u8LineContinueCounter;
    uint8 g_u8LineContineStop;
}track_STRUCT;

typedef struct outdata_STRUCT
{
    float g_fAngleControlOut;
    float g_fSpeedControlOut;         //速度控制输出给电机的参数
    float g_fDirectionControlOut;
}outdata_STRUCT;

typedef struct OLED_STRUCT
{
    uint8 g_u8changepara;
    uint16 g_u16ParaSelect;
    uint16 g_u16ParaNum;
    uint16 g_u16ParaPrecision;
    int8 g_s8showpage;
    int8 g_s8ShowpageMax;
    int8 g_s8ShowpageMin;
}OLED_STRUCT;

typedef struct PARA_LIST_STRUCT
{
    int* para;
    char label[13];
    uint8 precision;
}PARA_LIST_STRUCT;

typedef struct FLAG_STRUCT
{
    uint8 g_u8SpeedClear;
}FLAG_STRUCT; 

typedef struct SAVE_STRUCT
{
    uint8 g_u8SDBuffer[50][512];
    uint8 g_u8SDSend[512];
    uint32 g_u32SDBufNum;
    int g_s16SDDenoteNum;
    uint8 g_u8SendSD;
    uint8 g_u8ImageCounter;
    uint8 g_u8ImageTimes;
    uint16 g_u16WriteSDErrN;
    uint8 g_u8SDMaxDiff; 
}SAVE_STRUCT;

typedef struct ImageProcess_STRUCT
{
    uint8 g_u8PixelDisplay[128];
    uint8 g_u8PixelDisplay2[128];
    //uint8 g_u8PixelDisplay3[128];
    uint8 g_u8CCDPixelMax;
    uint8 g_u8CCDPixelMin;
    uint8 g_u8CCDPixel2Max;
    uint8 g_u8CCDPixel2Min;
    //    uint8 g_u8CCDPixel3Max;
    //    uint8 g_u8CCDPixel3Min;
    uint8 g_u8ImageRange;
}ImageProcess_STRUCT; 

typedef struct RightAngle_STRUCT
{
    uint8 g_u8RABarNum;
    uint8 g_u8RightAngleScanPause;
    uint8 g_u8RightAngleScanPauseCounter;
    uint32 g_u32RATime;
    uint8 g_u8RADelayL;
    uint8 g_u8RADelayR;
    uint8 g_u8LLoseF;
    uint8 g_u8RLoseF;
    uint8 g_u8RATurnC;
    int g_s16RALeftBias;
    int g_s16RARightBias;
}RightAngle_STRUCT;

typedef struct Barrier_STRUCT
{
    uint8 g_u8BarrierWidth;
    uint8 g_u8BarrierEdge;
    uint8 g_u8BarrierNum;
    uint8 g_u8BarrierCounter;
    int g_s16BarrierGyroRecover;
    uint8 g_u8BarrGyRecoverC;
}Barrier_STRUCT;

typedef struct CONTROL_STRUCT
{
    int g_s16BalanceKp;
    int g_s16BalanceKd;
    int g_s16BalanceAngle;
    int g_s16TargetSpeed;
    float g_fSpeedControlIntegral;    //速度控制的积分值
    float g_fSpeedControlOutOld;      //上一次速度控制输出值
    float g_fSpeedControlOutNew;      //这一次算出的速度控制值
    uint8 g_nSpeedControlCount;           //中断函数中的速度控制计数值
    uint8 g_nDirectionControlCount;
    float g_fDirectionControlOutNew;
    float g_fDirectionControlOutOld;
    int g_s16DirP;
    int g_s16DirBase;
    int g_s16DirKdDiff;
    uint8 g_u8CarACCNum;
    uint8 g_u8CarACC_F;
    uint8 g_u8AngleRecover;
    int g_s16DirGyroP;
}CONTROL_STRUCT;

typedef struct SingleLine_STRUCT
{
    uint8 g_u8SingleLW;
    uint8 g_u8SLLeftWit;
    uint8 g_u8SLRightWit;
    uint8 g_u8SLPosition;
    uint8 g_u8FindSL;
}SingleLine_STRUCT;

typedef struct Ramp_STRUCT
{
    //uint8 g_u8RampNum;
    //    uint8 g_u8RampCount;
    //    int g_s16RampRDiff;
    //    uint8 g_u8RampWid[5];
    //    int8 g_s8Angle[5];
    uint8 g_u8RampOnce;
    uint8 g_u8RampF;
    uint8 g_u8BrakeS;
    uint8 g_u8DelayNum;
    uint8 g_u8DelayNum2;
    uint8 g_u8AngleNum;
    uint8 g_u8RampSLF;
    uint8 g_u8RampSLCount;
    uint8 g_u8RampSLNum;
    uint8 g_u8RampBarrierF;
    uint8 g_u8RampBarrierC;
}Ramp_STRUCT;

typedef struct Tower_STRUCT
{
    uint8 g_u8Tower_vol;
    uint8 g_u8Finish;
    int8 g_s8TowerCounter;
    int8 g_s8TowerCounterL;
    int8 g_s8TowerCounterF;
}Tower_STRUCT;

typedef struct TEST_STRUCT
{
    uint8 g_u8BuzzerRing;
    int g_s16speedsumL;
    int g_s16speedsumR;
    int SpeedP;
    int I;
}TEST_STRUCT;

extern uint32 T;                       //PIT计时器
extern STATUS_BUTTON status_button;    //按钮状态
extern STATUS_TRACK status_track;      //赛道类型 
extern STATUS_CAR_STRUCT mycar;        //小车总状态
extern indata_STRUCT indata;           //输入变量
extern outdata_STRUCT outdata;         //输出变量  
extern setpara_STRUCT setpara;         //设定参数
extern OLED_STRUCT oled;               //屏幕显示
extern track_STRUCT track;             //赛道计算值
extern FLAG_STRUCT flag;               //标志位
extern PARA_LIST_STRUCT paralist[100];
extern ImageProcess_STRUCT imageprocess;
extern CONTROL_STRUCT control;
extern RightAngle_STRUCT rightangle;
extern Barrier_STRUCT barrier;
extern SAVE_STRUCT save;
extern SingleLine_STRUCT single;
extern Ramp_STRUCT ramp;
extern Tower_STRUCT tower;
extern TEST_STRUCT test;
extern uint32 PITValA,PITValB,PITVal;
//extern uint8 RoadWide[63];

extern void status_select();
extern void data_input();
extern void data_process();
extern void data_output();
extern void data_save();

#endif