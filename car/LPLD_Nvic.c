
#include "common.h"
#include "car_global.h"

/*
*  函数声明
*/  

void show_oled(int8 page);
void change_para(char event);
void show_changeable();
void show_upperpage(int8 page);
void save_flash();
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void OutPut_Data(int OutData[4]);
void UART_SendData(uint8_t Data);
void Complement_filter(float angle_m_cf,float gyro_m_cf);
void send_int(int value,char* name);
void SDCCDDataSend();
void AngleDebug();
void BrakingCar();
void SendAllPara();
void CarProtect();
void SaveSD();
void ReadSD();
void key_check();
int SDReadPara(uint16 num);
void SendToScope(uint16 a,uint16 b,uint16 c);
void SendUart();
void GetImage();
void SamplingDelay();
void CCDImageDisplay();
void ImageCapture(uint8 *ImageData);
void PixelMaxMin();
void ImageRecover();
void ImageCapture2(uint8 *ImageData);
//void ImageCapture3(uint8 *ImageData);
void CCDIntegration();
void BrakingInTime();


/*
*  全局变量声明
*/
uint32 T;                       //PIT计时器
STATUS_BUTTON status_button;    //按钮状态
STATUS_TRACK status_track;      //赛道类型 
STATUS_CAR_STRUCT mycar = {0};        //小车总状态
indata_STRUCT indata = {0};           //输入变量
outdata_STRUCT outdata = {0};         //输出变量  
setpara_STRUCT setpara;         //设定参数
OLED_STRUCT oled = {0};               //屏幕显示
FLAG_STRUCT flag = {0};               //标志位
track_STRUCT track = {0};             //赛道计算值
ImageProcess_STRUCT imageprocess = {0}; //图像处理
CONTROL_STRUCT control = {0};
RightAngle_STRUCT rightangle = {0};
Barrier_STRUCT barrier = {0};
SAVE_STRUCT save = {0};
SingleLine_STRUCT single = {0};
Ramp_STRUCT ramp = {0}; 
Tower_STRUCT tower = {0};
TEST_STRUCT test = {0};
uint32 PITValA,PITValB,PITVal;
int OutData[4] = {0};


uint16 Cos4Data[128] = {5801,5801,5801,5801,6073,6268,6600,6787,7068,7304,7556,7779,
                             7856,8021,8148,8351,8601,8556,8731,8726,8794,8920,8931,8954,
                                  8993,9078,9058,9091,9087,9184,9111,9254,9220,9269,9264,9247,
                                       9256,9327,9305,9474,9384,9356,9493,9405,9485,9566,9503,9599,
                                            9577,9584,9645,9634,9619,9703,9618,9692,9659,9721,9808,9860,
                                                 9828,9853,9938,9999,9821,9589,9372,9400,9707,9934,9814,9822,
                                                      9763,9806,9813,9828,9801,9777,9708,9725,9743,9690,9589,9698,
                                                           9600,9631,9503,9532,9449,9506,9472,9459,9416,9452,9398,9351,
                                                                9195,9197,9154,9141,9102,9062,8996,8958,8905,8747,8730,8630,
                                                                     8470,8292,8112,7970,7668,7616,7294,7203,6950,6745,6386,6155,
                                                                     5830,5550,5182,4943,4501,4501,4501,4501};
uint8 DarkCurrent[128] = {40,40,40,40,41,40,41,40,41,40,41,40,41,40,41,40,41,40,42,
                             41,42,41,42,41,42,41,42,41,42,41,42,41,42,41,42,41,42,41,
                                42,41,42,42,42,41,43,42,43,42,43,42,43,42,43,42,43,42,43,
                                   42,43,42,43,42,43,42,43,42,43,42,43,42,43,42,43,43,43,42,
                                      43,42,43,42,43,42,43,42,43,42,43,42,43,42,43,42,43,42,43,
                                         42,43,42,43,42,43,42,43,42,43,42,43,42,43,42,43,42,43,42,
                                            43,42,43,42,43,42,43,42,43,42,43,43,43,43};

PARA_LIST_STRUCT paralist[100]=      //可调参数表
{
    //比赛时要调的参数
    //模式
    {&setpara.Race,"Race",1},
    //坡道
    //哪种元素后减速标志
    {&setpara.RampF,"RampF",1},
    {&setpara.RampDNum,"RampUseT",1},
    {&setpara.RampSpeed,"RampSPD",1},
    {&setpara.RampRAD,"RampRAD",1},
    {&setpara.BiasLim,"PoBiasLim",1},
    //障碍
    {&setpara.BarrierLBias,"BariLBias",1},
    {&setpara.BarrierPNum1,"BariPNumL",1},
    {&setpara.BarrierRBias,"BariRBias",1},    
    {&setpara.BarrierPNum2,"BariPNumR",1},
    //直角
    {&setpara.RARightBias1,"RARightBias1",1},
    {&setpara.RAGyroP1,"RAGyroP1",1},
    {&setpara.RARightBias2,"RARightBias2",1},
    {&setpara.RAGyroP2,"RAGyroP2",1},
    {&setpara.RALeftBias1,"RALeftBias1",1},
    {&setpara.RALeftBias2,"RALeftBias2",1},
    //单线
    //灯塔
    {&setpara.SetTime,"TowerTime",1},
    {&setpara.TowerStopNum,"TowStopNum",1},
    //十字
    {&setpara.WhitePixelMin,"WhitePM",1}, 
    
    //坡道
    {&setpara.RampBrakeN,"RampBAngleN",1},
    {&setpara.RampGyroP,"RampGyP",1},
    {&setpara.RampSLNum,"RampSLNum",1},
    {&setpara.RampSLCountN,"RampSLCN",1},
    {&setpara.RampSLD,"RampSLD",1}, 
    {&setpara.RampBarriNum,"RampBarriN",1},
    {&setpara.RampBarriD,"RampBarriD",1},
    
    //障碍
    {&setpara.BarrierGyroP,"BarrGyP",1},
    {&setpara.BarrGyReT,"BarrGyReT",1},
    //直角
    {&setpara.RALD1,"RALD1",1},
    {&setpara.RALD2,"RALD2",1},
    {&setpara.RARD1,"RARD1",1},
    {&setpara.RARD2,"RARD2",1},
    
    //中间是不用调的平衡速度PID，还有元素中几乎不用变的参数
    {&setpara.RunAngle,"RunAngle",1},
    {&setpara.BalanceAngle,"BalAngle",1},
    {&setpara.ACCManualCounterNum,"ACCManualC",1},
    {&setpara.BalancePD.Kp,"BalanceKp",1},
    {&setpara.BalancePD.Kd,"BalanceKd",1},
    {&setpara.TargetSpeed,"TargetSpeed",1},
    {&setpara.SpeedPI.KpStart,"SpeedKPST",1},
    {&setpara.SpeedPI.KpSteady,"SpeedKPSD",1},
    {&setpara.SpeedPI.Ki,"SpeedKI",1},    
    {&setpara.LeftTurnLimit,"LTurnLimit",1},
    {&setpara.RightTurnLimit,"RTurnLimit",1},
    {&setpara.DutyTest,"DutyTest",1},
    {&setpara.MotorNum,"MotorNum",1},
    {&setpara.SendAllPara,"SendAllPara",1},
    {&setpara.AccidentProtect,"Protect",1},
    {&save.g_s16SDDenoteNum,"SDNum",1},
    {&setpara.GyroAdj,"GyroAdj",1},
    {&setpara.SaveSelect,"SaveSelect",1},
    
    //调试
    {&setpara.ToScope,"ToScope",1},
    //赛前试车要调好的参数
    {&setpara.ACCT,"ACCT",1},
    {&setpara.Expo,"Expo",1},
    {&setpara.BalancePD.KpS,"BalanceKPS",1},
    {&setpara.BalancePD.KdS,"BalanceKDS",1},
    {&setpara.DirKdDifK,"DirKdDifK",1},
    {&setpara.DirPD.KdGyro,"DirKdGyro",1},
    {&setpara.DirCoef,"DirCoef",1},
    {&setpara.DirBase,"DirBase",1},
    {&setpara.DirIniT,"DirIniT",1},
    {&setpara.DirBaseIni,"DirBaseIni",1},
    {&setpara.CCD2EdgeTH,"CCD2EdgeTH",1},
    {&setpara.CCD1EdgeTH,"CCD1EdgeTH",1},
    {&setpara.StartT,"StartT",1},
    
    {0}
};


void main()
{    
    DisableInterrupts;
    init_all();           //小车所有的初始化函数
    EnableInterrupts;
    
    while(1)
    {     
        if(test.g_u8BuzzerRing)
        {
            PTB21_O = 1;
        }
        else
        {
            PTB21_O = 0;
        }
        
        if(mycar.g_u8status == 0)  
        {
            key_check();
            show_oled(oled.g_s8showpage);
        }
        
        BrakingInTime();
        
        SaveSD();
        
        if(save.g_u8SendSD == 1)
        {           
            ReadSD();
            save.g_u8SendSD = 0;
        }
        
        if(setpara.AccidentProtect)
        {
            CarProtect();
        }
        
        if(setpara.SendAllPara)
        {
            SendAllPara();
            setpara.SendAllPara = 0;
        }
        
        //SendUart();
        //motor_test();      
        //motor_deadthreshold_test();
        //AngleDebug();
    }
}


void key_check()
{
    //记录按键时间
    uint32 pushtime=T;
    
    //旋钮或拨轮按下操作后屏幕初始化，以修复花屏
    if(status_button==PRESS||status_button==PUSH)
        OLED_Init();        
    
    switch(status_button)
    {
      case PRESS:
        while(!PTC3_I);
        if(T-pushtime<300)
        {
            oled.g_u8changepara ^= 1;    //状态取反
        }
        else
        {
            T = 0;
            status_track = DoubleLineRoad;
            memset(&flag,0,sizeof(struct FLAG_STRUCT));
            memset(&rightangle,0,sizeof(struct RightAngle_STRUCT));
            memset(&save,0,sizeof(struct SAVE_STRUCT));
            memset(&control,0,sizeof(struct CONTROL_STRUCT));
            //最后开始发车 避免未初始化变量进中断
            mycar.g_u8status = 1;
            mycar.g_u16ActiveT = T;
            //必须写在清零后面！
            control.g_s16DirGyroP = setpara.DirPD.KdGyro;
            track.g_s16DycMidP = 64;
            track.g_s16DycMidP2 = 64;
        }
        break;
      case PUSH:
        while(!PTC6_I);
        if(T-pushtime<800)
        {
            oled.g_u16ParaPrecision *= 10;
            if(oled.g_u16ParaPrecision == 1000)
                oled.g_u16ParaPrecision = 1;
        }
        else
        {
            save.g_u8SendSD =1;
        }
        break;
      case UP:
        while(!PTC4_I);
        if(T-pushtime<300)
        {
            if(oled.g_u8changepara)   //参数改变状态
            {
                if(oled.g_u16ParaSelect >0)
                    oled.g_u16ParaSelect --;
                else
                    oled.g_u16ParaSelect = oled.g_u16ParaNum-1;
            }
            else                 //参数选择状态
            {
                if(oled.g_s8showpage > oled.g_s8ShowpageMin)
                    oled.g_s8showpage --;
                else
                    oled.g_s8showpage = 0;
            }
        }
        else
        {
        }
        break;  
      case DOWN:
        while(!PTC8_I);
        if(T-pushtime<300)
        {
            if(oled.g_u8changepara)   //参数改变状态
            {
                if(oled.g_u16ParaSelect <oled.g_u16ParaNum-1)
                    oled.g_u16ParaSelect ++;
                else
                    oled.g_u16ParaSelect = 0;
            }
            else                 //参数选择状态
            {
                if(oled.g_s8showpage < oled.g_s8ShowpageMax)
                    oled.g_s8showpage ++;
                else
                    oled.g_s8showpage = 0;
            }
        }
        else
        {
            save_flash();
            
        }
        break;
      default:
        break;
    }
    //清除按键状态
    status_button = NONE;
    
}

void UART_SendData(uint8_t Data)
{
    while(!(UART0->S1 & UART_S1_TDRE_MASK));
    UART0->D = (uint8_t)Data;
}

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;
    
    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

void OutPut_Data(int OutData[4])
{
    int temp[4] = {0};
    unsigned int temp1[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;
    for(i=0;i<4;i++)
    {
        
        temp[i]  = (int)OutData[i];
        temp1[i] = (unsigned int)temp[i];
        
    }
    
    for(i=0;i<4;i++) 
    {
        databuf[i*2]   = (unsigned char)(temp1[i]%256);
        databuf[i*2+1] = (unsigned char)(temp1[i]/256);
    }
    
    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16%256;
    databuf[9] = CRC16/256;
    
    for(i=0;i<10;i++)
        UART_SendData(databuf[i]);
}

void show_changeable()
{
    int temp_para_select = oled.g_u16ParaSelect;      //由于潜在的中断干扰，必须先存储参数序号以避免危险
    if(temp_para_select>0)
    {
        oledprintf(5,0,"%02d.%-13s",temp_para_select-1,paralist[temp_para_select-1].label);
        oledprintf(5,96,"%5d",*paralist[temp_para_select-1].para);
    }
    else
    {
        LCD_ClearLine(5);
    }
    if(oled.g_u8changepara)
    {
        oledprintf(6,0,"%02d.%-13s",temp_para_select,paralist[temp_para_select].label);
        oledprintfw(6,96,"%5d",*paralist[temp_para_select].para);
    }
    else
    {  
        oledprintfw(6,0,"%02d.%-13s",temp_para_select,paralist[temp_para_select].label);
        oledprintf(6,96,"%5d",*paralist[temp_para_select].para);
    }
    if(temp_para_select<oled.g_u16ParaNum-1)
    {
        oledprintf(7,0,"%02d.%-13s",temp_para_select+1,paralist[temp_para_select+1].label);
        oledprintf(7,96,"%5d",*paralist[temp_para_select+1].para);
    }  
    else
    {
        LCD_ClearLine(7);
    }
}

void show_upperpage(int8 page)
{
    static int lastpage2;
    float SpeedOledShow;
    if(lastpage2!=page)
    {
        for(int i=0;i<5;LCD_ClearLine(i++));        //换页前清行
        lastpage2=page;
    }
    
    switch(page)
    {
      case 0:
        SpeedOledShow = (float)setpara.TargetSpeed*512.0/6075.0;
        oledprintf(0,0,"BAT:"); 
        LCD_PrintValueF(26,0,mycar.g_fbatt_volt,2);
        oledprintf(0,65,"SPD:"); 
        LCD_PrintValueF(91,0,SpeedOledShow,2);
        oledprintf(1,0,"%4d",(int)indata.g_fCarAngle);
        oledprintf(1,30,"%4d",(int)indata.g_fAngleAcc);
        oledprintf(1,60,"%4d",(int)indata.g_fAngleDotBal); 
        oledprintf(1,90,"%4d",(int)indata.g_fAngleDotDir);
        oledprintf(2,0,"%4d",imageprocess.g_u8CCDPixelMax);
        oledprintf(2,30,"%4d",imageprocess.g_u8CCDPixelMin);
        oledprintf(2,60,"%4d",imageprocess.g_u8CCDPixel2Max);
        oledprintf(2,90,"%4d",imageprocess.g_u8CCDPixel2Min);  
        oledprintf(3,0,"PCS:%3d",oled.g_u16ParaPrecision);
        oledprintf(3,50,"Mode:%2d",status_track);
        oledprintf(3,100,"%2d",setpara.SaveSelect);
        oledprintf(4,0,"RoadW:%4d",track.RoadWidth);
        oledprintf(4,70,"RW2:%4d",track.RoadWidth2);
        //        oledprintf(4,65,"CCD2:%3d",track.g_s16CCD2EdgeP); 
        break;
      case 1:
        oledprintf(1,0,"SpdL:%3d",indata.g_s32LSpeed);
        oledprintf(1,60,"SpdR:%3d",indata.g_s32RSpeed);
        oledprintf(2,0,"LCell:%2d",PTB2_I);
        oledprintf(2,60,"RCell:%2d",PTB10_I); 
        oledprintf(3,0,"SDDif:%2d",save.g_u8SDMaxDiff); 
        oledprintf(3,60,"WErr:%5d",save.g_u16WriteSDErrN); 
        oledprintf(4,0,"OE:%2d",rightangle.g_u8RABarNum); 
        oledprintf(4,60,"BariW:%2d",barrier.g_u8BarrierWidth); 
        break;
      case 2:
        oledprintf(0,0,"BariE:%2d",barrier.g_u8BarrierEdge); 
        oledprintf(0,60,"BariNum:%2d",barrier.g_u8BarrierNum);  
        oledprintf(1,0,"SLWid:%2d",single.g_u8SingleLW);
        oledprintf(1,60,"SLLWit:%2d",single.g_u8SLLeftWit);
        oledprintf(2,0,"SLRWit:%2d",single.g_u8SLRightWit);
        oledprintf(2,60,"SDNum:%4d",save.g_s16SDDenoteNum);
        oledprintf(3,0,"RCM:%02X",RCM->SRS1);//RCM->SRS1
        oledprintf(3,40,"%02X",RCM->SRS0);//RCM->SRS0
        oledprintf(4,0,"tower:%3d",(int)tower.g_u8Tower_vol);
        //        oledprintf(3,0,"SpdL:%10d",test.g_s32speedsumL);
        //        oledprintf(4,0,"SpdR:%10d",test.g_s32speedsumR);
        break;
      default:
        break;
    }
}


void send_int(int value,char* name)
{
    printf("%s",name);
    printf(" = ");
    printf("%d;\n",value);
}

void show_oled(int8 page)
{
    static int lastpage1;
    if(lastpage1!=page)
    {
        LCD_CLS();
        lastpage1=page;
        oled.g_u16ParaSelect = 0;
    }   
    if(save.g_u8SendSD != 1) 
    {
        if(page >=0)
        {
            show_upperpage(page);
            show_changeable();
        }
        if(page == -1)
        {
            LCD_CCD(imageprocess.g_u8PixelDisplay);
        } 
        if(page == -2)
        {
            LCD_CCD(imageprocess.g_u8PixelDisplay2);
        } 
        //        if(page == -3)
        //        {
        //            LCD_CCD(imageprocess.g_u8PixelDisplay3);
        //        } 
    }
}

void MotorOutput()
{   
    float fLeft,fRight;
    
    fLeft = outdata.g_fAngleControlOut;
    fRight = outdata.g_fAngleControlOut;
    
    fLeft += (outdata.g_fSpeedControlOut - outdata.g_fDirectionControlOut);  
    fRight += (outdata.g_fSpeedControlOut + outdata.g_fDirectionControlOut); 
    
    if(fLeft >= MAXMOTOR)fLeft = MAXMOTOR;
    if(fRight >= MAXMOTOR)fRight = MAXMOTOR;
    if(fLeft <= -MAXMOTOR)fLeft = -MAXMOTOR;
    if(fRight <= -MAXMOTOR)fRight = -MAXMOTOR;
    
    if(fRight>0)
    {
        LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0); 
        LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, (int)fRight); 
    }
    else
    {   
        LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, -(int)fRight); 
        LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0); 
    }
    
    if(fLeft>0)
    {
        LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0); 
        LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, (int)fLeft); 
    }
    else
    { 
        LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, -(int)fLeft); 
        LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0); 
    }
    
}

void Upright_Control_calculate()	
{   
    float fValue;
    
    //1.陀螺仪加低通滤波效果好一些
    //2.下次调角度融合试试MPU6050自带的低通滤波
    //3.阻尼力与回复力方向相反
    fValue =  control.g_s16BalanceKp*(indata.g_fCarAngle-(float)control.g_s16BalanceAngle/10) - control.g_s16BalanceKd*indata.g_fAngleDotBal/10;
    
    if(fValue > 1000)
        fValue = 1000;
    if(fValue < -1000)
        fValue = -1000;
    
    outdata.g_fAngleControlOut  = fValue;	
}

void Complement_filter(float angle_m_cf,float gyro_m_cf)
{
    static float dt = 0.005;
    static float  LowPassCoefficient  = 0.98;        
    
    indata.g_fCarAngle = (indata.g_fCarAngle + gyro_m_cf * dt) * LowPassCoefficient + angle_m_cf * (1.0 - LowPassCoefficient);
}

void IMU_Update(float Gyro,float Acc)
{  
    Complement_filter((float)Acc, (float)Gyro);//角度 角速度
}

void SpeedControl() 
{
    //计算中间变量
    float fP = 0.0;
    float fI = 0.0;
    float fDelta = 0.0;
    //编码器脉冲求和值转换成现在的车速
    
    if(indata.g_s32LSpeedSum*100/indata.g_s32RSpeedSum >= 200 || indata.g_s32LSpeedSum*100/indata.g_s32RSpeedSum <= 50)
    {
        indata.g_s32LSpeedSum = indata.g_s32LSpeedSumLast;
        indata.g_s32RSpeedSum = indata.g_s32RSpeedSumLast;
    }
    
    if(status_track == RampRoad)
    {
        if(indata.g_s32LSpeedSum*100/indata.g_s32RSpeedSum >= 150 || indata.g_s32LSpeedSum*100/indata.g_s32RSpeedSum <= 67)
        {
            indata.g_s32LSpeedSum = indata.g_s32LSpeedSumLast;
            indata.g_s32RSpeedSum = indata.g_s32RSpeedSumLast;
        }
    }
    
    indata.g_fCarSpeed = (indata.g_s32LSpeedSum+indata.g_s32RSpeedSum)*CAR_SPEED_CONSTANT/2;
    
    if(T-mycar.g_u16ActiveT > setpara.ACCT && mycar.g_u8status == 3)
    {
        if(indata.g_fCarSpeed > control.g_s16TargetSpeed+10)
            indata.g_fCarSpeed = control.g_s16TargetSpeed + 10;
        if(indata.g_fCarSpeed < control.g_s16TargetSpeed-10)
            indata.g_fCarSpeed = control.g_s16TargetSpeed - 10;
    }
    
    indata.g_s32LSpeedSumLast = indata.g_s32LSpeedSum;
    indata.g_s32RSpeedSumLast = indata.g_s32RSpeedSum;
    
    test.g_s16speedsumL = (int)(indata.g_s32LSpeedSum * CAR_SPEED_CONSTANT);
    test.g_s16speedsumR = (int)(indata.g_s32RSpeedSum * CAR_SPEED_CONSTANT);
    //test.SpeedP = test.g_s16speedsumL*100/test.g_s16speedsumR;
    
    indata.g_s32LSpeedSum = indata.g_s32RSpeedSum = 0;
    
    //抬轮PID修正 还要修改
    //应进行抬轮速度限制和防止倒转！！！
    
    
    fDelta = control.g_s16TargetSpeed - indata.g_fCarSpeed;
    
    //变速比例
    //if(abs_f(fDelta)>2)
    //{
    if(mycar.g_u8status == 2)
    {
        fP = fDelta * setpara.SpeedPI.KpStart;
    }
    else if(mycar.g_u8status == 3)
        fP = fDelta * setpara.SpeedPI.KpSteady;
    
    //一开始也可能
    if(fP > SpeedMax)
        fP = SpeedMax;
    if(fP < -SpeedMax)
        fP = -SpeedMax;
    
    test.SpeedP = (int)fP;
    
    //}
    //else
    //fP = fDelta * setpara.SpeedPI.KpSteady * 2;
    
    
    //积分分离
    if(abs_f(fDelta) > 10 && (T-mycar.g_u16ActiveT) < setpara.ACCT)
        fI = 0;
    else  
    {
        fI = fDelta * setpara.SpeedPI.Ki / 10;
        
        control.g_fSpeedControlIntegral += fI;
        
        //        if(abs_f(control.g_fSpeedControlIntegral) >= ((float)SpeedMax-abs_f(fP)))
        //        {
        //            if(control.g_fSpeedControlIntegral>0)
        //            {
        //                control.g_fSpeedControlIntegral = (float)SpeedMax-abs_f(fP);
        //            }
        //            if(control.g_fSpeedControlIntegral<0)
        //            {
        //                control.g_fSpeedControlIntegral = -((float)SpeedMax-abs_f(fP));
        //            }
        //        }
        if(control.g_fSpeedControlIntegral > 220)
            control.g_fSpeedControlIntegral = 220;
        if(control.g_fSpeedControlIntegral < -220)
            control.g_fSpeedControlIntegral = -220;
    }
    
    control.g_fSpeedControlOutOld = control.g_fSpeedControlOutNew;    
    
    control.g_fSpeedControlOutNew = fP + control.g_fSpeedControlIntegral;
    
}

//速度控制输出
void SpeedControlOutput() 
{
    float fValue = 0.0;
    //求出两次速度控制计算的增量
    fValue = control.g_fSpeedControlOutNew - control.g_fSpeedControlOutOld;
    //实际的输出值计算
    outdata.g_fSpeedControlOut = fValue*(control.g_nSpeedControlCount+1)/SpeedControl_Period_Const + control.g_fSpeedControlOutOld;
    
    if((T-mycar.g_u16ActiveT) > setpara.ACCT && mycar.g_u8status == 3)
    {
        if(outdata.g_fSpeedControlOut > 600)
            outdata.g_fSpeedControlOut = 600;
        if(outdata.g_fSpeedControlOut < -600)
            outdata.g_fSpeedControlOut = -600;
    }
    
    //    if((T-mycar.g_u16ActiveT) > 5000)
    //    {
    //        if(outdata.g_fSpeedControlOut > 200)
    //            outdata.g_fSpeedControlOut = 200;
    //        if(outdata.g_fSpeedControlOut < -200)
    //            outdata.g_fSpeedControlOut = -200;
    //    }
    
}


void DirectionControlOutput() 
{
    float fValue;
    fValue = control.g_fDirectionControlOutNew - control.g_fDirectionControlOutOld;
    outdata.g_fDirectionControlOut = fValue * (control.g_nDirectionControlCount + 1) / DIRECTION_CONTROL_COUNT + control.g_fDirectionControlOutOld;
}

void DirectionControl()
{
    ccd_cal();
}

void data_input()
{
    if(T%500 == 0 && mycar.g_u8status == 0)
        mycar.g_fbatt_volt = LPLD_ADC_Get(ADC0,AD23)*(9.9*1.147/256);                           //电池电压检查
    
    indata.g_s32LSpeed = (int16)LPLD_FTM_GetCounter(FTM1)/2;
    //防倒转
    if(indata.g_s32LSpeed <0)
        indata.g_s32LSpeed = 0;
    LPLD_FTM_ClearCounter(FTM1);
    indata.g_s32RSpeed = -(int16)LPLD_FTM_GetCounter(FTM2)/2;
    if(indata.g_s32RSpeed <0)
        indata.g_s32RSpeed = 0;
    LPLD_FTM_ClearCounter(FTM2);
    indata.g_s32LSpeedSum += indata.g_s32LSpeed;
    indata.g_s32RSpeedSum += indata.g_s32RSpeed;
    if(control.g_nDirectionControlCount == 1 && setpara.Expo == 5)
        CCDIntegration();
    if(control.g_nDirectionControlCount >= DIRECTION_CONTROL_COUNT)
        GetImage();
    indata.mpu6050 = MPU6050_GetData();                         //获取MPU6050数据
    indata.g_fAngleAcc = -indata.mpu6050.acc_z*180/16384;
    indata.g_fAngleDotBal = (indata.mpu6050.gyr_x+92)/65.5+setpara.GyroAdj;
    indata.g_fAngleDotDir = -(indata.mpu6050.gyr_y-145)/131;
}     


void save_flash()
{
    uint32 data_to_write[100];
    for(int i=0;i<100;i++)
        data_to_write[i] = *paralist[i].para;
    if(PTD1_I == 0 && setpara.SaveSelect == 0)
    {
        LPLD_Flash_SectorErase(FLASH_SAVE_SECTOR*0x800);
        LPLD_Flash_ByteProgram(FLASH_SAVE_SECTOR*0x800,data_to_write,oled.g_u16ParaNum*4);
        LCD_CLS();
        LCD_P6x8Str(40,4,"Save Done !");
        LPLD_LPTMR_DelayMs(500);
        LCD_CLS();
    }
    if(PTD1_I == 1 && setpara.SaveSelect == 1)
    {
        LPLD_Flash_SectorErase(FLASH_SAVE_SECTOR2*0x800);
        LPLD_Flash_ByteProgram(FLASH_SAVE_SECTOR2*0x800,data_to_write,oled.g_u16ParaNum*4);
        LCD_CLS();
        LCD_P6x8Str(40,4,"Save Done !");
        LPLD_LPTMR_DelayMs(500);
        LCD_CLS();
    }
}

void change_para(char event) 
{    
    if(oled.g_s8showpage >= 0)
    {
        switch(event)
        {                
          case 1:
            *paralist[oled.g_u16ParaSelect].para += paralist[oled.g_u16ParaSelect].precision*oled.g_u16ParaPrecision;
            break;
          case 2:
            *paralist[oled.g_u16ParaSelect].para -= paralist[oled.g_u16ParaSelect].precision*oled.g_u16ParaPrecision;
            break;
          default:
            break;
        }
    }
    else
    {
        switch(event)
        {
          case 1:
            imageprocess.g_u8ImageRange=(imageprocess.g_u8ImageRange+1)%3;
            break;
          case 2:
            imageprocess.g_u8ImageRange=(imageprocess.g_u8ImageRange-1)%3;
            break;
          default:
            break;
        }
    }
} 

void SDCCDDataSend()
{
    if(setpara.ToScope == 0)
    {
        for(int i=0;i<108;i++)
        {
            OutData[0] = (int)save.g_u8SDSend[i];
            OutData[1] = (int)save.g_u8SDSend[i+122];
            if(i<60)
                OutData[2] = (int)SDReadPara(5);
            else
                OutData[2] = (int)SDReadPara(6);
            OutPut_Data(OutData);
        }
        for(int i=108;i<122;i++)
        {
            OutData[0] = (int)save.g_u8SDSend[i];
            OutData[1] = 0;
            OutData[2] = 0;
            OutPut_Data(OutData);
        }
        OutData[0] = 400;
        OutData[1] = 400;
        OutData[2] = 400;
        OutPut_Data(OutData);
        OutData[3] = SDReadPara(0); //图像编号
        OutPut_Data(OutData);
        OutData[3] = SDReadPara(1); //图像编号
        OutPut_Data(OutData);
        OutData[0] = -50;
        OutData[1] = -50;
        OutData[2] = -50;
        OutPut_Data(OutData);  
    }
    if(setpara.ToScope == 1)
    {
        SendToScope(7,3,4);
    }
    if(setpara.ToScope == 2)
    {
        SendToScope(24,25,26);
    }
    if(setpara.ToScope == 3)
    {
        SendToScope(27,28,32);
    }
    if(setpara.ToScope == 4)
    {
        SendToScope(19,20,15);
    }
    if(setpara.ToScope == 5)
    {
        SendToScope(8,18,40);
    }
    if(setpara.ToScope == 6)
    {
        SendToScope(9,13,14);
    }
    //ActiveT
    if(setpara.ToScope == 7)
    {
        SendToScope(10,24,30);
    }
    if(setpara.ToScope == 8)
    {
        SendToScope(31,41,42);
    }
    //速度pid
    if(setpara.ToScope == 9)
    {
        SendToScope(36,37,38);
    }
    if(setpara.ToScope == 10)
    {
        SendToScope(6,15,4);
    }
    if(setpara.ToScope == 11)
    {
        SendToScope(36,43,44);
    }
    //角度
    if(setpara.ToScope == 12)
    {
        SendToScope(10,7,45);
    }
    if(setpara.ToScope == 13)
    {
        SendToScope(46,38,39);
    }
    if(setpara.ToScope == 14)
    {
        SendToScope(2,19,20);
    }
    if(setpara.ToScope == 15)
    {
        SendToScope(6,15,7);
    }
    if(setpara.ToScope == 16)
    {
        SendToScope(2,47,7);
    }
    if(setpara.ToScope == 17)
    {
        SendToScope(48,49,50);
    }
#if 0
    //速度
    SDSavePara((int)test.SpeedP,36);      
    SDSavePara((int)control.g_fSpeedControlIntegral,37);  
    SDSavePara((int)outdata.g_fSpeedControlOut,38); 
    //1
    SDSavePara((int)status_track,7);
    SDSavePara((int)indata.g_fCarAngle,3);
    SDSavePara((int)indata.g_fCarSpeed,4);
    //赛道宽 左右边线 看连续性
    SDSavePara((int)track.RoadWidth,15);
    SDSavePara((int)track.g_s16LEdge,19);
    SDSavePara((int)track.g_s16REdge,20);  
    //2 3单线
    SDSavePara((int)single.g_u8SingleLW,24);
    SDSavePara((int)single.g_u8SLLeftWit,25);
    SDSavePara((int)single.g_u8SLRightWit,26);
    SDSavePara((int)single.g_u8SLPosition,27);
    SDSavePara((int)single.g_u8FindSL,28);
    
    SDSavePara((int)track.g_u8LineContinueDetectPause,32);
    
    //障碍
    SDSavePara((int)barrier.g_u8BarrierEdge,18);
    SDSavePara((int)barrier.g_u8BarrierNum,40);
    
    //ActiveT
    SDSavePara((int)mycar.g_u16ActiveT,10); 
    SDSavePara((int)tower.g_u8Tower_vol,24); 
    SDSavePara((int)mycar.g_u8status,30); 
    SDSavePara((int)tower.g_s8TowerCounterF,31); 
    SDSavePara((int)tower.g_u8Finish,41); 
    SDSavePara((int)tower.g_s8TowerCounter,42);
#endif
    //    if(setpara.ToScope == 10)
    //    {
    //        for(int i=0;i<122;i++)
    //        {
    //            printf("%d",save.g_u8SDSend[i]);
    //            printf(" ");
    //        }
    //        printf("\n");
    //    }
    //    if(setpara.ToScope == 8)
    //    {
    //        for(int i=0;i<108;i++)
    //        {
    //            printf("%d",save.g_u8SDSend[i+122]);
    //            printf(" ");
    //        }
    //        printf("\n");
    //    }
}

void AngleDebug()
{
    OutData[0] = (int)indata.g_fCarAngle;
    OutData[1] = (int)indata.g_fAngleDotBal;
    OutData[2] = (int)indata.g_fAngleAcc;
    OutData[3] = (int)indata.g_fAngleDotDir;
    OutPut_Data(OutData);
}

void BrakingCar()
{
    LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
    LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
    LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
    LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
}

void SendAllPara()
{
    for(int i=0;i<oled.g_u16ParaNum;i++)
        send_int(*paralist[i].para,paralist[i].label);
}

void CarProtect()
{
    //大于25度陀螺仪漂移会发生变化！！！
    if(indata.g_fCarAngle > 50 || indata.g_fCarAngle < -50)
    {
        mycar.g_u8status = 0;
    }
    
    //7.7V为实测！！！
    //    if(mycar.g_fbatt_volt < 7.7 && mycar.g_fbatt_volt > 4)
    //        test.g_u8BuzzerRing = 1;
    //    else          
    //        test.g_u8BuzzerRing = 0;
    
    
    
    //    if(PTD1_I==1 || PTD2_I==1 || PTD3_I==1)
    //        PTB21_O = 1;
    //    else
    //        PTB21_O = 0;
}

void SaveSD()
{
    uint8 sd_status=0;
    while(save.g_s16SDDenoteNum<save.g_u32SDBufNum)
    {
        BrakingInTime(); //放在这里防止多次写SD卡循环不能及时停车
        
        sd_status = disk_write (0, save.g_u8SDBuffer[save.g_s16SDDenoteNum%50], save.g_s16SDDenoteNum, 1); //3.2ms 从0扇区开始 
        if(sd_status == 0 || save.g_u8SDMaxDiff >= 50) //如果缓存爆了就跳过不存SD
            save.g_s16SDDenoteNum ++;
        
        //    //写入错误验证
        if(sd_status)
        {
            save.g_u16WriteSDErrN ++;
            //        asm("nop");
            //        oledprintf(0,0,"SD WRITE ERROR! %1d    ",sd_status);
            //        while(1);
        }
        //追尾验证
        if(save.g_u8SDMaxDiff < save.g_u32SDBufNum - save.g_s16SDDenoteNum)
            save.g_u8SDMaxDiff = save.g_u32SDBufNum - save.g_s16SDDenoteNum;
    }
}

void ReadSD()
{
    DisableInterrupts;
    //uint8 sd_status=0;
    //static int SDReadErrorCount;
    for(int i=0;i<save.g_s16SDDenoteNum;i++)
    {
        while(disk_read (0, save.g_u8SDSend,i, 1));
        //读取错误检查
        //        if(sd_status)
        //        {
        //            SDReadErrorCount++;
        //            asm("nop");
        //            oledprintfw(0,0,"SD READ ERROR! %3d ",SDReadErrorCount);
        //            //while(1);
        //        }
        SDCCDDataSend();
    }
    EnableInterrupts;
}

int SDReadPara(uint16 num)
{
    int l_s16Para;
    l_s16Para = (int16)(save.g_u8SDSend[SDBase+2*num]*256 + save.g_u8SDSend[SDBase+2*num+1]);
    return l_s16Para;
}

void SendToScope(uint16 a,uint16 b,uint16 c)
{
    OutData[0] = SDReadPara(0);
    OutPut_Data(OutData);  
    OutData[0] = SDReadPara(1);
    OutData[1] = SDReadPara(a); 
    OutData[2] = SDReadPara(b); 
    OutData[3] = SDReadPara(c); 
    OutPut_Data(OutData); 
}

void SendUart()
{
    printf("C:%d",save.g_u32SDBufNum);
    printf("\t");
    printf("N:%d",save.g_s16SDDenoteNum);
    printf("\n");
    
    //    OutData[0] = (int8)indata.g_fCarSpeed;
    //    OutData[1] = (int8)indata.g_fCarAngle; 
    //    OutPut_Data(OutData); 
}

void SamplingDelay()
{
    for(uint8 i=0;i<2;i++)
    {
        asm("nop");
        asm("nop");
    }
}

void ImageCapture(uint8 *ImageData)
{
    PTB20_O=1;   //SI  D3
    SamplingDelay();
    PTC9_O=1;   //CLK  
    SamplingDelay();    
    PTB20_O=0;
    SamplingDelay();
    
    //Delay 10us for sample the first pixel
    /**/
    for(int i = 0; i < 250; i++)
    {                    //更改250，让CCD的图像看上去比较平滑，
        SamplingDelay() ;  //200ns                  //把该值改大或者改小达到自己满意的结果。
    }
    
    //Sampling Pixel1 
    
    *ImageData =  LPLD_ADC_Get(ADC1,AD15);
    ImageData ++;
    PTC9_O=0;           /* CLK = 0 */
    
    for(int i=1;i<128;i++)               
    {   
        SamplingDelay();
        SamplingDelay();
        PTC9_O=1;
        SamplingDelay();
        SamplingDelay();
        *ImageData=LPLD_ADC_Get(ADC1,AD15);
        ImageData ++;
        PTC9_O=0;
    }   
    SamplingDelay();
    SamplingDelay();
    PTC9_O=1;
    SamplingDelay();
    SamplingDelay();
    PTC9_O=0;
}

void CCDImageDisplay()
{
    if(oled.g_s8showpage == -1)
    {
        for(int i=0;i<128;i++)
        {
            switch(imageprocess.g_u8ImageRange)     //输出屏幕显示
            {
              case 0:
                imageprocess.g_u8PixelDisplay[i] =indata.Pixel[i]/4; 
                break;
              case 1:
                imageprocess.g_u8PixelDisplay[i]=indata.Pixel[i]/2;
                break;
              case 2:
                imageprocess.g_u8PixelDisplay[i]=indata.Pixel[i]/8;
                break;
              default:
                imageprocess.g_u8PixelDisplay[i]=indata.Pixel[i]/4;
                break;
            }
        }
    }
    if(oled.g_s8showpage == -2)
    {
        for(int i=0;i<128;i++)
        {
            switch(imageprocess.g_u8ImageRange)     //输出屏幕显示
            {
              case 0:
                imageprocess.g_u8PixelDisplay2[i] =indata.Pixel2[i]/4; 
                break;
              case 1:
                imageprocess.g_u8PixelDisplay2[i]=indata.Pixel2[i]/2;
                break;
              case 2:
                imageprocess.g_u8PixelDisplay2[i]=indata.Pixel2[i]/8;
                break;
              default:
                imageprocess.g_u8PixelDisplay2[i]=indata.Pixel2[i]/4;
                break;
            }
        }
    }
    //    if(oled.g_s8showpage == -3)
    //    {
    //        for(int i=0;i<128;i++)
    //        {
    //            switch(imageprocess.g_u8ImageRange)     //输出屏幕显示
    //            {
    //              case 0:
    //                imageprocess.g_u8PixelDisplay3[i] =indata.Pixel3[i]/4; 
    //                break;
    //              case 1:
    //                imageprocess.g_u8PixelDisplay3[i]=indata.Pixel3[i]/2;
    //                break;
    //              case 2:
    //                imageprocess.g_u8PixelDisplay3[i]=indata.Pixel3[i]/8;
    //                break;
    //              default:
    //                imageprocess.g_u8PixelDisplay3[i]=indata.Pixel3[i]/4;
    //                break;
    //            }
    //        }
    //    }
}

void PixelMaxMin()
{
    imageprocess.g_u8CCDPixelMax = 0;
    imageprocess.g_u8CCDPixelMin = 255;
    imageprocess.g_u8CCDPixel2Max = 0;
    imageprocess.g_u8CCDPixel2Min = 255;
    //    imageprocess.g_u8CCDPixel3Max = 0;
    //    imageprocess.g_u8CCDPixel3Min = 255;
    for(int i=3;i<125;i++)
    {
        if(imageprocess.g_u8CCDPixelMax<indata.Pixel[i])
            imageprocess.g_u8CCDPixelMax = indata.Pixel[i];
        if(imageprocess.g_u8CCDPixelMin>indata.Pixel[i])
            imageprocess.g_u8CCDPixelMin = indata.Pixel[i];
        if(imageprocess.g_u8CCDPixel2Max<indata.Pixel2[i])
            imageprocess.g_u8CCDPixel2Max = indata.Pixel2[i];
        if(imageprocess.g_u8CCDPixel2Min>indata.Pixel2[i])
            imageprocess.g_u8CCDPixel2Min = indata.Pixel2[i];
        //        if(imageprocess.g_u8CCDPixel3Max<indata.Pixel3[i])
        //            imageprocess.g_u8CCDPixel3Max = indata.Pixel3[i];
        //        if(imageprocess.g_u8CCDPixel3Min>indata.Pixel3[i])
        //            imageprocess.g_u8CCDPixel3Min = indata.Pixel3[i];
    }
}

void ImageRecover()
{
    int l_s16PixelDelta;
    int l_s16PixelDelta2;
    //int l_s16Pixel2Array[4];
    //int temp = 0;
    
    for(int i=0;i<128;i++)
    {
        l_s16PixelDelta = (indata.Pixel[i]-DarkCurrent[i])*10000/Cos4Data[i];
        l_s16PixelDelta2 = (indata.Pixel2[i]-DarkCurrent[i])*10000/Cos4Data[i];
        if(l_s16PixelDelta > 255)
            l_s16PixelDelta = 255;
        if(l_s16PixelDelta < 0)
            l_s16PixelDelta = 0; 
        if(l_s16PixelDelta2 > 255)
            l_s16PixelDelta2 = 255;
        if(l_s16PixelDelta2 < 0)
            l_s16PixelDelta2 = 0; 
        
        indata.Pixel[i] = (uint8)l_s16PixelDelta;
        indata.Pixel2[i] = (uint8)l_s16PixelDelta2;
    }
    //    for(int i=1;i<127;i++)
    //    {
    //        l_s16Pixel2Array[0] = indata.Pixel2[i-1];
    //        l_s16Pixel2Array[1] = indata.Pixel2[i];
    //        l_s16Pixel2Array[2] = indata.Pixel2[i+1];
    //        for(int k=0;k<2;k++)
    //        {
    //            if(l_s16Pixel2Array[k] > l_s16Pixel2Array[k+1])
    //            {
    //                temp = l_s16Pixel2Array[k];
    //                l_s16Pixel2Array[k] = l_s16Pixel2Array[k+1];
    //                l_s16Pixel2Array[k+1] = temp;
    //            }
    //        }
    //        if(l_s16Pixel2Array[0]>l_s16Pixel2Array[1])
    //        {
    //            temp = l_s16Pixel2Array[0];
    //            l_s16Pixel2Array[0] = l_s16Pixel2Array[1];
    //            l_s16Pixel2Array[1] = temp;
    //        }
    //        indata.Pixel2[i] = l_s16Pixel2Array[1];
    //    }
}

void GetImage()
{
    ImageCapture(indata.Pixel);
    ImageCapture2(indata.Pixel2);
    //ImageCapture3(indata.Pixel3);
    ImageRecover();    
    CCDImageDisplay();
    PixelMaxMin();   
}

void ImageCapture2(uint8 *ImageData)
{
    PTA16_O=1;   //SI
    SamplingDelay();
    PTA19_O=1;   //CLK  
    SamplingDelay();    
    PTA16_O=0;
    SamplingDelay();
    
    //Delay 10us for sample the first pixel
    /**/
    for(int i = 0; i < 250; i++)
    {                    //更改250，让CCD的图像看上去比较平滑，
        SamplingDelay() ;  //200ns                  //把该值改大或者改小达到自己满意的结果。
    }
    
    //Sampling Pixel2
    
    *ImageData =  LPLD_ADC_Get(ADC1,AD17);
    ImageData ++;
    PTA19_O=0;           /* CLK = 0 */
    
    for(int i=1;i<128;i++)               
    {   
        SamplingDelay();
        SamplingDelay();
        PTA19_O=1;
        SamplingDelay();
        SamplingDelay();
        *ImageData=LPLD_ADC_Get(ADC1,AD17);
        ImageData ++;
        PTA19_O=0;
        //        if(i>2 && i<125)
        //            Pixel[i] /= ccd_table[i-3]; //flat
    }   
    SamplingDelay();
    SamplingDelay();
    PTA19_O=1;
    SamplingDelay();
    SamplingDelay();
    PTA19_O=0;
    
    //CCD采集
    //    PTA16_O=1;   //SI  D3
    //    PTA19_O=1;   //CLK      
    //    PTA16_O=0;
    //    for(int i=0;i<128;i++)               
    //    {   
    //        PTA19_O=0;
    //        *ImageData =  LPLD_ADC_Get(ADC1,AD17);
    //        ImageData ++;
    //        PTA19_O=1;
    //    }   
    //    PTA19_O=0;
}

//void ImageCapture3(uint8 *ImageData)
//{
//    PTB1_O=1;   //SI
//    SamplingDelay();
//    PTB0_O=1;   //CLK  
//    SamplingDelay();    
//    PTB1_O=0;
//    SamplingDelay();
//    
//    //Delay 10us for sample the first pixel
//    /**/
//    for(int i = 0; i < 250; i++)
//    {                    //更改250，让CCD的图像看上去比较平滑，
//        SamplingDelay() ;  //200ns                  //把该值改大或者改小达到自己满意的结果。
//    }
//    
//    //Sampling Pixel2
//    
//    *ImageData =  LPLD_ADC_Get(ADC0,AD13);
//    ImageData ++;
//    PTB0_O=0;           /* CLK = 0 */
//    
//    for(int i=1;i<128;i++)               
//    {   
//        SamplingDelay();
//        SamplingDelay();
//        PTB0_O=1;
//        SamplingDelay();
//        SamplingDelay();
//        *ImageData=LPLD_ADC_Get(ADC0,AD13);
//        ImageData ++;
//        PTB0_O=0;
//        //        if(i>2 && i<125)
//        //            Pixel[i] /= ccd_table[i-3]; //flat
//    }   
//    SamplingDelay();
//    SamplingDelay();
//    PTB0_O=1;
//    SamplingDelay();
//    SamplingDelay();
//    PTB0_O=0;
//}

void CCDIntegration()
{
    PTB20_O=1; 
    PTA16_O=1;
    //PTB1_O=1;
    PTC9_O=1; 
    PTA19_O=1;
    //PTB0_O=1;
    PTB20_O=0;
    PTA16_O=0;
    //PTB1_O=0;
    for(int i=0;i<128;i++)               
    {   
        PTC9_O=0;
        PTA19_O=0;
        //PTB0_O=0;
        PTC9_O=1;
        PTA19_O=1;
        PTB0_O=1;
    }   
    PTC9_O=0;
    PTA19_O=0;
    //PTB0_O=0;
}

void BrakingInTime()
{
    //    if(T/100 >= setpara.SetTime)
    //    {
    //        mycar.g_u8status = 0;
    //    }
    if(mycar.g_u8status == 0)
        BrakingCar();
}

