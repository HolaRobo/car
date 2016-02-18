#include "car_isr.h"

#if 1 //中断服务函数★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★

void uart_isr()
{
    uint8 DeltaV;
    DeltaV = LPLD_UART_GetChar(UART0);
    if(DeltaV == (3+48))
        mycar.g_u8status = 0;
}

/**PortC中断回调函数&468按钮357旋转编码器**/  
void portc_isr()
{
    //如果PTC3产生中断——旋钮按下
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin3))
    {
        status_button = PRESS;
    }
    //如果PTC5产生中断——旋钮旋转,顺时针先
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin5))
    {   
        if(PTC7_I)  //旋钮IO
        {
            status_button = CW;
            if(oled.g_s8showpage >= 0)
            {
                if(oled.g_u8changepara)
                    change_para(1);
                else 
                {
                    if(oled.g_u16ParaSelect <oled.g_u16ParaNum-1)
                        oled.g_u16ParaSelect ++;
                    else
                        oled.g_u16ParaSelect = 0;
                }
            }
            else
                change_para(1);
        }
        else
        {
            status_button = CCW;
            //旋转编码器响应
            if(oled.g_s8showpage >= 0)
            {
                if(oled.g_u8changepara)
                    change_para(2);
                else
                {
                    if(oled.g_u16ParaSelect >0)
                        oled.g_u16ParaSelect --;
                    else
                        oled.g_u16ParaSelect = oled.g_u16ParaNum-1;
                }
            }
            else
                change_para(2);
        }
    }
    //如果PTC6产生中断——拨轮按下
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin6))
    {
        status_button = PUSH;
    }
    //如果PTC4产生中断——拨轮向上    
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin4))
    {
        status_button = UP;
    }
    //如果PTC8产生中断——拨轮向下
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin8))
    {
        status_button = DOWN;
    }
}

void pit0_isr()
{   
    T+=PIT0_MS;
    
    //if(mycar.g_u8status == 1 && tower.g_u8Tower_vol <180)
    if(setpara.Race == 0)
    {
        if(mycar.g_u8status == 1 && (T-mycar.g_u16ActiveT) >= setpara.StartT*1000)
        {
            mycar.g_u16ActiveT = T;
            mycar.g_u8status = 2;
            control.g_s16TargetSpeed = setpara.TargetSpeed;
            control.g_s16BalanceAngle = setpara.RunAngle;
            control.g_s16BalanceKp = setpara.BalancePD.Kp;
            control.g_s16BalanceKd = setpara.BalancePD.Kd;
        }
        if(mycar.g_u8status == 2 && (T-mycar.g_u16ActiveT) >=setpara.ACCT)
            mycar.g_u8status = 3;
    }
    
    tower.g_u8Tower_vol =  LPLD_ADC_Get(ADC0,AD18);
    
    if(setpara.Race == 1)
    {
        if(mycar.g_u8status == 1)
        {
            //连续10次小于160认为灯塔灯灭
            if(tower.g_u8Tower_vol < 160)
                tower.g_s8TowerCounter ++;
            else
            {
                tower.g_s8TowerCounter --;
                if(tower.g_s8TowerCounter < 0)
                    tower.g_s8TowerCounter = 0;
            }
            if(tower.g_s8TowerCounter >= 5 && mycar.g_u8status == 1)
            {
                tower.g_s8TowerCounter = 0;
                mycar.g_u16ActiveT = T;
                mycar.g_u8status = 2;
                control.g_s16TargetSpeed = setpara.TargetSpeed;
                control.g_s16BalanceAngle = setpara.RunAngle;
                control.g_s16BalanceKp = setpara.BalancePD.Kp;
                control.g_s16BalanceKd = setpara.BalancePD.Kd;
            }
        }
        if(mycar.g_u8status == 2 && (T-mycar.g_u16ActiveT) >=setpara.ACCT)
            mycar.g_u8status = 3;
        
        if(tower.g_u8Finish == 0 && (T-mycar.g_u16ActiveT) >= setpara.SetTime*1000)
        {
            if(mycar.g_u8status == 3 && tower.g_u8Tower_vol > 160)
                tower.g_s8TowerCounterL ++;
            else if(mycar.g_u8status == 3 && tower.g_u8Tower_vol < 160)
            {
                tower.g_s8TowerCounterL --;
                if(tower.g_s8TowerCounterL < 0)
                    tower.g_s8TowerCounterL = 0;
            }
            if(tower.g_s8TowerCounterL >= setpara.TowerStopNum)
            {
                tower.g_u8Finish = 1;
                tower.g_s8TowerCounterL = 0;
            }
        }
        
        if(tower.g_u8Finish == 1)
        {
            if(tower.g_u8Tower_vol < 130)
            {
                tower.g_s8TowerCounterF ++;
            }
            else if(tower.g_u8Tower_vol > 140)
            {
                tower.g_s8TowerCounterF --;
                if(tower.g_s8TowerCounterF < 0)
                    tower.g_s8TowerCounterF = 0;
            }
            if(tower.g_s8TowerCounterF >= 2)
            {
                tower.g_s8TowerCounterF = 0;
                mycar.g_u8status = 0;
            }
        }
    }
    
    data_input();
    
    //直立控制★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★ 
    
    IMU_Update(indata.g_fAngleDotBal, indata.g_fAngleAcc);
    
    Upright_Control_calculate();
    
    //速度控制★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
    
    if(mycar.g_u8status == 1)
    {
        control.g_s16BalanceAngle = setpara.BalanceAngle;
        control.g_s16TargetSpeed = 0;
        control.g_s16BalanceKp = setpara.BalancePD.KpS;
        control.g_s16BalanceKd = setpara.BalancePD.KdS;
        if(control.g_nSpeedControlCount >= SPEED_CONTROL_COUNT)
        {
            SpeedControl(); 
            control.g_nSpeedControlCount = 0;
        }
        SpeedControlOutput();
        control.g_nSpeedControlCount ++;
    }
    else if(mycar.g_u8status >= 2)
    {
        if(flag.g_u8SpeedClear == 0)
        {
            control.g_fSpeedControlIntegral = 0;
            control.g_fSpeedControlOutNew = 0;
            control.g_fSpeedControlOutOld = 0;
            flag.g_u8SpeedClear = 1;
        }
        
        if(control.g_nSpeedControlCount >= SPEED_CONTROL_COUNT)
        {
            SpeedControl(); 
            control.g_nSpeedControlCount = 0;
            
            if((T-mycar.g_u16ActiveT) >= setpara.ACCT && control.g_u8CarACC_F == 0)
            {
                control.g_u8AngleRecover = 1;
            }
            if(control.g_u8AngleRecover == 1 && control.g_u8CarACC_F == 0)
            {
                control.g_s16BalanceAngle += 10;
                control.g_u8CarACCNum ++;
                if(control.g_u8CarACCNum == setpara.ACCManualCounterNum)
                {
                    control.g_u8CarACC_F = 1;
                    control.g_u8AngleRecover = 0;
                }
            }
            if(ramp.g_u8BrakeS)
            {
                control.g_s16BalanceAngle -= 20;
                ramp.g_u8AngleNum ++;
                if(ramp.g_u8AngleNum >= setpara.RampBrakeN)
                {
                    ramp.g_u8AngleNum = 0;
                    ramp.g_u8BrakeS = 0;
                }
            }
        }
        
        SpeedControlOutput();
        
        control.g_nSpeedControlCount ++;
    }
    
    //方向控制★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★    
    
    if(control.g_nDirectionControlCount >= DIRECTION_CONTROL_COUNT) //10ms period
    {
        DirectionControl(); 
        control.g_nDirectionControlCount = 0;
    } 
    
    DirectionControlOutput(); //方向控制、计数和输出顺序很重要！！！现在已经合理安排
    control.g_nDirectionControlCount ++;
    
    //长按旋转编码器开始平衡
    if(mycar.g_u8status >= 1)
    {
        MotorOutput(); 
    }
}


#endif