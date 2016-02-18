#include "car_isr.h"

#if 1 //�жϷ������������������������������������

void uart_isr()
{
    uint8 DeltaV;
    DeltaV = LPLD_UART_GetChar(UART0);
    if(DeltaV == (3+48))
        mycar.g_u8status = 0;
}

/**PortC�жϻص�����&468��ť357��ת������**/  
void portc_isr()
{
    //���PTC3�����жϡ�����ť����
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin3))
    {
        status_button = PRESS;
    }
    //���PTC5�����жϡ�����ť��ת,˳ʱ����
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin5))
    {   
        if(PTC7_I)  //��ťIO
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
            //��ת��������Ӧ
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
    //���PTC6�����жϡ������ְ���
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin6))
    {
        status_button = PUSH;
    }
    //���PTC4�����жϡ�����������    
    if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin4))
    {
        status_button = UP;
    }
    //���PTC8�����жϡ�����������
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
            //����10��С��160��Ϊ��������
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
    
    //ֱ�����ơ��������������������������������� 
    
    IMU_Update(indata.g_fAngleDotBal, indata.g_fAngleAcc);
    
    Upright_Control_calculate();
    
    //�ٶȿ��ơ���������������������������������
    
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
    
    //������ơ���������������������������������    
    
    if(control.g_nDirectionControlCount >= DIRECTION_CONTROL_COUNT) //10ms period
    {
        DirectionControl(); 
        control.g_nDirectionControlCount = 0;
    } 
    
    DirectionControlOutput(); //������ơ����������˳�����Ҫ�����������Ѿ�������
    control.g_nDirectionControlCount ++;
    
    //������ת��������ʼƽ��
    if(mycar.g_u8status >= 1)
    {
        MotorOutput(); 
    }
}


#endif