
#include "common.h"
#include "car_global.h"

void motor_test() //ֻ�ܴ�1-8˳����ԣ���
{
    if(mycar.g_u8status == 0)
    {
        switch(setpara.MotorNum)
        {     
          case 1:
            if(T%10 == 0 && setpara.DutyTest<400)
                setpara.DutyTest += 10;
            //�Һ�
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, setpara.DutyTest);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 2:
            if(T%10 == 0 && setpara.DutyTest > 0)
                setpara.DutyTest -= 10;
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, setpara.DutyTest);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 3:
            if(T%10 == 0 && setpara.DutyTest<400)
                setpara.DutyTest += 10;
            //��ǰ
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, setpara.DutyTest);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 4:
            if(T%10 == 0 && setpara.DutyTest > 0)
                setpara.DutyTest -= 10;
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, setpara.DutyTest);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 5:
            if(T%10 == 0 && setpara.DutyTest<400)
                setpara.DutyTest += 10;
            //��ǰ
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, setpara.DutyTest);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 6:
            if(T%10 == 0 && setpara.DutyTest > 0)
                setpara.DutyTest -= 10;
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, setpara.DutyTest);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 7:
            if(T%10 == 0 && setpara.DutyTest<400)
                setpara.DutyTest += 10;
            //���
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, setpara.DutyTest);
            break;
          case 8:
            if(T%10 == 0 && setpara.DutyTest > 0)
                setpara.DutyTest -= 10;
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, setpara.DutyTest);
            break;
          default:
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
        }
    }
}

void motor_deadthreshold_test()
{
    if(mycar.g_u8status == 0)
    {
        switch(setpara.MotorNum)
        {     
          case 1:
            //�Һ�
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, setpara.DutyTest);  //80
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 2:
            //��ǰ
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, setpara.DutyTest); //66
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 3:
            //��ǰ
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, setpara.DutyTest);//70
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
          case 4:
            //���
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, setpara.DutyTest);  //66
            break;
          default:
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch4, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch5, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch6, 0);
            LPLD_FTM_PWM_ChangeDuty(FTM0, FTM_Ch7, 0);
            break;
        }
    }
}