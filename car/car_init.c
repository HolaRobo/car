#include "car_init.h"

void init_all()
{
    init_gpio();
    pwm_init();
    adc_init();
    init_all_pulse_counter();
    LPLD_Flash_Init();
    init_sdhc();          //SD卡模块初始化   
    uart_interr_init();
    init_i2c();           //MPU6050初始化
    //LPLD_MMA8451_Init();
    OLED_Init();
    pit_init();
    init_paranum();   
    init_setpara();    
    init_readpara();
    save.g_s16SDDenoteNum = 0;//防止不小心保存非0数
}

void init_setpara()
{
    if(PTD1_I == 0)
    {
        control.g_s16DirGyroP = setpara.DirPD.KdGyro;
        track.g_s16DycMidP = 64;
        track.g_s16DycMidP2 = 64;
        status_track = DoubleLineRoad;
        oled.g_u16ParaPrecision = 1;
        setpara.DirBaseIni = 20;
        setpara.DirIniT = 3;
        setpara.CCD1EdgeTH = 24;
        setpara.CCD2EdgeTH = 24;
        setpara.WhitePixelMin = 70;
        setpara.RALeftBias1 = -22;
        setpara.RARightBias1 = 22;
        setpara.RALeftBias2 = -22;
        setpara.RARightBias2 = 22;
        setpara.RALD1 = 0;
        setpara.RALD2 = 0;
        setpara.RARD1 = 0;
        setpara.RARD2 = 0;
        setpara.BarrierLBias = 19;
        setpara.BarrierRBias = -19;
        setpara.BarrierPNum1 = 13;
        setpara.BarrierPNum2 = 13;
        setpara.RampDNum = 120;
        setpara.RunAngle = -90;
        setpara.BalanceAngle = 30;
        setpara.ACCManualCounterNum = 6;
        setpara.TargetSpeed = 27;
        setpara.LeftTurnLimit = -27;
        setpara.RightTurnLimit = 27;
        setpara.BalancePD.Kp = 90;
        setpara.BalancePD.Kd = 2;
        setpara.SpeedPI.KpStart = 26;
        setpara.SpeedPI.KpSteady = 42;
        setpara.SpeedPI.Ki = 10;
        setpara.DirPD.KdGyro = 8;
        setpara.DirBase = 19;
        setpara.DirCoef = 450;
        setpara.DirKdDifK = 12;
        setpara.DutyTest = 200;
        setpara.MotorNum = 0;
        setpara.ToScope = 0;
        setpara.SetTime = 13;
        setpara.SendAllPara = 0;
        setpara.AccidentProtect = 1;
        setpara.Expo = 10;
        setpara.GyroAdj = 0;
        setpara.Race = 1;
        setpara.RampF = 1;
        setpara.ACCT = 1500;
        setpara.RampRAD = 40;
        setpara.RAGyroP1 = 14;
        setpara.RAGyroP2 = 14;
        setpara.RampSLD = 10;
        setpara.BiasLim = 6;
        setpara.RampBrakeN = 4;
        setpara.RampSpeed = 15;
        setpara.RampSLCountN = 10;
        setpara.BarrGyReT = 7;
        setpara.BarrierGyroP = 14;
        setpara.RampGyroP = 18;
        setpara.SaveSelect = 0;
        setpara.BalancePD.KpS = 120;
        setpara.BalancePD.KdS = 4;
        setpara.StartT = 3;
        setpara.TowerStopNum = 3;
        setpara.RampSLNum = 1;
    }
    if(PTD1_I == 1)
    {
        control.g_s16DirGyroP = setpara.DirPD.KdGyro;
        track.g_s16DycMidP = 64;
        track.g_s16DycMidP2 = 64;
        status_track = DoubleLineRoad;
        oled.g_u16ParaPrecision = 1;
        setpara.DirBaseIni = 14;
        setpara.DirIniT = 2;
        setpara.CCD1EdgeTH = 24;
        setpara.CCD2EdgeTH = 24;
        setpara.WhitePixelMin = 70;
        setpara.RALeftBias1 = -21;
        setpara.RARightBias1 = 19;
        setpara.RALeftBias2 = -21;
        setpara.RARightBias2 = 19;
        setpara.RALD1 = 0;
        setpara.RALD2 = 0;
        setpara.RARD1 = 0;
        setpara.RARD2 = 0;
        setpara.BarrierLBias = 16;
        setpara.BarrierRBias = -16;
        setpara.BarrierPNum1 = 30;
        setpara.BarrierPNum2 = 30;
        setpara.RampDNum = 120;
        setpara.RunAngle = -90;
        setpara.BalanceAngle = 30;
        setpara.ACCManualCounterNum = 6;
        setpara.LeftTurnLimit = -27;
        setpara.RightTurnLimit = 27;
        setpara.BalancePD.Kp = 90;
        setpara.BalancePD.Kd = 2;
        setpara.TargetSpeed = 18;
        setpara.SpeedPI.KpStart = 26;
        setpara.SpeedPI.KpSteady = 42;
        setpara.SpeedPI.Ki = 10;
        setpara.DirPD.KdGyro = 12;
        setpara.LeftTurnLimit = -27;
        setpara.RightTurnLimit = 27;
        setpara.DirBase = 13;
        setpara.DirCoef = 550;
        setpara.DirKdDifK = 12;
        setpara.DutyTest = 500;
        setpara.MotorNum = 0;
        setpara.ToScope = 1;
        setpara.SetTime = 15;
        setpara.SendAllPara = 1;
        setpara.AccidentProtect = 1;
        setpara.Expo = 10;
        setpara.GyroAdj = 0;
        setpara.Race = 0;
        setpara.ACCT = 1500;
        setpara.Race = 1;
        setpara.RampF = 0;
        setpara.RampRAD = 40;
        setpara.RAGyroP1 = 13;
        setpara.RAGyroP2 = 14;
        setpara.RampSLD = 10;
        setpara.BiasLim = 6;
        setpara.RampBrakeN = 4;
        setpara.RampSpeed = 15;
        setpara.RampSLCountN = 10;
        setpara.BarrGyReT = 7;
        setpara.BarrierGyroP = 14;
        setpara.RampGyroP = 18;
        setpara.SaveSelect = 1;
        setpara.BalancePD.KpS = 120;
        setpara.BalancePD.KdS = 4;
        setpara.StartT = 3;
        setpara.TowerStopNum = 3;
        setpara.RampSLNum = 1;
    }
}

//前瞻43左右 2.2m/s 这个前瞻直角不用延时直接过
//DirBaseIni = 20;
//DirIniT = 5;
//CCD1EdgeTH = 24;
//CCD2EdgeTH = 24;
//WhitePM = 70;
//RALeftBias = -22;
//RARightBias = 22;
//RAAgT = 8;
//RALF = 0;
//RALM = 0;
//RARF = 0;
//RARM = 0;
//BariLBias = 19;
//BariRBias = -19;
//障碍前都是巡迹13  过坡道后减速18
//BariPNum1 = 13;
//BariPNum2 = 13;
//RampDNum = 120;
//RunAngle = -90;
//BalAngle = 30;
//ACCManualC = 6;
//BalanceKp = 90;
//BalanceKd = 2;
//TargetSpeed = 27;
//SpeedKPST = 26;
//SpeedKPSD = 42;
//SpeedKI = 10;
//DirKdGyro = 8;
//LTurnLimit = -27;
//RTurnLimit = 27;
//DirBase = 19;
//DirCoef = 450;
//DirKdDifK = 12;
//DutyTest = 500;
//MotorNum = 0;
//ToScope = 1;
//SetTime = 3000;
//SendAllPara = 2;
//Protect = 1;
//Expo = 10;
//SDNum = 0;
//GyroAdj = 0;
//Race = 0;
//ACCT = 1500;
//RampF = 2;
//RampW = 53;
//RampRAD = 40;
//RAGyroP = 13;
//RampSLD = 10;
//PoBiasLim = 6;
//RampBN = 4;
//RampSPD = 15;
//RampSLCN = 10;
//BarrGyReT = 7;
//BarrGyP = 14;
//RampGyP = 18;


//保命速度
//DirBaseIni = 14;
//DirIniT = 2;
//CCD1EdgeTH = 24;
//CCD2EdgeTH = 24;
//WhitePM = 70;
//RALeftBias = -21;
//RARightBias = 19;
//RAAgT = 8;
//RALF = 0;
//RALM = 0;
//RARF = 0;
//RARM = 0;
//BariLBias = 16;
//BariRBias = -16;
//BariPNum1 = 30;
//BariPNum2 = 30;
//RampDNum = 120;
//RunAngle = -90;
//BalAngle = 30;
//ACCManualC = 6;
//BalanceKp = 90;
//BalanceKd = 2;
//TargetSpeed = 18;
//SpeedKPST = 26;
//SpeedKPSD = 42;
//SpeedKI = 10;
//DirKdGyro = 12;
//LTurnLimit = -27;
//RTurnLimit = 27;
//DirBase = 13;
//DirCoef = 550;
//DirKdDifK = 12;
//DutyTest = 500;
//MotorNum = 0;
//ToScope = 1;
//SetTime = 3000;
//SendAllPara = 1;
//Protect = 1;
//Expo = 10;
//SDNum = 0;
//GyroAdj = 0;
//Race = 0;
//ACCT = 1500;
//RampF = 3;
//RampW = 53;
//RampRAD = 40;
//RAGyroP = 13;
//RampSLD = 10;
//PoBiasLim = 6;
//RampBN = 4;
//RampSPD = 15;
//RampSLCN = 5;
//BarrGyReT = 7;
//BarrGyP = 14;
//RampGyP = 18;

void init_gpio()
{
    GPIO_InitTypeDef gpio_init_struct={0};
    
    gpio_init_struct.GPIO_PTx = PTC;              //PORTC
    gpio_init_struct.GPIO_Pins = GPIO_Pin3|GPIO_Pin4|GPIO_Pin5|GPIO_Pin6|GPIO_Pin7|GPIO_Pin8;     //引脚3、4、5、6、7、8
    gpio_init_struct.GPIO_Dir = DIR_INPUT;        //输入
    gpio_init_struct.GPIO_PinControl = INPUT_PF_EN|IRQC_FA;   //输入低通滤波，下降沿中断
    gpio_init_struct.GPIO_Isr = portc_isr;        //中断函数
    LPLD_GPIO_Init(gpio_init_struct);
    //使能中断
    LPLD_GPIO_EnableIrq(gpio_init_struct);
    
    //CCD1
    gpio_init_struct.GPIO_PTx = PTB;
    gpio_init_struct.GPIO_Pins = GPIO_Pin20;
    gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
    gpio_init_struct.GPIO_Output=OUTPUT_L;
    gpio_init_struct.GPIO_PinControl = IRQC_DIS;
    LPLD_GPIO_Init(gpio_init_struct);
    gpio_init_struct.GPIO_PTx = PTC;
    gpio_init_struct.GPIO_Pins = GPIO_Pin9;
    gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
    gpio_init_struct.GPIO_Output=OUTPUT_L;
    gpio_init_struct.GPIO_PinControl = IRQC_DIS;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //CCD2
    gpio_init_struct.GPIO_PTx = PTA;
    gpio_init_struct.GPIO_Pins = GPIO_Pin16 | GPIO_Pin19;
    gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
    gpio_init_struct.GPIO_Output=OUTPUT_L;
    gpio_init_struct.GPIO_PinControl = IRQC_DIS;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //CCD3
    //    gpio_init_struct.GPIO_PTx = PTB;
    //    gpio_init_struct.GPIO_Pins = GPIO_Pin0 | GPIO_Pin1;
    //    gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
    //    gpio_init_struct.GPIO_Output=OUTPUT_L;
    //    gpio_init_struct.GPIO_PinControl = IRQC_DIS;
    //    LPLD_GPIO_Init(gpio_init_struct);
    
    //光电管 直角检测 
    gpio_init_struct.GPIO_PTx = PTB;
    gpio_init_struct.GPIO_Pins = GPIO_Pin2 | GPIO_Pin9 | GPIO_Pin10 | GPIO_Pin16;
    gpio_init_struct.GPIO_Dir = DIR_INPUT;
    gpio_init_struct.GPIO_PinControl = INPUT_PULL_DOWN | IRQC_DIS;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //蜂鸣器
    gpio_init_struct.GPIO_PTx = PTB;
    gpio_init_struct.GPIO_Pins = GPIO_Pin21;
    gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
    gpio_init_struct.GPIO_Output=OUTPUT_L;
    gpio_init_struct.GPIO_PinControl = IRQC_DIS;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //拨码开关
    gpio_init_struct.GPIO_PTx = PTD;
    gpio_init_struct.GPIO_Pins = GPIO_Pin1 | GPIO_Pin2 | GPIO_Pin3;
    gpio_init_struct.GPIO_Dir = DIR_INPUT;
    gpio_init_struct.GPIO_PinControl = IRQC_DIS | INPUT_PULL_DOWN;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //备用IO
    //    gpio_init_struct.GPIO_PTx = PTA;
    //    gpio_init_struct.GPIO_Pins = GPIO_Pin4 | GPIO_Pin5;
    //    gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
    //    gpio_init_struct.GPIO_Output=OUTPUT_L;
    //    gpio_init_struct.GPIO_PinControl = IRQC_DIS;
    //    LPLD_GPIO_Init(gpio_init_struct);
}

void pit_init()
{
    PIT_InitTypeDef pit0_init_struct={PIT0};
    
    pit0_init_struct.PIT_Pitx = PIT0;            //配置PIT0参数
    pit0_init_struct.PIT_PeriodMs = PIT0_MS;     //定时周期
    pit0_init_struct.PIT_Isr = pit0_isr;         //设置中断函数
    
    LPLD_PIT_Init(pit0_init_struct);             //初始化PIT0
    
    LPLD_PIT_EnableIrq(pit0_init_struct);        //使能PIT0
    
    PIT->CHANNEL[PIT1].LDVAL = 0xFFFFFFFF;       //用于测程序执行时间
    PIT->CHANNEL[PIT1].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void adc_init()
{
    ADC_InitTypeDef adc_init_struct={0};
    //ccd1
    adc_init_struct.ADC_Adcx = ADC1;
    adc_init_struct.ADC_DiffMode = ADC_SE;        //单端采集
    adc_init_struct.ADC_BitMode = SE_8BIT;      
    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //短采样时间
    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4次硬件平均
    //adc_init_struct.ADC_CalEnable = TRUE; //使能初始化校验  使能就不行了
    LPLD_ADC_Init(adc_init_struct);
    LPLD_ADC_Chn_Enable(ADC1, AD15); 
    
    //ccd2
    adc_init_struct.ADC_Adcx = ADC1;
    adc_init_struct.ADC_DiffMode = ADC_SE;        //单端采集
    adc_init_struct.ADC_BitMode = SE_8BIT;      
    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //短采样时间
    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4次硬件平均
    //adc_init_struct.ADC_CalEnable = TRUE; //使能初始化校验  使能就不行了
    LPLD_ADC_Init(adc_init_struct);
    LPLD_ADC_Chn_Enable(ADC1, AD17); 
    
    //ccd3
    //    adc_init_struct.ADC_Adcx = ADC0;
    //    adc_init_struct.ADC_DiffMode = ADC_SE;        //单端采集
    //    adc_init_struct.ADC_BitMode = SE_8BIT;      
    //    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //短采样时间
    //    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4次硬件平均
    //    //adc_init_struct.ADC_CalEnable = TRUE; //使能初始化校验  使能就不行了
    //    LPLD_ADC_Init(adc_init_struct);
    //    LPLD_ADC_Chn_Enable(ADC0, AD13); 
    
    adc_init_struct.ADC_Adcx = ADC0;
    adc_init_struct.ADC_DiffMode = ADC_SE;        //单端采集
    adc_init_struct.ADC_BitMode = SE_8BIT;      
    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //短采样时间
    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4次硬件平均
    //adc_init_struct.ADC_CalEnable = TRUE; //使能初始化校验  使能就不行了
    LPLD_ADC_Init(adc_init_struct);
    LPLD_ADC_Chn_Enable(ADC0, AD23);
    
    adc_init_struct.ADC_Adcx = ADC0;
    adc_init_struct.ADC_DiffMode = ADC_SE;        //单端采集
    adc_init_struct.ADC_BitMode = SE_8BIT;      
    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //短采样时间
    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4次硬件平均
    //adc_init_struct.ADC_CalEnable = TRUE; //使能初始化校验  使能就不行了
    LPLD_ADC_Init(adc_init_struct);
    LPLD_ADC_Chn_Enable(ADC0, AD18); 
}

void pwm_init()
{
    FTM_InitTypeDef ftm_init_struct={0};
    ftm_init_struct.FTM_Ftmx = FTM0;	//使能FTM0通道
    ftm_init_struct.FTM_Mode = FTM_MODE_PWM;	//使能PWM模式
    ftm_init_struct.FTM_PwmFreq = 15000;  
    
    LPLD_FTM_Init(ftm_init_struct);
    
    //FTM0 ch4右前 ch5右后 ch6左后 ch7左前
    LPLD_FTM_PWM_Enable(FTM0,FTM_Ch4,0, PTD4,ALIGN_LEFT);
    LPLD_FTM_PWM_Enable(FTM0,FTM_Ch5,0, PTD5,ALIGN_LEFT);
    LPLD_FTM_PWM_Enable(FTM0,FTM_Ch6,0, PTD6,ALIGN_LEFT);
    LPLD_FTM_PWM_Enable(FTM0,FTM_Ch7,0, PTD7,ALIGN_LEFT);
}

void init_all_pulse_counter()
{
    GPIO_InitTypeDef gpio_init_struct={0};
    gpio_init_struct.GPIO_PTx = PTA;
    gpio_init_struct.GPIO_Pins = GPIO_Pin12 | GPIO_Pin13;
    gpio_init_struct.GPIO_Dir = DIR_INPUT;
    gpio_init_struct.GPIO_PinControl = INPUT_PULL_UP | INPUT_PF_EN;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //配置正交解码功能参数
    FTM_InitTypeDef ftm_init_struct={0};
    ftm_init_struct.FTM_Ftmx = FTM1;              //只有FTM1和FTM2有正交解码功能
    ftm_init_struct.FTM_Mode = FTM_MODE_QD;       //正交解码功能
    ftm_init_struct.FTM_QdMode = QD_MODE_PHAB;    //AB相输入模式
    //初始化FTM
    LPLD_FTM_Init(ftm_init_struct);
    //使能AB相输入通道
    //PTB0引脚接A相输入、PTB1引脚接B相输入
    LPLD_FTM_QD_Enable(FTM1, PTA12, PTA13);
    
    gpio_init_struct.GPIO_PTx = PTB;
    gpio_init_struct.GPIO_Pins = GPIO_Pin18 | GPIO_Pin19;
    gpio_init_struct.GPIO_Dir = DIR_INPUT;
    gpio_init_struct.GPIO_PinControl = INPUT_PULL_UP | INPUT_PF_EN;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //配置正交解码功能参数
    ftm_init_struct.FTM_Ftmx = FTM2;              //只有FTM1和FTM2有正交解码功能
    ftm_init_struct.FTM_Mode = FTM_MODE_QD;       //正交解码功能
    ftm_init_struct.FTM_QdMode = QD_MODE_PHAB;    //AB相输入模式
    //初始化FTM
    LPLD_FTM_Init(ftm_init_struct);
    //使能AB相输入通道
    //PTB0引脚接A相输入、PTB1引脚接B相输入
    LPLD_FTM_QD_Enable(FTM2, PTB18, PTB19);
}

void uart_interr_init()
{
    UART_InitTypeDef term_port_structure={0};
    
    term_port_structure.UART_Uartx = TERM_PORT;
    term_port_structure.UART_BaudRate = TERMINAL_BAUD;
    term_port_structure.UART_RxPin = PTA15;
    term_port_structure.UART_TxPin = PTA14;
    term_port_structure.UART_RxIntEnable = TRUE;    //使能接收中断
    term_port_structure.UART_RxIsr = uart_isr;      //设置接收中断函数
    LPLD_UART_Init(term_port_structure);
    
    LPLD_UART_EnableIrq(term_port_structure);    //Enable uart interrupt
}

void init_i2c()
{
    uint8 device_id = MPU6050_Init();
    //通过设备ID判断设备是否为MPU6050
    if(device_id == MPU6050_ID)
    {
        printf("MPU6050初始化成功!\r\n");
        printf("设备ID: 0x%X\r\n", device_id);
    }
    else
    {
        printf("MPU6050初始化失败!\r\n");
        printf("设备ID: 0x%X\r\n", device_id);
        //while(1);
    }
}

void init_sdhc()
{
    if(!disk_initialize(0)) printf("SD卡初始化成功.\n");
    else printf("SD卡初始化失败.\n");
}

void init_readpara()
{
    //定义参数读取
    if(PTD1_I == 0)
    {
        for(int i=0;i<oled.g_u16ParaNum;i++)
        {
            *paralist[i].para = *(int*)(FLASH_SAVE_SECTOR*0x800+i*4);
        }
    }
    
    if(PTD1_I == 1)
    {
        for(int i=0;i<oled.g_u16ParaNum;i++)
        {
            *paralist[i].para = *(int*)(FLASH_SAVE_SECTOR2*0x800+i*4);
        }
    }
}

void init_paranum()
{
    while(paralist[oled.g_u16ParaNum].precision)
        oled.g_u16ParaNum++;
    oled.g_s8ShowpageMax=2;
    oled.g_s8ShowpageMin=-2;
}
