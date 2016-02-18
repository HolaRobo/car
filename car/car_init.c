#include "car_init.h"

void init_all()
{
    init_gpio();
    pwm_init();
    adc_init();
    init_all_pulse_counter();
    LPLD_Flash_Init();
    init_sdhc();          //SD��ģ���ʼ��   
    uart_interr_init();
    init_i2c();           //MPU6050��ʼ��
    //LPLD_MMA8451_Init();
    OLED_Init();
    pit_init();
    init_paranum();   
    init_setpara();    
    init_readpara();
    save.g_s16SDDenoteNum = 0;//��ֹ��С�ı����0��
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

//ǰհ43���� 2.2m/s ���ǰհֱ�ǲ�����ʱֱ�ӹ�
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
//�ϰ�ǰ����Ѳ��13  ���µ������18
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


//�����ٶ�
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
    gpio_init_struct.GPIO_Pins = GPIO_Pin3|GPIO_Pin4|GPIO_Pin5|GPIO_Pin6|GPIO_Pin7|GPIO_Pin8;     //����3��4��5��6��7��8
    gpio_init_struct.GPIO_Dir = DIR_INPUT;        //����
    gpio_init_struct.GPIO_PinControl = INPUT_PF_EN|IRQC_FA;   //�����ͨ�˲����½����ж�
    gpio_init_struct.GPIO_Isr = portc_isr;        //�жϺ���
    LPLD_GPIO_Init(gpio_init_struct);
    //ʹ���ж�
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
    
    //���� ֱ�Ǽ�� 
    gpio_init_struct.GPIO_PTx = PTB;
    gpio_init_struct.GPIO_Pins = GPIO_Pin2 | GPIO_Pin9 | GPIO_Pin10 | GPIO_Pin16;
    gpio_init_struct.GPIO_Dir = DIR_INPUT;
    gpio_init_struct.GPIO_PinControl = INPUT_PULL_DOWN | IRQC_DIS;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //������
    gpio_init_struct.GPIO_PTx = PTB;
    gpio_init_struct.GPIO_Pins = GPIO_Pin21;
    gpio_init_struct.GPIO_Dir = DIR_OUTPUT;
    gpio_init_struct.GPIO_Output=OUTPUT_L;
    gpio_init_struct.GPIO_PinControl = IRQC_DIS;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //���뿪��
    gpio_init_struct.GPIO_PTx = PTD;
    gpio_init_struct.GPIO_Pins = GPIO_Pin1 | GPIO_Pin2 | GPIO_Pin3;
    gpio_init_struct.GPIO_Dir = DIR_INPUT;
    gpio_init_struct.GPIO_PinControl = IRQC_DIS | INPUT_PULL_DOWN;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //����IO
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
    
    pit0_init_struct.PIT_Pitx = PIT0;            //����PIT0����
    pit0_init_struct.PIT_PeriodMs = PIT0_MS;     //��ʱ����
    pit0_init_struct.PIT_Isr = pit0_isr;         //�����жϺ���
    
    LPLD_PIT_Init(pit0_init_struct);             //��ʼ��PIT0
    
    LPLD_PIT_EnableIrq(pit0_init_struct);        //ʹ��PIT0
    
    PIT->CHANNEL[PIT1].LDVAL = 0xFFFFFFFF;       //���ڲ����ִ��ʱ��
    PIT->CHANNEL[PIT1].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void adc_init()
{
    ADC_InitTypeDef adc_init_struct={0};
    //ccd1
    adc_init_struct.ADC_Adcx = ADC1;
    adc_init_struct.ADC_DiffMode = ADC_SE;        //���˲ɼ�
    adc_init_struct.ADC_BitMode = SE_8BIT;      
    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //�̲���ʱ��
    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4��Ӳ��ƽ��
    //adc_init_struct.ADC_CalEnable = TRUE; //ʹ�ܳ�ʼ��У��  ʹ�ܾͲ�����
    LPLD_ADC_Init(adc_init_struct);
    LPLD_ADC_Chn_Enable(ADC1, AD15); 
    
    //ccd2
    adc_init_struct.ADC_Adcx = ADC1;
    adc_init_struct.ADC_DiffMode = ADC_SE;        //���˲ɼ�
    adc_init_struct.ADC_BitMode = SE_8BIT;      
    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //�̲���ʱ��
    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4��Ӳ��ƽ��
    //adc_init_struct.ADC_CalEnable = TRUE; //ʹ�ܳ�ʼ��У��  ʹ�ܾͲ�����
    LPLD_ADC_Init(adc_init_struct);
    LPLD_ADC_Chn_Enable(ADC1, AD17); 
    
    //ccd3
    //    adc_init_struct.ADC_Adcx = ADC0;
    //    adc_init_struct.ADC_DiffMode = ADC_SE;        //���˲ɼ�
    //    adc_init_struct.ADC_BitMode = SE_8BIT;      
    //    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //�̲���ʱ��
    //    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4��Ӳ��ƽ��
    //    //adc_init_struct.ADC_CalEnable = TRUE; //ʹ�ܳ�ʼ��У��  ʹ�ܾͲ�����
    //    LPLD_ADC_Init(adc_init_struct);
    //    LPLD_ADC_Chn_Enable(ADC0, AD13); 
    
    adc_init_struct.ADC_Adcx = ADC0;
    adc_init_struct.ADC_DiffMode = ADC_SE;        //���˲ɼ�
    adc_init_struct.ADC_BitMode = SE_8BIT;      
    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //�̲���ʱ��
    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4��Ӳ��ƽ��
    //adc_init_struct.ADC_CalEnable = TRUE; //ʹ�ܳ�ʼ��У��  ʹ�ܾͲ�����
    LPLD_ADC_Init(adc_init_struct);
    LPLD_ADC_Chn_Enable(ADC0, AD23);
    
    adc_init_struct.ADC_Adcx = ADC0;
    adc_init_struct.ADC_DiffMode = ADC_SE;        //���˲ɼ�
    adc_init_struct.ADC_BitMode = SE_8BIT;      
    adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //�̲���ʱ��
    adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4��Ӳ��ƽ��
    //adc_init_struct.ADC_CalEnable = TRUE; //ʹ�ܳ�ʼ��У��  ʹ�ܾͲ�����
    LPLD_ADC_Init(adc_init_struct);
    LPLD_ADC_Chn_Enable(ADC0, AD18); 
}

void pwm_init()
{
    FTM_InitTypeDef ftm_init_struct={0};
    ftm_init_struct.FTM_Ftmx = FTM0;	//ʹ��FTM0ͨ��
    ftm_init_struct.FTM_Mode = FTM_MODE_PWM;	//ʹ��PWMģʽ
    ftm_init_struct.FTM_PwmFreq = 15000;  
    
    LPLD_FTM_Init(ftm_init_struct);
    
    //FTM0 ch4��ǰ ch5�Һ� ch6��� ch7��ǰ
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
    
    //�����������빦�ܲ���
    FTM_InitTypeDef ftm_init_struct={0};
    ftm_init_struct.FTM_Ftmx = FTM1;              //ֻ��FTM1��FTM2���������빦��
    ftm_init_struct.FTM_Mode = FTM_MODE_QD;       //�������빦��
    ftm_init_struct.FTM_QdMode = QD_MODE_PHAB;    //AB������ģʽ
    //��ʼ��FTM
    LPLD_FTM_Init(ftm_init_struct);
    //ʹ��AB������ͨ��
    //PTB0���Ž�A�����롢PTB1���Ž�B������
    LPLD_FTM_QD_Enable(FTM1, PTA12, PTA13);
    
    gpio_init_struct.GPIO_PTx = PTB;
    gpio_init_struct.GPIO_Pins = GPIO_Pin18 | GPIO_Pin19;
    gpio_init_struct.GPIO_Dir = DIR_INPUT;
    gpio_init_struct.GPIO_PinControl = INPUT_PULL_UP | INPUT_PF_EN;
    LPLD_GPIO_Init(gpio_init_struct);
    
    //�����������빦�ܲ���
    ftm_init_struct.FTM_Ftmx = FTM2;              //ֻ��FTM1��FTM2���������빦��
    ftm_init_struct.FTM_Mode = FTM_MODE_QD;       //�������빦��
    ftm_init_struct.FTM_QdMode = QD_MODE_PHAB;    //AB������ģʽ
    //��ʼ��FTM
    LPLD_FTM_Init(ftm_init_struct);
    //ʹ��AB������ͨ��
    //PTB0���Ž�A�����롢PTB1���Ž�B������
    LPLD_FTM_QD_Enable(FTM2, PTB18, PTB19);
}

void uart_interr_init()
{
    UART_InitTypeDef term_port_structure={0};
    
    term_port_structure.UART_Uartx = TERM_PORT;
    term_port_structure.UART_BaudRate = TERMINAL_BAUD;
    term_port_structure.UART_RxPin = PTA15;
    term_port_structure.UART_TxPin = PTA14;
    term_port_structure.UART_RxIntEnable = TRUE;    //ʹ�ܽ����ж�
    term_port_structure.UART_RxIsr = uart_isr;      //���ý����жϺ���
    LPLD_UART_Init(term_port_structure);
    
    LPLD_UART_EnableIrq(term_port_structure);    //Enable uart interrupt
}

void init_i2c()
{
    uint8 device_id = MPU6050_Init();
    //ͨ���豸ID�ж��豸�Ƿ�ΪMPU6050
    if(device_id == MPU6050_ID)
    {
        printf("MPU6050��ʼ���ɹ�!\r\n");
        printf("�豸ID: 0x%X\r\n", device_id);
    }
    else
    {
        printf("MPU6050��ʼ��ʧ��!\r\n");
        printf("�豸ID: 0x%X\r\n", device_id);
        //while(1);
    }
}

void init_sdhc()
{
    if(!disk_initialize(0)) printf("SD����ʼ���ɹ�.\n");
    else printf("SD����ʼ��ʧ��.\n");
}

void init_readpara()
{
    //���������ȡ
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
