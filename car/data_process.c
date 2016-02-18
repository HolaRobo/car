#include "data_process.h"


void EdgeDetect(int16 midline)
{    
    for(int i=midline;i>5;i--) 
    {
        if(indata.Pixel[i] - indata.Pixel[i-3] > setpara.CCD1EdgeTH)
        {
            track.g_s16LEdge = i;
            track.g_u8LMissP = 0;
            break;
        }
        else
        {  
            track.g_u8LMissP = 1;
        }            
    }
    
    for(int i=midline;i<122;i++)
    {
        if(indata.Pixel[i] - indata.Pixel[i+3] > setpara.CCD1EdgeTH)
        {
            track.g_s16REdge = i;
            track.g_u8RMissP = 0;
            break;
        }
        else 
        {   
            track.g_u8RMissP = 1;
        }
    } 
    //���߲�����ʱ���������� ccd���ܿ�����
    track.g_s16DycMidP = (track.g_s16LEdge +track.g_s16REdge)/2;
    track.RoadWidth = track.g_s16REdge - track.g_s16LEdge;
}

void CCD2Detect()
{
    //    if(T<500)
    //        track.g_s16DycMidP2 = 64; 
    for(int i=64;i>5;i--) 
    {
        if(indata.Pixel2[i] - indata.Pixel2[i-3] > setpara.CCD2EdgeTH)
        {
            track.g_s16LEdge2 = i;
            break;
        }
    }
    
    for(int i=64;i<122;i++)
    {
        if(indata.Pixel2[i] - indata.Pixel2[i+3] > setpara.CCD2EdgeTH)
        {
            track.g_s16REdge2 = i;
            break;
        }
    } 
    track.g_s16DycMidP2 = (track.g_s16LEdge2+track.g_s16REdge2)/2;
    
    track.RoadWidth2 = track.g_s16REdge2 - track.g_s16LEdge2;
    
    track.g_s16DirBiasNew2 = track.g_s16DycMidP2 - 64;
    
    if(mycar.g_u8status == 0)
    {
        imageprocess.g_u8PixelDisplay2[64] = 0x00;
        imageprocess.g_u8PixelDisplay2[track.g_s16DycMidP2] = 0x00;
    }
}

//void CCD3EdgeDetect(int16 midline)
//{    
//    for(int i=midline;i>5;i--) 
//    {
//        if(indata.Pixel3[i] - indata.Pixel3[i-3] > setpara.CCD3EdgeTH)
//        {
//            track.g_s16LEdge = i;
//            track.g_u8LMissP = 0;
//            break;
//        }
//        else
//        {  
//            track.g_u8LMissP = 1;
//        }            
//    }
//    
//    for(int i=midline;i<122;i++)
//    {
//        if(indata.Pixel3[i] - indata.Pixel3[i+3] > setpara.CCD3EdgeTH)
//        {
//            track.g_s16REdge = i;
//            track.g_u8RMissP = 0;
//            break;
//        }
//        else 
//        {   
//            track.g_u8RMissP = 1;
//        }
//    } 
//    //���߲�����ʱ���������� ccd���ܿ�����
//    track.g_s16DycMidP = (track.g_s16LEdge +track.g_s16REdge)/2;
//    track.RoadWidth = track.g_s16REdge - track.g_s16LEdge;
//    
//    if(mycar.g_u8status == 0)
//    {
//        imageprocess.g_u8PixelDisplay3[64] = 0x00;
//        imageprocess.g_u8PixelDisplay3[track.g_s16DycMidP] = 0x00;
//    }
//}

void CommonRoadDriving()
{    
    //CCD1����������̬����
    //ʮ�ֱ�����һ�εı���ֵ
    //�Ҳ�������g_s16LEdge g_s16REdge ��������ϴε�ֵ
    //�Һ���ı���
    
    //�������ϰ��������������� ���ֱܷ��������Ѱ�ҳ����У�������ظ��ۼ�
    if(track.g_u8LineContinueDetectPause == 1)
    {
        track.g_u8LineContinueCounter ++;
        if(track.g_u8LineContinueCounter == 10)
        {
            track.g_u8LineContinueDetectPause = 0;
            track.g_u8LineContinueCounter = 0;
        }
    }
    
    for(int i=track.g_s16DycMidP;i>5;i--) 
    {
        if(indata.Pixel[i] - indata.Pixel[i-3] > setpara.CCD1EdgeTH)
        {
            if((T-mycar.g_u16ActiveT) <= 1000)
            {
                track.g_s16LEdge = i; //�Ҳ����Ͳ�����
                track.g_u8LMissP = 0;
                break;
            }
            else
            {
                if(abs(i-track.Edge[0])<10 && track.g_u8LineContinueDetectPause == 0 && track.g_u8LineContineStop == 0)
                {
                    track.g_s16LEdge = i; //�Ҳ����Ͳ�����
                    track.g_u8LMissP = 0;
                    break;
                }
                if(track.g_u8LineContinueDetectPause)
                {
                    track.g_s16LEdge = i; //�Ҳ����Ͳ�����
                    track.g_u8LMissP = 0;
                    break;
                }
                if(track.g_u8LineContineStop)
                {
                    track.g_s16LEdge = i; //�Ҳ����Ͳ�����
                    track.g_u8LMissP = 0;
                    break;
                }
            }
        }
        else
        {   
            track.g_u8LMissP = 1;
        }            
    }
    
    for(int i=track.g_s16DycMidP;i<122;i++)
    {
        if(indata.Pixel[i] - indata.Pixel[i+3] > setpara.CCD1EdgeTH)
        {
            if(T <= 3000)
            {
                track.g_s16REdge = i; //�Ҳ����Ͳ�����
                track.g_u8RMissP = 0;
                break;
            }
            else
            {
                if(abs(i-track.Edge[1])<10 && track.g_u8LineContinueDetectPause == 0 && track.g_u8LineContineStop == 0)
                {
                    track.g_s16REdge = i; //�Ҳ����Ͳ�����
                    track.g_u8RMissP = 0;
                    break;
                }
                if(track.g_u8LineContinueDetectPause)
                {
                    track.g_s16REdge = i; //�Ҳ����Ͳ�����
                    track.g_u8RMissP = 0;
                    break;
                }
                if(track.g_u8LineContineStop)
                {
                    track.g_s16REdge = i; //�Ҳ����Ͳ�����
                    track.g_u8RMissP = 0;
                    break;
                }
            }
        }
        else 
        {   
            track.g_u8RMissP = 1;
        }
    }
    
    //�Ȱ����ұ���ֵ�����ٸ������ڵ����ұ���ֵ���ߣ�����
    
    track.g_s16DycMidP = (track.g_s16LEdge + track.g_s16REdge)/2;
    track.g_s16DirBiasNew = track.g_s16DycMidP - 64;
    
    track.RoadWidth = track.g_s16REdge - track.g_s16LEdge;
    
    track.Edge[0] = track.g_s16LEdge; 
    track.Edge[1] = track.g_s16REdge; 
    
    if(mycar.g_u8status == 0)
    {
        imageprocess.g_u8PixelDisplay[64] = 0x00;
        imageprocess.g_u8PixelDisplay[track.g_s16DycMidP] = 0x00;
    }
}

void CrossRoadDetect()
{
    if(track.g_u8LMissP ==1 && track.g_u8RMissP ==1 &&imageprocess.g_u8CCDPixelMin > setpara.WhitePixelMin)
        status_track = CrossRoad;
}

void RightAngleDetect()
{
    //ֱ��
    //�жϺ���ͣɨ�� g_u8RightAngleScanPause = 0ʱ�ж�
    if(PTB2_I == 1 && PTB10_I == 1 && rightangle.g_u8RightAngleScanPause == 0)
    {
        rightangle.g_u8RABarNum ++;
        if(rightangle.g_u8RABarNum%2 == 1) //������Ϊ��ֱ�Ǳ�־
        {
            status_track = RightAngleRoad;
            track.g_u8LineContineStop = 1;
            
            if(rightangle.g_u8RABarNum == 1)
            {
                control.g_s16DirGyroP = setpara.RAGyroP1;
                rightangle.g_u8RADelayL = setpara.RARD1;
                rightangle.g_u8RADelayR = setpara.RARD1;
                rightangle.g_s16RALeftBias = setpara.RALeftBias1;
                rightangle.g_s16RARightBias = setpara.RARightBias1;
            }
            if(rightangle.g_u8RABarNum == 3)
            {
                control.g_s16DirGyroP = setpara.RAGyroP2;
                rightangle.g_u8RADelayL = setpara.RARD2;
                rightangle.g_u8RADelayR = setpara.RARD2;
                rightangle.g_s16RALeftBias = setpara.RALeftBias2;
                rightangle.g_s16RARightBias = setpara.RARightBias2;
            }
        }
        else //ż����ͣɨ��15��
        {
            //�����ж�ֻ���һ�θ�ֵ
            control.g_s16DirGyroP = setpara.DirPD.KdGyro;
            rightangle.g_u8RightAngleScanPause = 1;
        }
    }
    
    //ż�������ѭ������ͣɨ�裬��������ֱ�ǳ�������ͣɨ��
    if(rightangle.g_u8RightAngleScanPause == 1)
    {
        rightangle.g_u8RightAngleScanPauseCounter ++;
        if(rightangle.g_u8RightAngleScanPauseCounter == 15)
        {
            rightangle.g_u8RightAngleScanPauseCounter = 0;
            rightangle.g_u8RightAngleScanPause = 0;
        }
    }
}

int RightAngleTurn(uint8 num)
{
    int l_s16RATurnBias;
    EdgeDetect(64);
    
    if(track.g_u8LMissP == 0 && track.g_u8RMissP == 0)
    {
        status_track = DoubleLineRoad;
        track.g_s16DycMidP = 64;
        track.g_u8LineContineStop = 0;
        //ת��ֱ����ͣ10������������  ��ʱ���Գ�ֱ�Ǻ���
        track.g_u8LineContinueDetectPause = 1;
    }
    if(num == 1) l_s16RATurnBias = rightangle.g_s16RALeftBias;
    if(num == 2) l_s16RATurnBias = rightangle.g_s16RARightBias;
    return l_s16RATurnBias;
}


void SDSavePara(int para,uint16 num)
{
    save.g_u8SDBuffer[save.g_u32SDBufNum%50][SDBase+2*num] = (uint8)(((uint16)para)>>8);
    save.g_u8SDBuffer[save.g_u32SDBufNum%50][SDBase+2*num+1] = (uint8)((uint16)para);
}

void PassCrossRoad()
{
    for(int i=64;i>5;i--) 
    {
        if(indata.Pixel[i] - indata.Pixel[i-3] > setpara.CCD1EdgeTH)
        {
            track.g_s16LEdge = i;
            track.g_u8LMissP = 0;
            break;
        }
        else
        {  
            track.g_u8LMissP = 1;
        }            
    }
    
    for(int i=64;i<122;i++)
    {
        if(indata.Pixel[i] - indata.Pixel[i+3] > setpara.CCD1EdgeTH)
        {
            track.g_s16REdge = i;
            track.g_u8RMissP = 0;
            break;
        }
        else 
        {   
            track.g_u8RMissP = 1;
        }
    }
    
    //���߶��߳�����ֱ����
    if(track.g_u8LMissP == 1 && track.g_u8RMissP ==1)
    {
        track.g_s16DirBiasNew = 0;
    }
    else
    {
        status_track =DoubleLineRoad;
        track.g_s16DycMidP = 64;
        track.g_u8LineContinueDetectPause = 1;
    }
}

void PassRARoad()
{
    //CCD2Detect();
    if(rightangle.g_u8LLoseF == 0 && rightangle.g_u8RLoseF == 0)
        CommonRoadDriving();
    
    //��ֱ�����һ�߶��߲��գ���ֱ��
    //һ�߶���ֻ�ж�һ��
    if(track.g_u8LMissP == 1 && track.g_u8RMissP == 0 && rightangle.g_u8RLoseF == 0)
        rightangle.g_u8LLoseF = 1;
    if(track.g_u8LMissP == 0 && track.g_u8RMissP == 1 && rightangle.g_u8LLoseF == 0)
        rightangle.g_u8RLoseF = 1;
    
    if(rightangle.g_u8LLoseF)
    {
        rightangle.g_u8RATurnC ++;
        //��ֱ��
        track.g_s16DirBiasNew = 0;
        
        if(rightangle.g_u8RATurnC >= rightangle.g_u8RADelayL)
        {
            status_track = RightAngleTurnLeft;
            rightangle.g_u8LLoseF = 0;
            rightangle.g_u8RATurnC = 0;
        }
    }
    if(rightangle.g_u8RLoseF)
    {
        rightangle.g_u8RATurnC ++;
        //��ֱ��
        track.g_s16DirBiasNew = 0;
        
        if(rightangle.g_u8RATurnC >= rightangle.g_u8RADelayR)
        {
            status_track = RightAngleTurnRight;
            rightangle.g_u8RLoseF = 0;
            rightangle.g_u8RATurnC = 0;
        }
    }
}

void BarrierDetect()
{
    uint8 i,j;  
    
    if(barrier.g_s16BarrierGyroRecover == 1)
        barrier.g_u8BarrGyRecoverC ++;
    if(barrier.g_u8BarrGyRecoverC >= setpara.BarrGyReT)
    {
        barrier.g_s16BarrierGyroRecover = 0;
        barrier.g_u8BarrGyRecoverC = 0;
        control.g_s16DirGyroP = setpara.DirPD.KdGyro;
    }
    
    //track.g_s16REdge-7��ֹ�����ұ��أ�����Ϊ�ϰ����� 
    for(i=track.g_s16LEdge;i<track.g_s16REdge-7;i++)
    {
        if((indata.Pixel[i]-indata.Pixel[i+3]) > setpara.CCD1EdgeTH)
        {
            //track.g_s16REdge-2������ָ�����
            for(j=i+7;j<track.g_s16REdge-2;j++)
            {
                if((indata.Pixel[j]-indata.Pixel[j+3]) < -setpara.CCD1EdgeTH)
                {
                    barrier.g_u8BarrierWidth = j - i;
                    break;
                }
            }
            if(barrier.g_u8BarrierWidth > 10 && barrier.g_u8BarrierWidth <= 25)
            {
                if((i+j)/2 < 64)
                {
                    status_track = LeftBarrier;
                    //����ת��΢��
                    control.g_s16DirGyroP = setpara.BarrierGyroP;
                    track.g_s16DirBiasNew = setpara.BarrierLBias;
                    barrier.g_u8BarrierEdge = j+5;
                    
                    //                    if((T-mycar.g_u16ActiveT) < setpara.g_s16RAAngleT*1000)    
                    //                        barrier.g_u8BarrierNum = (uint8)setpara.BarrierPNum1;
                    //                    else
                    barrier.g_u8BarrierNum = (uint8)setpara.BarrierPNum1;
                    
                    //����������
                    barrier.g_u8BarrierCounter = 0;
                    track.g_s16DycMidP = (j+5+track.g_s16REdge)/2;
                    break;
                }
                //���������ߵ�ƽ��ֵ������Ϊ�ж�
                if((i+j)/2 > 64)
                {
                    status_track = RightBarrier;
                    //����ת��΢��
                    control.g_s16DirGyroP = setpara.BarrierGyroP;
                    track.g_s16DirBiasNew = setpara.BarrierRBias;                   
                    barrier.g_u8BarrierEdge = i;
                    
                    //                    if((T-mycar.g_u16ActiveT) < setpara.g_s16RAAngleT*1000)    
                    //                        barrier.g_u8BarrierNum = (uint8)setpara.BarrierPNum1;
                    //                    else
                    barrier.g_u8BarrierNum = (uint8)setpara.BarrierPNum2;
                    //����������
                    barrier.g_u8BarrierCounter = 0;
                    track.g_s16DycMidP = (track.g_s16LEdge+i)/2;
                    break;
                }
            }
        }
    }
}

void SingleLineDetect()
{
    //���˳����߳����ʱ�������е���ȫ�ֱ�����Ҳ����˵��¼SD�����������˳����ߺ������
    for(int i=track.g_s16LEdge;i<track.g_s16REdge-5;i++)
    {
        if((indata.Pixel[i]-indata.Pixel[i+3]) > setpara.CCD1EdgeTH)
        {
            for(int j=i+3;j<track.g_s16REdge;j++)
            {
                if((indata.Pixel[j]-indata.Pixel[j+3]) < -setpara.CCD1EdgeTH)
                {
                    single.g_u8SingleLW = j-i; 
                    break;
                }
            }
            if(single.g_u8SingleLW >= 2 && single.g_u8SingleLW <=10)
            {
                //��������ɫ������
                single.g_u8SLLeftWit = i-track.g_s16LEdge;
                //���Ҽ����ɫ������
                single.g_u8SLRightWit = track.g_s16REdge-(i+single.g_u8SingleLW+2);
                //�����������ұ� i+4Ϊ�������м�
                if((i+4)>=64)
                {
                    if((i+4-64)<=30 && single.g_u8SLLeftWit>15 && single.g_u8SLRightWit > 15)
                    {
                        single.g_u8SLPosition = i+2;
                        status_track = SingleLineRoad;                           
                        break; //������forѭ��
                    }
                    if((i+4-64)>30 && (i+4-64)<=40 && single.g_u8SLLeftWit>10 && single.g_u8SLRightWit > 5)
                    {
                        single.g_u8SLPosition = i+2;
                        status_track = SingleLineRoad;
                        break;
                    }
                    if((i+4-64)>40 && single.g_u8SLLeftWit>10 && single.g_u8SLRightWit > 2)
                    {
                        single.g_u8SLPosition = i+2;
                        status_track = SingleLineRoad;
                        break;
                    }
                }
                //������������� i+4Ϊ�������м�
                if((i+4)<64)
                {
                    if((64-i-4)<=30 && single.g_u8SLLeftWit>15 && single.g_u8SLRightWit > 15)
                    {                        
                        single.g_u8SLPosition = i+2;
                        status_track = SingleLineRoad;                           
                        break; //������forѭ��
                    }
                    if((64-i-4)>30 && (64-i-4)<=40 && single.g_u8SLLeftWit>5 && single.g_u8SLRightWit > 10)
                    {
                        single.g_u8SLPosition = i+2;
                        status_track = SingleLineRoad;
                        break;
                    }
                    if((64-i-4)>40 && single.g_u8SLLeftWit>2 && single.g_u8SLRightWit > 10)
                    {
                        single.g_u8SLPosition = i+2;
                        status_track = SingleLineRoad;
                        break;
                    }
                }
            }
        }
    }
}

void SLScanBothEdge()
{
    for(int i=single.g_u8SLPosition;i>5;i--) 
    {
        if((indata.Pixel[i]-indata.Pixel[i-3]) > setpara.CCD1EdgeTH && abs(i-single.g_u8SLPosition)>5)
        {
            track.g_s16LEdge = i; //�Ҳ����Ͳ�����
            break;
        }
        else
        {
            track.g_s16LEdge = 3;
        }            
    }
    
    for(int i=single.g_u8SLPosition;i<122;i++)
    {
        if((indata.Pixel[i]-indata.Pixel[i+3]) > setpara.CCD1EdgeTH && abs(i-single.g_u8SLPosition)>5)
        {
            track.g_s16REdge = i;
            break;
        }
        else 
        {
            track.g_s16REdge = 124;
        }
    }
}

void SingleLineTrack()
{
    single.g_u8FindSL = 0;
    //-7��ֹ��������ұ���
    for(int i=track.g_s16LEdge;i<track.g_s16REdge-7;i++)
    {
        if((indata.Pixel[i]-indata.Pixel[i+3]) > setpara.CCD1EdgeTH)
        {
            for(int j=i+3;j<track.g_s16REdge;j++)
            {
                if((indata.Pixel[j]-indata.Pixel[j+3]) < -setpara.CCD1EdgeTH)
                {
                    single.g_u8SingleLW = j-i; 
                    break;
                }
            }
            if(single.g_u8SingleLW >= 2 && single.g_u8SingleLW <=10)
            {
                //��������ɫ������
                single.g_u8SLLeftWit = i-track.g_s16LEdge;
                //���Ҽ����ɫ������
                single.g_u8SLRightWit = track.g_s16REdge-(i+single.g_u8SingleLW+2);
                //�����������ұ� i+4Ϊ�������м�
                if((i+4)>=64)
                {
                    if((i+4-64)<=30 && single.g_u8SLLeftWit>15 && single.g_u8SLRightWit > 15)
                    {
                        single.g_u8SLPosition = i+2;
                        single.g_u8FindSL = 1;
                        status_track = SingleLineRoad;                           
                        break; //������forѭ��
                    }
                    if((i+4-64)>30 && (i+4-64)<=40 && single.g_u8SLLeftWit>10 && single.g_u8SLRightWit > 5)
                    {
                        single.g_u8SLPosition = i+2;
                        single.g_u8FindSL = 1;
                        status_track = SingleLineRoad;
                        break;
                    }
                    if((i+4-64)>40 && single.g_u8SLLeftWit>10 && single.g_u8SLRightWit > 2)
                    {
                        single.g_u8SLPosition = i+2;
                        single.g_u8FindSL = 1;
                        status_track = SingleLineRoad;
                        break;
                    }
                }
                //������������� i+4Ϊ�������м�
                if((i+4)<64)
                {
                    if((64-i-4)<=30 && single.g_u8SLLeftWit>15 && single.g_u8SLRightWit > 15)
                    {                        
                        single.g_u8SLPosition = i+2;
                        single.g_u8FindSL = 1;
                        status_track = SingleLineRoad;                           
                        break; //������forѭ��
                    }
                    if((64-i-4)>30 && (64-i-4)<=40 && single.g_u8SLLeftWit>5 && single.g_u8SLRightWit > 10)
                    {
                        single.g_u8SLPosition = i+2;
                        single.g_u8FindSL = 1;
                        status_track = SingleLineRoad;
                        break;
                    }
                    if((64-i-4)>40 && single.g_u8SLLeftWit>2 && single.g_u8SLRightWit > 10)
                    {
                        single.g_u8SLPosition = i+2;
                        single.g_u8FindSL = 1;
                        status_track = SingleLineRoad;
                        break;
                    }
                }
            }
        }
    }
    if(single.g_u8FindSL) //�����������������
    {
        track.g_s16DirBiasNew = single.g_u8SLPosition - 64;
        ramp.g_u8RampSLCount ++;
    }
    else
    { 
        status_track = DoubleLineRoad;
        track.g_s16DycMidP = 64;   
        //��ͣ10�������ұ��ߣ��������ȶ����������ұ��ߣ�
        track.g_u8LineContinueDetectPause = 1;
        //���˳����߳����ʱ�������е���ȫ�ֱ���
        memset(&single,0,sizeof(struct SingleLine_STRUCT));
        
        if(ramp.g_u8RampSLCount >= setpara.RampSLCountN)
        {
            ramp.g_u8RampSLCount = 0;
            //��������
            ramp.g_u8RampSLNum ++; 
        }
        
        if(ramp.g_u8RampOnce == 0 && setpara.RampF == 2 && ramp.g_u8RampSLNum == setpara.RampSLNum)
            ramp.g_u8RampSLF = 1;
    }
}

//�µ��㷨�������ģ���û����ǰ�������ǰ��ǶȲ��󣬵���ģ������������ȱ���˺ܶ࣬����Ϊ���µ���
void RampDetect()
{
    if(rightangle.g_u8RABarNum == 2 && setpara.RampF == 1 && ramp.g_u8RampOnce == 0)
        ramp.g_u8DelayNum2 ++ ;
    if(ramp.g_u8RampSLF == 1 && setpara.RampF == 2 && ramp.g_u8RampOnce == 0)
        ramp.g_u8DelayNum2 ++ ;
    if(ramp.g_u8RampBarrierF == 1 && setpara.RampF == 3 && ramp.g_u8RampOnce == 0)
        ramp.g_u8DelayNum2 ++ ;
    //�µ�ֻ�ж�һ��
    if(rightangle.g_u8RABarNum == 2 && setpara.RampF == 1 && track.g_s16DirBiasNew < 5 && ramp.g_u8RampOnce == 0 && ramp.g_u8DelayNum2 > setpara.RampRAD)
    {
        status_track = RampRoad;
        track.g_s16DycMidP = 64;
        track.g_s16DycMidP2 = 64;
        ramp.g_u8RampOnce = 1;
        control.g_s16DirGyroP = setpara.RampGyroP;
        ramp.g_u8DelayNum2 = 0;
    }
    if(ramp.g_u8RampSLF == 1 && setpara.RampF == 2 && track.g_s16DirBiasNew < 5 && ramp.g_u8RampOnce == 0 && ramp.g_u8DelayNum2 > setpara.RampSLD)
    {
        status_track = RampRoad;
        track.g_s16DycMidP = 64;
        track.g_s16DycMidP2 = 64;
        ramp.g_u8RampOnce = 1;
        ramp.g_u8RampSLF = 0;
        control.g_s16DirGyroP = setpara.RampGyroP;
        ramp.g_u8DelayNum2 = 0;
    }
    if(ramp.g_u8RampBarrierF == 1 && setpara.RampF == 3 && track.g_s16DirBiasNew < 5 && ramp.g_u8RampOnce == 0 && ramp.g_u8DelayNum2 > setpara.RampBarriD)
    {
        status_track = RampRoad;
        track.g_s16DycMidP = 64;
        track.g_s16DycMidP2 = 64;
        ramp.g_u8RampOnce = 1;
        ramp.g_u8RampBarrierF = 0;
        control.g_s16DirGyroP = setpara.RampGyroP;
        ramp.g_u8DelayNum2 = 0;
    }
}


//�㷨˼·��˫�����ж�ֱ����˫�ߺ�ֱ�������Խ����ߡ��ϰ�ֻ����ֱ�����жϡ�ֱ���ж��ڵ�˫���ж��С��ϰ�ֱ���ں󣬲�Ӱ�������жϡ�
//          ֱ��Ѳ���������ֱ���˳�����������ǰΪ�ж��߻�ƫ����ͺ��ж�һ�����ڣ������ϰ�ֱ�������жϡ�
//          �����˳�����Ϊ�Ҳ���������������˫�ߡ�ֱ��Ϊת����ҵ�������������˫�ߡ��ϰ�Ϊ���ϰ����������ߡ�
void ccd_cal()
{  
    //���������������� ͼ���� ������������������
    //CCDͼ��ɼ�
    
    //    PITValA = PIT->CHANNEL[PIT1].CVAL;
    //    PITValB = PIT->CHANNEL[PIT1].CVAL;
    //    PITVal = PITValA - PITValB; 
    
    //���������������� ·���㷨 ������������������
    switch(status_track)
    {
      case DoubleLineRoad:
        
        //���������������� ģʽʶ�� ������������������
        
        if(status_track == DoubleLineRoad)
        {
            //CCD2Detect();
            CommonRoadDriving();
        }
        
        CrossRoadDetect();
        
        RightAngleDetect();
        
        //�������ж�
        BarrierDetect();
        
        if(status_track == DoubleLineRoad)
            RampDetect();
        
        SingleLineDetect();
        
        if(mycar.g_u8status == 0)
            CCD2Detect();
        
        break;
        
      case CrossRoad:      
        PassCrossRoad();
        
        break;
      case RightAngleRoad:
        CommonRoadDriving();
        if(track.g_u8LMissP == 0 && track.g_u8RMissP == 0)
            status_track = RATurnDetect;
        
        break;
      case RATurnDetect:
        PassRARoad();
        
        break;
      case RightAngleTurnLeft:
        track.g_s16DirBiasNew = RightAngleTurn(1);
        
        break;
      case RightAngleTurnRight:
        track.g_s16DirBiasNew = RightAngleTurn(2);
        
        break;
      case LeftBarrier:
        barrier.g_u8BarrierCounter ++;
        
        EdgeDetect(track.g_s16DycMidP);
        
        if(track.g_s16DycMidP >= 64)
        {
            track.g_s16DirBiasNew = setpara.BarrierLBias;
        }
        else
        {
            track.g_s16DirBiasNew = (barrier.g_u8BarrierEdge+track.g_s16REdge)/2-64;
        }
        if(barrier.g_u8BarrierCounter >= barrier.g_u8BarrierNum)
        {
            status_track = DoubleLineRoad;
            //�˳��ָ�΢��
            barrier.g_s16BarrierGyroRecover = 1;
            barrier.g_u8BarrierCounter = 0;
            track.g_u8LineContinueDetectPause = 1;
            //�ڴ�����������´��жϻ�����֮ǰ������ϰ���
            barrier.g_u8BarrierWidth = 0;
            //��������
            ramp.g_u8RampBarrierC ++;
            if(ramp.g_u8RampOnce == 0 && setpara.RampF == 3 && ramp.g_u8RampBarrierC == setpara.RampBarriNum)
                ramp.g_u8RampBarrierF = 1;
        }
        
        break;
      case RightBarrier:
        barrier.g_u8BarrierCounter ++;
        
        EdgeDetect(track.g_s16DycMidP);
        
        if(track.g_s16DycMidP <= 64)
        {
            track.g_s16DirBiasNew = setpara.BarrierRBias;
        }
        else
        {
            track.g_s16DirBiasNew = (track.g_s16LEdge+barrier.g_u8BarrierEdge)/2-64;
        }
        if(barrier.g_u8BarrierCounter >= barrier.g_u8BarrierNum)
        {
            status_track = DoubleLineRoad;
            //�˳��ָ�΢��
            barrier.g_s16BarrierGyroRecover = 1;
            control.g_s16DirGyroP = setpara.DirPD.KdGyro;
            barrier.g_u8BarrierCounter = 0;
            track.g_u8LineContinueDetectPause = 1;
            //�ڴ�����������´��жϻ�����֮ǰ������ϰ���
            barrier.g_u8BarrierWidth = 0;
            //��������
            ramp.g_u8RampBarrierC ++;
            if(ramp.g_u8RampOnce == 0 && setpara.RampF == 3 && ramp.g_u8RampBarrierC == setpara.RampBarriNum)
                ramp.g_u8RampBarrierF = 1;
        }
        
        
        break;
      case SingleLineRoad:
        
        SLScanBothEdge();
        
        SingleLineTrack();
        
        break;
      case RampRoad:
        test.g_u8BuzzerRing = 1;
        CCD2Detect();
        track.g_s16DirBiasNew = track.g_s16DirBiasNew2;
        //track.g_s16DirBiasNew = track.g_s16DycMidP - 64;
        if(track.g_s16DirBiasNew > setpara.BiasLim)
            track.g_s16DirBiasNew = setpara.BiasLim;
        if(track.g_s16DirBiasNew < -setpara.BiasLim)
            track.g_s16DirBiasNew = -setpara.BiasLim;
        //track.g_s16DirBiasNew = 0;
        control.g_s16TargetSpeed = setpara.RampSpeed;
        setpara.BalancePD.Kp = 120;
        setpara.BalancePD.Kd = 4;
        //        setpara.SpeedPI.KpSteady = 26;
        //        setpara.SpeedPI.Ki = 4;
        ramp.g_u8DelayNum ++;
        if(ramp.g_u8DelayNum <= setpara.RampBrakeN)
            control.g_s16BalanceAngle += 20;
        if(ramp.g_u8DelayNum >= setpara.RampDNum)
        {
            test.g_u8BuzzerRing = 0;
            ramp.g_u8DelayNum = 0;
            status_track = DoubleLineRoad;
            track.g_s16DycMidP = 64;   
            track.g_s16DycMidP2 = 64;   
            track.g_u8LineContinueDetectPause = 1;
            //            setpara.BalancePD.Kp = 90;
            //            setpara.BalancePD.Kd = 3;
            control.g_s16TargetSpeed = setpara.TargetSpeed;
            ramp.g_u8BrakeS = 1;
            setpara.BalancePD.Kp = 90;
            setpara.BalancePD.Kd = 2;
            //            setpara.SpeedPI.KpSteady = 42;
            //            setpara.SpeedPI.Ki = 10;
            control.g_s16DirGyroP = setpara.DirPD.KdGyro;
        }
        
        //        if(track.RoadWidth < 30)
        //        {
        //            ramp.g_u8RampDown = 1;
        //            control.g_s16TargetSpeed = 10;
        //            //ramp.g_u8Brake = 1;     
        //        }
        //        if(ramp.g_u8RampDown == 1 && track.RoadWidth > 55)
        //        {
        //            //            test.g_u8BuzzerRing = 0;
        //            ramp.g_u8RampDown = 0;
        //            control.g_s16TargetSpeed = 29;
        //            track.g_s16DycMidP = 64;
        //            track.g_u8LineContinueDetectPause = 1;
        //            status_track = DoubleLineRoad;
        //            //            if(indata.g_fCarAngle > 0)
        //            //                control.g_s16BalanceAngle -= 2;
        //            //            if(control.g_s16BalanceAngle <= -100)
        //            //                control.g_s16BalanceAngle =-100;
        //        }
        
        break;
      default:
        break;
    }
    
    //���һ��ƫ���������
    track.g_s16DirBiasDiff = track.g_s16DirBiasNew - track.g_s16DirBiasOld;
    track.g_s16DirBiasOld = track.g_s16DirBiasNew;
    
    //PDD����
    if(track.g_s16DirBiasNew > setpara.RightTurnLimit)
        track.g_s16DirBiasNew = setpara.RightTurnLimit;
    if(track.g_s16DirBiasNew < setpara.LeftTurnLimit)
        track.g_s16DirBiasNew = setpara.LeftTurnLimit;
    
    if((T-mycar.g_u16ActiveT) <= setpara.DirIniT*1000)
        control.g_s16DirP = setpara.DirBaseIni;
    else
    {
        if(abs(track.g_s16DirBiasNew)<=15)
            control.g_s16DirP = setpara.DirBase;
        else
            control.g_s16DirP = setpara.DirBase + track.g_s16DirBiasNew*track.g_s16DirBiasNew/setpara.DirCoef;
    }
    
    control.g_s16DirKdDiff = control.g_s16DirP*setpara.DirKdDifK/10;
    
    control.g_fDirectionControlOutOld = control.g_fDirectionControlOutNew;
    control.g_fDirectionControlOutNew = track.g_s16DirBiasNew*control.g_s16DirP + track.g_s16DirBiasDiff*control.g_s16DirKdDiff - (int)(indata.g_fAngleDotDir*control.g_s16DirGyroP/10);
    
    
    //SD������д���������������������������������
    if(mycar.g_u8status >= 2)  
    {   
        for(int i=3;i<125;i++)
        {
            save.g_u8SDBuffer[save.g_u32SDBufNum%50][i-3] = indata.Pixel[i];
        }
        //        for(int i=10;i<118;i++)
        //        {
        //            save.g_u8SDBuffer[save.g_u32SDBufNum%50][i+112] = indata.Pixel2[i];
        //        }
        
        save.g_u8ImageCounter ++;
        if(save.g_u8ImageCounter == 200)
        {
            save.g_u8ImageCounter = 0;
            save.g_u8ImageTimes ++;
        }
        
        //17
        SDSavePara((int)save.g_u8ImageCounter,0);
        SDSavePara((int)save.g_u8ImageTimes,1);
        
        //ֱ��
        SDSavePara((int)rightangle.g_u8RABarNum,9);
        SDSavePara((int)PTB2_I,13);
        SDSavePara((int)PTB10_I,14);
        SDSavePara((int)track.g_u8LineContineStop,21);
        
        //ֱ��
        SDSavePara((int)rightangle.g_u8LLoseF,22);
        SDSavePara((int)rightangle.g_u8RLoseF,23);
        
        //�ϰ� 
        SDSavePara((int)barrier.g_u8BarrierWidth,8);
        SDSavePara((int)barrier.g_u8BarrierEdge,18);
        SDSavePara((int)barrier.g_u8BarrierNum,40);
        
        //����
        //��鵥�߿�ȣ��������Ұ�ɫ������
        //�������SingleLinePosition�Ƿ�С��5
        SDSavePara((int)single.g_u8SLLeftWit,25);
        SDSavePara((int)single.g_u8SLRightWit,26);
        SDSavePara((int)single.g_u8SLPosition,27);
        SDSavePara((int)single.g_u8FindSL,28);
        
        //�µ�
        
        SDSavePara((int)mycar.g_fbatt_volt*10,29);
        
        //Ѳ��        
        SDSavePara((int)status_track,7);
        SDSavePara((int)indata.g_fCarAngle,3);
        SDSavePara((int)indata.g_fCarSpeed,4);
        
        SDSavePara((int)track.RoadWidth,15);
        SDSavePara((int)track.RoadWidth2,6);
        SDSavePara((int)track.g_s16DirBiasNew,2);
        SDSavePara((int)track.g_s16DycMidP,5);
        
        SDSavePara((int)track.g_u8LMissP,11);
        SDSavePara((int)track.g_s16LEdge,19);
        
        SDSavePara((int)track.g_u8RMissP,12);      
        SDSavePara((int)track.g_s16REdge,20);    
        
        SDSavePara((int)track.g_u8LineContinueDetectPause,32);
        SDSavePara((int)track.g_u8LineContineStop,33);      
        SDSavePara((int)mycar.g_u8status,34);  
        SDSavePara((int)track.g_u8LineContinueCounter,35);  
        
        //�ٶ�
        SDSavePara((int)test.SpeedP,36);      
        SDSavePara((int)control.g_fSpeedControlIntegral,37);  
        SDSavePara((int)outdata.g_fSpeedControlOut,38); 
        
        //����
        SDSavePara((int)outdata.g_fDirectionControlOut,39); 
        
        //ActiveT
        SDSavePara((int)mycar.g_u16ActiveT,10); 
        SDSavePara((int)tower.g_u8Tower_vol,24); 
        SDSavePara((int)mycar.g_u8status,30); 
        SDSavePara((int)tower.g_s8TowerCounterF,31); 
        SDSavePara((int)tower.g_u8Finish,41); 
        SDSavePara((int)tower.g_s8TowerCounter,42); 
        
        //�����ٶ�
        SDSavePara((int)test.g_s16speedsumL,43); 
        SDSavePara((int)test.g_s16speedsumR,44); 
        SDSavePara((int)control.g_s16BalanceAngle,45); 
        SDSavePara((int)outdata.g_fAngleControlOut,46);
        SDSavePara((int)track.g_s16DirBiasNew2,47);
        SDSavePara((int)ramp.g_u8DelayNum2,48);
        SDSavePara((int)test.g_u8BuzzerRing,49);
        SDSavePara((int)PTB21_O,50);
        
        save.g_u32SDBufNum ++;
    }
}
