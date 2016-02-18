#ifndef _CCDARITHMETIC_H
#define _CCDARITHMETIC_H

#include "common.h"
#include "car_global.h"

void ccd_cal();
void CommonRoadDriving();
void CCD1BarrierDriving();
void EdgeDetect(int16 midline);
void CrossRoadDetect();
void RightAngleDetect();
int RightAngleTurn(uint8 num);
void SDSavePara(int para,uint16 num);
void PassCrossRoad();
void PassRARoad();void CCD2Detect();
void BarrierDetect();
void SingleLineDetect();
void SLScanBothEdge();
void SingleLineTrack();
void RampDetect();
//void CCD3EdgeDetect(int16 midline);

#endif