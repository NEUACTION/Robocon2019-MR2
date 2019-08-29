#ifndef __DATA_H
#define __DATA_H

#include "moveBase.h"

void DataRecord(void);
void DataConversion(void);
void GetFootEndCoordinate_B(leg_ *leg);
void Compensate(void);
void DistanceConversion_RED(void);
void DistanceConversion_BLUE(void);
void GetLegRD_RED(leg_ *leg);
void GetLegRD_BLUE(leg_ *leg);
coordinate RCT(leg_ leg, float *w);

#endif





