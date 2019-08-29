#ifndef __DATAOUT_H
#define __DATAOUT_H

void ResetDataSendOut(void);
void DataSendOut(void);
void SensorDataOut(void);
void adjustDataOut(void);
void MotorDataOut(void);
void GetODOut(void);
void CANErrorNumber(void);
void GyroDataOut(void);

void ResetDataOut(void);
#endif

