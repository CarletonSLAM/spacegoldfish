
#ifndef __SPACEGOLDFISH_ACCELEROMETER_H__
#define __SPACEGOLDFISH_ACCELEROMETER_H__


#define M_PI 3.14159265358979323846 


void accel_setup(void);
void setAcceltoActive(void);
void readAccelData(void);
void setAcceltoStandby(void);
void calcTiltAnglesFromAccelData(void);

#endif /*__SPACEGOLDFISH_ACCELEROMETER_H__*/