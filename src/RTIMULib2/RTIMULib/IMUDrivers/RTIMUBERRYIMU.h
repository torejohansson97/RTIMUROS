#ifndef _RTIMUBERRYIMU_H
#define _RTIMUBERRYIMU_H

#include "RTIMU.h"

class RTIMUBERRYIMU : public RTIMU
{
public:
    RTIMUBERRYIMU(RTIMUSettings *settings);
    ~RTIMUBERRYIMU();

    virtual const char *IMUName() { return "BERRYIMU"; }
    virtual int IMUType() { return RTIMU_TYPE_BERRYIMU; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool setGyroControl();
    bool setAccelControl();
    bool setCompassControl();

    unsigned char m_gyroAccelAddr;          // I2C address of gyro and accel
    unsigned char m_compassAddr;       // I2C address of mag

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;
};

#endif