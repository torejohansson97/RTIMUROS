#include "RTIMUBERRYIMU.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation
#define COMPASS_ALPHA 0.2f

RTIMUBERRYIMU::RTIMUBERRYIMU(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
    m_settings->m_BERRYIMUGyroSampleRate = LSM6DSL_ODR_6660Hz;
	m_settings->m_BERRYIMUAccelSampleRate = LSM6DSL_ODR_6660Hz;
    m_settings->m_BERRYIMUCompassSampleRate = LIS3MDL_MAG_OM_XY_ULTRAHIGH | LIS3MDL_MAG_ODR_80_HZ | 0x02;
	m_settings->m_BERRYIMUGyroFsr = LSM6DSL_GYRO_FS_245;
	m_settings->m_BERRYIMUAccelFsr = LSM6DSL_ACC_FULLSCALE_8G;
	m_settings->m_BERRYIMUAccelLpf = LSM6DSL_XL_LPF_400;
}

RTIMUBERRYIMU::~RTIMUBERRYIMU()
{
}

bool RTIMUBERRYIMU::IMUInit()
{
    unsigned char result;

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    m_gyroAccelAddr = LSM6DSL_ADDRESS;
    m_compassAddr = LIS3MDL_ADDRESS;

    setCalibrationData();

    // enable I2C bus
    if (!m_settings->HALOpen())
        return false;

    // set up the LSM6DSL
    if (!m_settings->HALWrite(m_gyroAccelAddr, LSM6DSL_ACC_GYRO_CTRL3_C, 0x01, "Failed to reset LSM6DSL"))
        return false;

    if (!m_settings->HALRead(m_gyroAccelAddr, LSM6DSL_ACC_GYRO_WHO_AM_I_REG, 1, &result, "Faild to read LSM6DSL ID"))
        return false;

    if (result != LSM6DSL_ACC_GYRO_WHO_AM_I)
    {
        HAL_ERROR1("Incorrect LSM6DSL id %d\n", result);
        return false;
    }

    // set up the gyro
    if (!setGyroControl())
        return false;
    gyroBiasInit();

    // set up the accel
    if (!setAccelControl())
        return false;

    // set up the LIS3MDL
    if(!m_settings->HALWrite(m_compassAddr, LIS3MDL_MAG_CTRL_REG2, 0x04, "Failed to reset LIS3MDL"))
        return false;

    // power mode (Continuous-conversion mode )
    if(!m_settings->HALWrite(m_compassAddr, LIS3MDL_MAG_CTRL_REG3, 0x00, "Failed to set LIS3MDL Power Mode"))
        return false;

    // full scale configuration (8 gauss)
    if(!m_settings->HALWrite(m_compassAddr, LIS3MDL_MAG_CTRL_REG2, 0x20, "Failed to set LIS3MDL Full Scale Reg"))
        return false;
    
    m_compassScale = LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA / 1000;
    
    if(!m_settings->HALRead(m_compassAddr, LIS3MDL_MAG_WHO_AM_I_REG, 1, &result, "Failed to read LIS3MDL ID"))
        return false;

    if(result != I_AM_LIS3MDL){
        HAL_ERROR1("Incorrect LIS3MDL id %d\n", result);
        return false;
    }

    // set up the compass
    if (!setCompassControl())
        return false;

    HAL_INFO("BerryIMU init complete\n");
    return true;
}

bool RTIMUBERRYIMU::setGyroControl()
{
    unsigned char ctrl;

    switch (m_settings->m_BERRYIMUGyroSampleRate)
    {
    case LSM6DSL_ODR_POWER_DOWN:
        ctrl = 0x00;
        // m_sampleRate = 0; FIXME: Zero Division
        break;
    case LSM6DSL_ODR_13Hz:
        ctrl = 0x10;
        m_sampleRate = 12.5;
        break;
    case LSM6DSL_ODR_26Hz:
        ctrl = 0x20;
        m_sampleRate = 26;
        break;
    case LSM6DSL_ODR_52Hz:
        ctrl = 0x30;
        m_sampleRate = 52;
        break;
    case LSM6DSL_ODR_104Hz:
        ctrl = 0x40;
        m_sampleRate = 104;
        break;
    case LSM6DSL_ODR_208Hz:
        ctrl = 0x50;
        m_sampleRate = 208;
        break;
    case LSM6DSL_ODR_416Hz:
        ctrl = 0x60;
        m_sampleRate = 416;
        break;
    case LSM6DSL_ODR_833Hz:
        ctrl = 0x70;
        m_sampleRate = 833;
        break;
    case LSM6DSL_ODR_1660Hz:
        ctrl = 0x80;
        m_sampleRate = 1600;
        break;
    case LSM6DSL_ODR_3330Hz:
        ctrl = 0x90;
        m_sampleRate = 3330;
        break;
    case LSM6DSL_ODR_6660Hz:
        ctrl = 0xA0;
        m_sampleRate = 6600;
        break;
    default:
        HAL_ERROR1("Illigal LSM6DSL gyro output data rate %d\n", m_settings->m_BERRYIMUGyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_BERRYIMUGyroFsr)
    { // TODO: Scale factors
    case LSM6DSL_GYRO_FS_245:
        ctrl |= 0x00;
        m_gyroScale = (RTFLOAT)0.0763 * RTMATH_DEGREE_TO_RAD;
        break;
    case LSM6DSL_GYRO_FS_500:
        ctrl |= 0x04;
        m_gyroScale = (RTFLOAT)0.0153 * RTMATH_DEGREE_TO_RAD;
        break;
    case LSM6DSL_GYRO_FS_1000:
        ctrl |= 0x08;
        m_gyroScale = (RTFLOAT)0.0305 * RTMATH_DEGREE_TO_RAD;
        break;
    case LSM6DSL_GYRO_FS_2000:
        ctrl |= 0x0A;
        m_gyroScale = (RTFLOAT)0.0610 * RTMATH_DEGREE_TO_RAD;
        break;
    default:
        HAL_ERROR1("Illigal LSM6DSL gyro full scale %d\n", m_settings->m_BERRYIMUGyroFsr);
    }

    return (m_settings->HALWrite(m_gyroAccelAddr, LSM6DSL_ACC_GYRO_CTRL2_G, ctrl, "Failed to set LSM6DSL CTRL2_G"));
}

bool RTIMUBERRYIMU::setAccelControl()
{
    unsigned char ctrl;

    switch (m_settings->m_BERRYIMUAccelSampleRate)
    {
    case LSM6DSL_ODR_POWER_DOWN:
        ctrl = 0x00;
        // m_sampleRate = 0; FIXME: Zero Division
        break;
    case LSM6DSL_ODR_13Hz:
        ctrl = 0x10;
        m_sampleRate = 12.5;
        break;
    case LSM6DSL_ODR_26Hz:
        ctrl = 0x20;
        m_sampleRate = 26;
        break;
    case LSM6DSL_ODR_52Hz:
        ctrl = 0x30;
        m_sampleRate = 52;
        break;
    case LSM6DSL_ODR_104Hz:
        ctrl = 0x40;
        m_sampleRate = 104;
        break;
    case LSM6DSL_ODR_208Hz:
        ctrl = 0x50;
        m_sampleRate = 208;
        break;
    case LSM6DSL_ODR_416Hz:
        ctrl = 0x60;
        m_sampleRate = 416;
        break;
    case LSM6DSL_ODR_833Hz:
        ctrl = 0x70;
        m_sampleRate = 833;
        break;
    case LSM6DSL_ODR_1660Hz:
        ctrl = 0x80;
        m_sampleRate = 1600;
        break;
    case LSM6DSL_ODR_3330Hz:
        ctrl = 0x90;
        m_sampleRate = 3330;
        break;
    case LSM6DSL_ODR_6660Hz:
        ctrl = 0xA0;
        m_sampleRate = 6600;
        break;
    default:
        HAL_ERROR1("Illigal LSM6DSL accel output data rate %d\n", m_settings->m_BERRYIMUAccelSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_BERRYIMUAccelFsr)
    {
    case LSM6DSL_ACC_FULLSCALE_2G:
        ctrl |= 0x00;
        m_accelScale = (RTFLOAT)0.000061;
        break;
    case LSM6DSL_ACC_FULLSCALE_4G:
        ctrl |= 0x04;
        m_accelScale = (RTFLOAT)0.000122;
        break;
    case LSM6DSL_ACC_FULLSCALE_8G:
        ctrl |= 0x08;
        m_accelScale = (RTFLOAT)0.000244;
        break;
    case LSM6DSL_ACC_FULLSCALE_16G:
        ctrl |= 0x0A;
        m_accelScale = (RTFLOAT)0.000488;
        break;
    default:
        HAL_ERROR1("Illigal LSM6DSL accel full scale %d\n", m_settings->m_BERRYIMUAccelFsr);
    }

    // Enable digital LPF
    ctrl |= 0x02;

    switch (m_settings->m_BERRYIMUAccelLpf)
    {
    case LSM6DSL_XL_LPF_1500:
        ctrl |= 0x00;
        break;
    case LSM6DSL_XL_LPF_400:
        ctrl |= 0x01;
        break;
    default:
        HAL_ERROR1("Illigal LSM6DSL accel bandwidth %d\n", m_settings->m_BERRYIMUAccelLpf);
    }

    return (m_settings->HALWrite(m_gyroAccelAddr, LSM6DSL_ACC_GYRO_CTRL1_XL, ctrl, "Failed to set LSM6DSL CTRL1_XL"));
}

bool RTIMUBERRYIMU::setCompassControl()
{
    unsigned char ctrl;
    
    switch ((m_settings->m_BERRYIMUCompassSampleRate & 0x1C)) // DO bits
    {
    case LIS3MDL_MAG_ODR_0_625_HZ:
        ctrl = 0x00;
        break;
    case LIS3MDL_MAG_ODR_1_25_HZ:
        ctrl = 0x04;
        break;
    case LIS3MDL_MAG_ODR_2_5_HZ:
        ctrl = 0x08;
        break;
    case LIS3MDL_MAG_ODR_5_0_HZ:
        ctrl = 0x0C;
        break;
    case LIS3MDL_MAG_ODR_10_HZ:
        ctrl = 0x10;
        break;
    case LIS3MDL_MAG_ODR_20_HZ:
        ctrl = 0x14;
        break;
    case LIS3MDL_MAG_ODR_40_HZ:
        ctrl = 0x18;
        break;
    case LIS3MDL_MAG_ODR_80_HZ:
        ctrl = 0x1C;
        break;
    default:
        HAL_ERROR1("Illigal LIS3MDL sample rate %d\n", (m_settings->m_BERRYIMUAccelSampleRate & 0x1C));
    }

    switch ((m_settings->m_BERRYIMUAccelSampleRate & 0x60)) // OM bits
    {
    case LIS3MDL_MAG_OM_XY_LOWPOWER:
        ctrl |= 0x00;
        break;
    case LIS3MDL_MAG_OM_XY_MEDIUM:
        ctrl |= 0x20;
        break;
    case LIS3MDL_MAG_OM_XY_HIGH:
        ctrl |= 0x40;
        break;
    case LIS3MDL_MAG_OM_XY_ULTRAHIGH:
        ctrl |= 0x60;
        break;
    default:
        HAL_ERROR1("Illigal LIS3MDL OM_XY %d\n", (m_settings->m_BERRYIMUAccelSampleRate & 0x60));
    }

    switch ((m_settings->m_BERRYIMUAccelSampleRate & 0x02)) // FAST_ODR bit
    {
    case 0x00:
        ctrl |= 0x00;
        break;
    case 0x02:
        ctrl |= 0x02;
        break;
    default:
        HAL_ERROR1("Illigal LIS3MDL FAST_ODR %d\n", (m_settings->m_BERRYIMUAccelSampleRate & 0x02));
    }
    
    return (m_settings->HALWrite(m_compassAddr, LIS3MDL_MAG_CTRL_REG1, ctrl, "Failed to set LIS3MDL CTRL_REG1"));
}

// TODO: Rest of the set functions

int RTIMUBERRYIMU::IMUGetPollInterval()
{
    return (400 / m_sampleRate); // TODO: Is this correct?
}

bool RTIMUBERRYIMU::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    if (!m_settings->HALRead(m_gyroAccelAddr, LSM6DSL_ACC_GYRO_STATUS_REG, 1, &status, "Failed to read LSM6DSL status"))
        return false;
    
    if ((status & 0x07) == 0)
        return false;

    if (!m_settings->HALRead(m_gyroAccelAddr, LSM6DSL_ACC_GYRO_OUTX_L_G, 6, gyroData, "Failed to read LSM6DSL gyro data"))
        return false;

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    if (!m_settings->HALRead(m_gyroAccelAddr, LSM6DSL_ACC_GYRO_OUTX_L_XL, 6, accelData, "Failed to read LSM6DSL accel data"))
        return false;

    if (!m_settings->HALRead(m_gyroAccelAddr, LIS3MDL_MAG_STATUS_REG, 1, &status, "Failed to read LIS3MDL status"))
        return false;

    if ((status & 0x0F) == 0)
        return false;

    if(!m_settings->HALRead(m_compassAddr, LIS3MDL_MAG_OUTX_L, 6, compassData, "Failed to read LIS3MDL compass data")) return false;

    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_imuData.accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale, false);

    // sort out gyro axes and correct for bias

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    // sort out accel axes

    m_imuData.accel.setX(-m_imuData.accel.x());

    // sort out compass axes

    m_imuData.compass.setY(-m_imuData.compass.y());

    // standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    // update filter

    updateFusion();

    return true;
}