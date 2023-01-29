/*
 Note: The MPU6886 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or
 a 3.3 V Teensy 3.1. We have disabled the internal pull-ups used by the Wire
 library in the Wire.h/twi.c utility file. We are also using the 400 kHz fast
 I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#ifndef _MPU6886s_H_
#define _MPU6886s_H_

#include <Arduino.h>
#include <Wire.h>

#define MPU6886_SELF_TEST_X_ACCEL  0x0D
#define MPU6886_SELF_TEST_Y_ACCEL  0x0E
#define MPU6886_SELF_TEST_Z_ACCEL  0x0F

#define MPU6886_ADDRESS            0x68
#define MPU6886_WHOAMI             0x75
#define MPU6886_ACCEL_INTEL_CTRL   0x69
#define MPU6886_SMPLRT_DIV         0x19
#define MPU6886_INT_PIN_CFG        0x37
#define MPU6886_INT_ENABLE         0x38
#define MPU6886_FIFO_WM_INT_STATUS 0x39
#define MPU6886_INT_STATUS         0x3A
#define MPU6886_ACCEL_WOM_X_THR    0x20
#define MPU6886_ACCEL_WOM_Y_THR    0x21
#define MPU6886_ACCEL_WOM_Z_THR    0x22

#define MPU6886_ACCEL_XOUT_H 0x3B
#define MPU6886_ACCEL_XOUT_L 0x3C
#define MPU6886_ACCEL_YOUT_H 0x3D
#define MPU6886_ACCEL_YOUT_L 0x3E
#define MPU6886_ACCEL_ZOUT_H 0x3F
#define MPU6886_ACCEL_ZOUT_L 0x40

#define MPU6886_TEMP_OUT_H 0x41
#define MPU6886_TEMP_OUT_L 0x42

#define MPU6886_GYRO_XOUT_H 0x43
#define MPU6886_GYRO_XOUT_L 0x44
#define MPU6886_GYRO_YOUT_H 0x45
#define MPU6886_GYRO_YOUT_L 0x46
#define MPU6886_GYRO_ZOUT_H 0x47
#define MPU6886_GYRO_ZOUT_L 0x48

#define MPU6886_USER_CTRL     0x6A
#define MPU6886_PWR_MGMT_1    0x6B
#define MPU6886_PWR_MGMT_2    0x6C
#define MPU6886_CONFIG        0x1A
#define MPU6886_GYRO_CONFIG   0x1B
#define MPU6886_ACCEL_CONFIG  0x1C
#define MPU6886_ACCEL_CONFIG2 0x1D

#define MPU6886_ACCEL_XA_OFFSET_H 0x77
#define MPU6886_ACCEL_XA_OFFSET_L 0x78
#define MPU6886_ACCEL_YA_OFFSET_H 0x7A
#define MPU6886_ACCEL_YA_OFFSET_L 0x7B
#define MPU6886_ACCEL_ZA_OFFSET_H 0x7D
#define MPU6886_ACCEL_ZA_OFFSET_L 0x7E

#define MPU6886_GYRO_X_OFFS_USRH 0x13
#define MPU6886_GYRO_X_OFFS_USRL 0x14
#define MPU6886_GYRO_Y_OFFS_USRH 0x15
#define MPU6886_GYRO_Y_OFFS_USRL 0x16
#define MPU6886_GYRO_Z_OFFS_USRH 0x17
#define MPU6886_GYRO_Z_OFFS_USRL 0x18

#define MPU6886_FIFO_EN       0x23
#define MPU6886_FIFO_COUNT  0x72
#define MPU6886_FIFO_R_W    0x74

//#define G (9.8)
#define RtA     57.324841
#define AtR     0.0174533
#define Gyro_Gr 0.0010653

class MPU6886s {
   public:
    enum Ascale { AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };

    enum Gscale { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };

    Gscale Gyscale = GFS_2000DPS;
    Ascale Acscale = AFS_8G;

   public:
    MPU6886s();
    int Init(void);
    void calibrateAccel(int _bufsize, int _acl_deadzone);
    void enableWakeOnMotion(Ascale ascale, uint8_t thresh_num_lsb);
    void getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az);
    void getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz);
    void getTempAdc(int16_t* t);

    void getAccelData(float* ax, float* ay, float* az);
    void getGyroData(float* gx, float* gy, float* gz);
    void getTempData(float* t);

    void SetGyroFsr(Gscale scale);
    void SetAccelFsr(Ascale scale);

    int16_t getXAccelOffset();
    void setXAccelOffset(int16_t offset);
    int16_t getYAccelOffset();
    void setYAccelOffset(int16_t offset);
    int16_t getZAccelOffset();
    void setZAccelOffset(int16_t offset);
    void setXGyroOffset(int16_t offset);
    void setYGyroOffset(int16_t offset);
    void setZGyroOffset(int16_t offset);

    void SetINTPinActiveLogic(uint8_t level);
    void DisableAllIRQ();
    void ClearAllIRQ();

    void setFIFOEnable(bool enableflag);
    uint8_t ReadFIFO();
    void ReadFIFOBuff(uint8_t* DataBuff, uint16_t Length);
    uint16_t ReadFIFOCount();
    void RestFIFO();
   public:
    float aRes, gRes;

   private:
    int bufsize;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
    int acl_deadzone;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)

    int16_t ax, ay, az;
    int16_t mean_ax,mean_ay,mean_az,state;
    int16_t ax_offset,ay_offset,az_offset;

   private:
    void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr,
                         uint8_t number_Bytes, uint8_t* read_Buffer);
    void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr,
                          uint8_t number_Bytes, uint8_t* write_Buffer);

    void meansensors();
    void calibration();

    void getGres();
    void getAres();
};
#endif