#include "MPU6886s.h"
#ifdef STICKC
  #include <M5StickC.h>
#else
  #include <M5StickCPlus.h>
#endif
#include <math.h>
#include "util.h"

MPU6886s::MPU6886s() {
}

void MPU6886s::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr,
                              uint8_t number_Bytes, uint8_t* read_Buffer) {
    Wire1.beginTransmission(driver_Addr);
    Wire1.write(start_Addr);
    Wire1.endTransmission(false);
    uint8_t i = 0;
    Wire1.requestFrom(driver_Addr, number_Bytes);

    //! Put read results in the Rx buffer
    while (Wire1.available()) {
        read_Buffer[i++] = Wire1.read();
    }
}

void MPU6886s::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr,
                               uint8_t number_Bytes, uint8_t* write_Buffer) {
    Wire1.beginTransmission(driver_Addr);
    Wire1.write(start_Addr);
    Wire1.write(*write_Buffer);
    Wire1.endTransmission();
}

int MPU6886s::Init(void) {
    unsigned char tempdata[1];
    unsigned char regdata;

    Wire1.begin(21, 22);

// 0x75, REGISTER 117 – WHOMI: Check if correct device
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_WHOAMI, 1, tempdata);
    if (tempdata[0] != 0x19) return -1;
    delay(1);

// 0x6B, REGISTER 107 – POWER MANAGEMENT 1:
    regdata = 0b00000000; // 0x00
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

// 0x6B, REGISTER 107 – POWER MANAGEMENT 1: Reset internal registers
    regdata = 0b10000000; // (0x01 << 7)
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

// 0x6B, REGISTER 107 – POWER MANAGEMENT 1: The default value of CLKSEL[2:0] is 001
    regdata = 0b00000001; // (0x01 << 0)
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    delay(10);

// 0x1C, REGISTER 28 – ACCELEROMETER CONFIGURATION: 0b00010000 : Select +-2g for accel
    regdata = 0b00000000; // 0x00
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(1);

// 0x1B,REGISTER 27 – GYROSCOPE CONFIGURATION: 0b00011000 : Select +-250dps for gyro
    regdata = 0b00000000; // 0x00
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(1);

// 0x1A,REGISTER 26 – CONFIGURATION: DLPF_CFG - 1kHz
    regdata = 0b00000110; // 0x01
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_CONFIG, 1, &regdata);
    delay(1);

// 0X19, REGISTER 25 – SAMPLE RATE DIVIDER: Sample Rate Divider - SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) Where INTERNAL_SAMPLE_RATE = 1 kHz
//    regdata = 0b01100011; // 99 -> 10Hz 
//    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1, &regdata);
//    delay(1);

// 0x38, REGISTER 56 – INTERRUPT ENABLE
    regdata = 0b00000000; // 0x00
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);
    delay(1);

// Disable Gyrometer
regdata = 0b00000111; // set gyro x, y, and z to disable
I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_2, 1, &regdata);

/* 0x1D, REGISTER 29 – ACCELEROMETER CONFIGURATION 2
 * ACCEL_FCHOICE_B | A_DLPF_CFG | 3-DB BW(HZ)  | NOISE BW(HZ) | RATE(KHZ)
 * ----------------+------------+--------------+--------------+-----------
 * 1               | X          | 1046.0       | 1100.0       | 4
 * 0               | 0          | 218.1        | 235.0        | 1
 * 0               | 1          | 218.1        | 235.0        | 1
 * 0               | 2          | 99.0         | 121.3        | 1
 * 0               | 3          | 44.8         | 61.5         | 1
 * 0               | 4          | 21.2         | 31.0         | 1
 * 0               | 5          | 10.2         | 15.5         | 1
 * 0               | 6          | 5.1          | 7.8          | 1
 * 0               | 7          | 420.0        | 441.6        | 1
 */

    /* Step 2: Set Accelerometer LPF bandwidth to 218.1 Hz
        • In ACCEL_CONFIG2 register (0x1D) set ACCEL_FCHOICE_B = 0 and
       A_DLPF_CFG[2:0] = 1 (b001)
    */
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG2, 1, &regdata);
    regdata = 0b00000110;  // average 4 samples, use 5 Hz DLPF
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG2, 1, &regdata);
    delay(1);

// 0x6A, REGISTER 106 – USER CONTROL:
    regdata = 0b00000000; // 0x00
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
    delay(1);

// 0x23, REGISTER 35 – FIFO ENABLE:
    regdata = 0b00000000; // 0x00
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_EN, 1, &regdata);
    delay(1);

// 0x37, REGISTER 55 – INT/DRDY PIN / BYPASS ENABLE CONFIGURATION:
//    regdata = 0x22;
//    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &regdata);
//    delay(1);

// 0x38, 8.26 REGISTER 56 – INTERRUPT ENABLE:
//    regdata = 0x01;
//    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);

    delay(100);
    getGres();
    getAres();
  
    setXAccelOffset(0);
    setYAccelOffset(0);
    setZAccelOffset(0);

    state = 0;
    return 0;
}

void MPU6886s::calibrateAccel(int _bufsize, int _acl_deadzone) {
    bufsize = _bufsize;
    acl_deadzone = _acl_deadzone;
    unsigned char regdata;

// 0X19, REGISTER 25 – SAMPLE RATE DIVIDER: Sample Rate Divider - SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) Where INTERNAL_SAMPLE_RATE = 1 kHz
    regdata = 0b00000000; // 0 -> 1kHz 
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1, &regdata);
    delay(1);

    if (state==0){
        logln("\nReading sensors for first time...");
        meansensors();
        log("\nFirst Read Mean Values:\t");
        log(String(mean_ax)); 
        log("\t");
        log(String(mean_ay)); 
        log("\t");
        logln(String(mean_az)); 
        state++;
        delay(1000);
    }
 
    if (state==1) {
        logln("\nCalculating offsets...");
        calibration();
        state++;
        delay(1000);
    }
 
    if (state==2) {
        meansensors();
        logln("\nFINISHED!");
        log("\nSensor mean readings with offsets:\t");
        log(String(mean_ax)); 
        log("\t");
        log(String(mean_ay)); 
        log("\t");
        logln(String(mean_az)); 
        log("Your calculated offsets:\t");
        log(String(ax_offset)); 
        log("\t");
        log(String(ay_offset)); 
        log("\t");
        logln(String(az_offset)); 
        logln("\nData is printed as: acelX acelY acelZ");
        logln("Check that your sensor readings are close to 16384 0 0");
        state++;
    }
// 0X19, REGISTER 25 – SAMPLE RATE DIVIDER: Sample Rate Divider - SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) Where INTERNAL_SAMPLE_RATE = 1 kHz
    regdata = 0b01100011; // 99 -> 10Hz 
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1, &regdata);
    delay(1);
}

void MPU6886s::meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0;

  while (i<(bufsize+101)){
    // read raw accel/gyro measurements from device
    getAccelAdc(&ax, &ay, &az);
/*
    log(String(ax)); 
    log("\t");
    log(String(ay)); 
    log("\t");
    log(String(az));
    log("\t");
    log(String(ax_offset)); 
    log("\t");
    log(String(ay_offset)); 
    log("\t");
    logln(String(az_offset));
*/
    if (i>100 && i<=(bufsize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
    }
    if (i==(bufsize+100)){
      mean_ax=buff_ax/bufsize;
      mean_ay=buff_ay/bufsize;
      mean_az=buff_az/bufsize;
    }
    i++;
    delay(1); //Needed so we don't get repeated measures
  }
}

void MPU6886s::calibration(){
  ax_offset=(16384-mean_ax)/8;
  ay_offset=-mean_ay/8;
  az_offset=-mean_az/8;
  while (1){
    int ready=0;
    setXAccelOffset(ax_offset);
    setYAccelOffset(ay_offset);
    setZAccelOffset(az_offset);
    meansensors();
    log("Calibrating:\t");
    log(String(mean_ax)); 
    log("\t");
    log(String(mean_ay)); 
    log("\t");
    logln(String(mean_az));

    if (abs(16384-mean_ax)<=acl_deadzone) ready++;
    else ax_offset=ax_offset+(16384-mean_ax)/acl_deadzone;
 
    if (abs(mean_ay)<=acl_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acl_deadzone;
 
    if (abs(mean_az)<=acl_deadzone) ready++;
    else az_offset=az_offset-mean_az/acl_deadzone;
 
    if (ready==3) break;
  }
}

void MPU6886s::enableWakeOnMotion(Ascale ascale, uint8_t thresh_num_lsb) {
    uint8_t regdata;
    /* 5.1 WAKE-ON-MOTION INTERRUPT
        The MPU-6886 provides motion detection capability. A qualifying motion
       sample is one where the high passed sample from any axis has an absolute
       value exceeding a user-programmable threshold. The following steps
       explain how to configure the Wake-on-Motion Interrupt.
    */

    /* Step 0: this isn't explicitly listed in the steps, but configuring the
       FSR or full-scale-range of the accelerometer is important to setting up
       the accel/motion threshold in Step 4
    */
    regdata = (ascale << 3);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(10);

    /* Step 1: Ensure that Accelerometer is running
        • In PWR_MGMT_1 register (0x6B) set CYCLE = 0, SLEEP = 0, and
       GYRO_STANDBY = 0 • In PWR_MGMT_2 register (0x6C) set STBY_XA = STBY_YA =
       STBY_ZA = 0, and STBY_XG = STBY_YG = STBY_ZG = 1
    */
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    regdata =
        regdata & 0b10001111;  // set cyle, sleep, and gyro to standby, i.e. 0
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);

    regdata = 0b00000111;  // set accel x, y, and z to standby
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_2, 1, &regdata);

    /* Step 2: Set Accelerometer LPF bandwidth to 218.1 Hz
        • In ACCEL_CONFIG2 register (0x1D) set ACCEL_FCHOICE_B = 0 and
       A_DLPF_CFG[2:0] = 1 (b001)
    */
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG2, 1, &regdata);
    regdata = 0b00100001;  // average 32 samples, use 218 Hz DLPF
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG2, 1, &regdata);

    /* Step 2.5 - active low? */
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &regdata);
    regdata = ((regdata | 0b10000000) &
               0b11011111);  // configure pin active-low, no latch
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &regdata);

    /* Step 3: Enable Motion Interrupt
        • In INT_ENABLE register (0x38) set WOM_INT_EN = 111 to enable motion
       interrupt
    */
    regdata =
        0b11100000;  // enable wake-on-motion interrupt for X, Y, and Z axes
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &regdata);

    /* Step 4: Set Motion Threshold
        • Set the motion threshold in ACCEL_WOM_THR register (0x1F)
        NOTE: the data sheet mentions 0x1F, but is probably referring to
              registers 0x20, 0x21, and 0x22 based on empirical tests
    */
    regdata =
        thresh_num_lsb;  // set accel motion threshold for X, Y, and Z axes
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_WOM_X_THR, 1, &regdata);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_WOM_Y_THR, 1, &regdata);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_WOM_Z_THR, 1, &regdata);

    /* Step 5: Enable Accelerometer Hardware Intelligence
        • In ACCEL_INTEL_CTRL register (0x69) set ACCEL_INTEL_EN =
       ACCEL_INTEL_MODE = 1; Ensure that bit 0 is set to 0
    */
    regdata = 0b11000010;  // enable wake-on-motion if any of X, Y, or Z axes is
                           // above threshold
    // WOM_STEP5_ACCEL_INTEL_CTRL_INTEL_EN_1_MODE_1_WOM_TH_MODE_0;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_INTEL_CTRL, 1, &regdata);

    /* Step 7: Set Frequency of Wake-Up
        • In SMPLRT_DIV register (0x19) set SMPLRT_DIV[7:0] = 3.9 Hz – 500 Hz
    */
    // sample_rate = 1e3 / (1 + regdata)
    //   4.0 Hz = 1e3 / (1 + 249)
    //  10.0 Hz = 1e3 / (1 +  99)
    //  20.0 Hz = 1e3 / (1 +  49)
    //  25.0 Hz = 1e3 / (1 +  39)
    //  50.0 Hz = 1e3 / (1 +  19) <----
    // 500.0 Hz = 1e3 / (1 +   1)
    regdata = 19;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_SMPLRT_DIV, 1, &regdata);

    /* Step 8: Enable Cycle Mode (Accelerometer Low-Power Mode)
        • In PWR_MGMT_1 register (0x6B) set CYCLE = 1
    */
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
    regdata = regdata | 0b00100000;  // enable accelerometer low-power mode
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &regdata);
}

void MPU6886s::getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buf[6];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_XOUT_H, 6, buf);

    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}
void MPU6886s::getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buf[6];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_XOUT_H, 6, buf);

    *gx = ((uint16_t)buf[0] << 8) | buf[1];
    *gy = ((uint16_t)buf[2] << 8) | buf[3];
    *gz = ((uint16_t)buf[4] << 8) | buf[5];
}

void MPU6886s::getTempAdc(int16_t* t) {
    uint8_t buf[14];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_TEMP_OUT_H, 14, buf);

    *t = ((uint16_t)buf[6] << 8) | buf[7];
}


void MPU6886s::getGres() {
    switch (Gyscale) {
            // Possible gyro scales (and their register bit settings) are:
        case GFS_250DPS:
            gRes = 250.0 / 32768.0;
            break;
        case GFS_500DPS:
            gRes = 500.0 / 32768.0;
            break;
        case GFS_1000DPS:
            gRes = 1000.0 / 32768.0;
            break;
        case GFS_2000DPS:
            gRes = 2000.0 / 32768.0;
            break;
    }
}

void MPU6886s::getAres() {
    switch (Acscale) {
            // Possible accelerometer scales (and their register bit settings)
            // are: 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). Here's a
            // bit of an algorith to calculate DPS/(ADC tick) based on that
            // 2-bit value:
        case AFS_2G:
            aRes = 2.0 / 32768.0;
            break;
        case AFS_4G:
            aRes = 4.0 / 32768.0;
            break;
        case AFS_8G:
            aRes = 8.0 / 32768.0;
            break;
        case AFS_16G:
            aRes = 16.0 / 32768.0;
            break;
    }
}

void MPU6886s::SetGyroFsr(Gscale scale) {
    // return IIC_Write_Byte(MPU_GYRO_CFG_REG,scale<<3);//设置陀螺仪满量程范围
    unsigned char regdata;
    regdata = (scale << 3);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(10);

    Gyscale = scale;
    getGres();
}

void MPU6886s::SetAccelFsr(Ascale scale) {
    unsigned char regdata;
    regdata = (scale << 3);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(10);

    Acscale = scale;
    getAres();
}

int16_t MPU6886s::getXAccelOffset() {
    uint8_t buf[2];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_XA_OFFSET_H, 2, buf);
	return (((int16_t)buf[0]) << 8) | buf[1];
}

void MPU6886s::setXAccelOffset(int16_t offset) {
    uint8_t rawData[2];
    uint8_t tempdata;
    uint8_t mask_bit = 1;

    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_XA_OFFSET_L, 1, &tempdata);
    if (tempdata % 2) {
        mask_bit = 0;
    }

    if (mask_bit) {
        offset = offset & ~mask_bit;  // Preserve temperature compensation bit
    } else {
        offset = offset | 0x0001;  // Preserve temperature compensation bit
    }
    rawData[0] = (offset >> 8) & 0xFF;
    rawData[1] = (offset) & 0xFF;

    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_XA_OFFSET_H, 1,  &rawData[0]);
    delay(1);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_XA_OFFSET_L, 1, &rawData[1]);
    delay(1);
}

int16_t MPU6886s::getYAccelOffset() {
    uint8_t buf[2];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_YA_OFFSET_H, 2, buf);
	return (((int16_t)buf[0]) << 8) | buf[1];
}

void MPU6886s::setYAccelOffset(int16_t offset) {
    uint8_t rawData[2];
    uint8_t tempdata;
    uint8_t mask_bit = 1;

    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_YA_OFFSET_L, 1, &tempdata);
    if (tempdata % 2) {
        mask_bit = 0;
    }

    if (mask_bit) {
        offset = offset & ~mask_bit;  // Preserve temperature compensation bit
    } else {
        offset = offset | 0x0001;  // Preserve temperature compensation bit
    }

    rawData[0] = (offset >> 8) & 0xFF;
    rawData[1] = (offset) & 0xFF;

    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_YA_OFFSET_H, 1, &rawData[0]);
    delay(1);    
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_YA_OFFSET_L, 1, &rawData[1]);
    delay(1);    
}

int16_t MPU6886s::getZAccelOffset() {
    uint8_t buf[2];
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_ZA_OFFSET_H, 2, buf);
	return (((int16_t)buf[0]) << 8) | buf[1];
}

void MPU6886s::setZAccelOffset(int16_t offset) {
    uint8_t rawData[2];
    uint8_t tempdata;
    uint8_t mask_bit = 1;

    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_ZA_OFFSET_L, 1, &tempdata);
    if (tempdata % 2) {
        mask_bit = 0;
    }

    if (mask_bit) {
        offset = offset & ~mask_bit;  // Preserve temperature compensation bit
    } else {
        offset = offset | 0x0001;  // Preserve temperature compensation bit
    }

    rawData[0] = (offset >> 8) & 0xFF;
    rawData[1] = (offset) & 0xFF;

    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_ZA_OFFSET_H, 1, &rawData[0]);
    delay(1);  
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_ACCEL_ZA_OFFSET_L, 1, &rawData[1]);
    delay(1);    
}

void MPU6886s::setXGyroOffset(int16_t offset) {
    unsigned char regdata;
    regdata = offset;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_X_OFFS_USRH, 2, &regdata);
    delay(1);    
}

void MPU6886s::setYGyroOffset(int16_t offset) {
    unsigned char regdata;
    regdata = offset;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_Y_OFFS_USRH, 2, &regdata);
    delay(1);    
}

void MPU6886s::setZGyroOffset(int16_t offset) {
    unsigned char regdata;
    regdata = offset;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_GYRO_Z_OFFS_USRH, 2, &regdata);
    delay(1);    
}

void MPU6886s::getAccelData(float* ax, float* ay, float* az) {
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    getAccelAdc(&accX, &accY, &accZ);

    *ax = (float)accX * aRes;
    *ay = (float)accY * aRes;
    *az = (float)accZ * aRes;
}

void MPU6886s::getGyroData(float* gx, float* gy, float* gz) {
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    getGyroAdc(&gyroX, &gyroY, &gyroZ);

    *gx = (float)gyroX * gRes;
    *gy = (float)gyroY * gRes;
    *gz = (float)gyroZ * gRes;
}

void MPU6886s::getTempData(float* t) {
    int16_t temp = 0;
    getTempAdc(&temp);

    *t = (float)temp / 326.8 + 25.0;
}

void MPU6886s::SetINTPinActiveLogic(uint8_t level) {
    uint8_t tempdata;
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &tempdata);
    tempdata &= 0x7f;
    tempdata |= level ? 0x00 : (0x01 << 7);
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &tempdata);
}

void MPU6886s::DisableAllIRQ() {
    uint8_t tempdata = 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_ENABLE, 1, &tempdata);
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &tempdata);
    tempdata |= 0x01 << 6;
    // int pin is configured as open drain
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_INT_PIN_CFG, 1, &tempdata);
}

void MPU6886s::ClearAllIRQ() {
    uint8_t tempdata = 0x00;
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_WM_INT_STATUS, 1, &tempdata);
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_INT_STATUS, 1, &tempdata);
}

void MPU6886s::setFIFOEnable(bool enableflag) {
    uint8_t regdata = 0;
    regdata         = enableflag ? 0x08 : 0x00; // 0x18 for both gyro and accel
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_EN, 1, &regdata);
    regdata = enableflag ? 0x40 : 0x00;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &regdata);
}

uint8_t MPU6886s::ReadFIFO() {
    uint8_t ReData = 0;
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_R_W, 1, &ReData);
    return ReData;
}

void MPU6886s::ReadFIFOBuff(uint8_t* DataBuff, uint16_t Length) {
    uint8_t number = Length / 210;
    for (uint8_t i = 0; i < number; i++) {
        I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_R_W, 210,
                        &DataBuff[i * 210]);
    }

    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_R_W, Length % 210,
                    &DataBuff[number * 210]);
}

void MPU6886s::RestFIFO() {
    uint8_t buf_out;
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &buf_out);
    buf_out |= 0x04;
    I2C_Write_NBytes(MPU6886_ADDRESS, MPU6886_USER_CTRL, 1, &buf_out);
}

uint16_t MPU6886s::ReadFIFOCount() {
    uint8_t Buff[2];
    uint16_t ReData = 0;
    I2C_Read_NBytes(MPU6886_ADDRESS, MPU6886_FIFO_COUNT, 2, Buff);
    ReData = (Buff[0] << 8) | Buff[1];
    return ReData;
}
