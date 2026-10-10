//  Written by Mark Youngsoo Ahn (ysahn2000@hotmail.com)
//  2018.12.31


//---------- I2C setting ------------------------------------------------------------------------
#include <Wire.h> 

//////////// Quaternion //////////////////////////////////////////////////////////////////////////
//---------- Registers of MPU6050 ----------------------------------------------------------------
/*
References for Quaternion
    . MahonyAHRS Filter Algorithm
          http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
    . Jeff Rowberg's MPU6050
          https://github.com/jrowberg/i2cdevlib
    . Kris Winer's MPU6050
          https://github.com/kriswiner/MPU6050
    . Davide Gironi's AVR atmega MPU6050
          http://davidegironi.blogspot.com/2013/02/avr-atmega-mpu6050-gyroscope-and.html#.W7zM7mgzaUk
*/
// Registers refer to Jeff Rowberg's MPU6050.h
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW


#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_SELF_TEST_XA_1_BIT     0x07
#define MPU6050_SELF_TEST_XA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_XA_2_BIT     0x05
#define MPU6050_SELF_TEST_XA_2_LENGTH  0x02
#define MPU6050_SELF_TEST_YA_1_BIT     0x07
#define MPU6050_SELF_TEST_YA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_YA_2_BIT     0x03
#define MPU6050_SELF_TEST_YA_2_LENGTH  0x02
#define MPU6050_SELF_TEST_ZA_1_BIT     0x07
#define MPU6050_SELF_TEST_ZA_1_LENGTH  0x03
#define MPU6050_SELF_TEST_ZA_2_BIT     0x01
#define MPU6050_SELF_TEST_ZA_2_LENGTH  0x02

#define MPU6050_SELF_TEST_XG_1_BIT     0x04
#define MPU6050_SELF_TEST_XG_1_LENGTH  0x05
#define MPU6050_SELF_TEST_YG_1_BIT     0x04
#define MPU6050_SELF_TEST_YG_1_LENGTH  0x05
#define MPU6050_SELF_TEST_ZG_1_BIT     0x04
#define MPU6050_SELF_TEST_ZG_1_LENGTH  0x05

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

//---------- Offset Value of MPU6050 -------------------------------------------------------------
#define MPU6050_AXOFFSET 150 // Input Accelerometer Offset valuer After IMU Calibration
#define MPU6050_AYOFFSET 39 // Input Accelerometer Offset valuer After IMU Calibration
#define MPU6050_AZOFFSET 3 // Input Accelerometer Offset valuer After IMU Calibration
#define MPU6050_GXOFFSET 27 // Input Gyroscope Offset valuer After IMU Calibration
#define MPU6050_GYOFFSET -47 // Input Gyroscope Offset valuer After IMU Calibration
#define MPU6050_GZOFFSET -17 // Input Gyroscope Offset valuer After IMU Calibration

//---------- Setting Value of Acceleromter of MPU6050 --------------------------------------------
//#define MPU6050_AXGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AYGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AZGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
//#define MPU6050_AXGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
//#define MPU6050_AYGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
//#define MPU6050_AZGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
#define MPU6050_AXGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AYGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AZGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
//#define MPU6050_AXGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
//#define MPU6050_AYGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
//#define MPU6050_AZGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16

//---------- Setting Value of Gyroscope of MPU6050 -----------------------------------------------
//#define MPU6050_GXGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GYGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GZGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
//#define MPU6050_GXGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GYGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GZGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
//#define MPU6050_GXGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
//#define MPU6050_GYGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
//#define MPU6050_GZGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
#define MPU6050_GXGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GYGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GZGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000

//---------- Variables of Quaternion -------------------------------------------------------------
#include <math.h>

#define Yaw 0
#define Pitch 1
#define Roll 2
#define Throttle 3

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

volatile float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

unsigned long sampling_timer;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float axg, ayg, azg, gxrs, gyrs, gzrs;
float Angle[3], Prev_Angle[3];

float sampleFreq = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t now_quaternion = 0;                                 // used to calculate integration interval

////////// PCI (Pin Change Interrupt) ////////////////////////////////////////////////////////////
//---------- Pin Change Interrupt for Transmitter ------------------------------------------------
#define Channel1 0
#define Channel2 1
#define Channel3 2
#define Channel4 3
unsigned long Current_Time, PWM_Timer[4];
byte Trigger[4];
int PWM_Width[4];
float TargetValue[4], Prev_TargetValue[4];

////////// PID Control ///////////////////////////////////////////////////////////////////////////
//---------- PID MIX & PWM Output ----------------------------------------------------------------
int PWM_Port[4] = {6, 9, 10, 11}; // PWM Port for Servo
float Error[3]; // Error = Target Value from RC Receiver - Measured Angle from IMU
float Prev_Error[3]; // Previous Error
float dError[3]; // dError = Error - Previous_Error
float Input[3]; // Measured Angle from IMU for derivative component
float Prev_Input[3]; // Last Measured Angle from IMU for derivative component
float dInput[3]; // dInput =  Input - Prev_Input -> dInput = Measured Angle from IMU - Last Measured Angle from IMU
unsigned long now, Prev_time; // Current & Previous time
float dtime; // dt = micros() - Prev_time
float Kp[3] = {4.6, 3.0, 3.0}; // P coefficients in that order, Yaw, Pitch, Roll
float Ki[3] = {0.3, 0.5, 0.5 }; // I coefficients in that order, Yaw, Pitch, Roll
float Kd[3] = {0, 0.3, 0.3}; // D coefficients in that order, Yaw, Pitch, Roll
float PTerm[3]; // Kp * Error, Yaw, Pitch, Roll
float ITerm[3]; // ITerm += Ki * Error * dtime, Yaw, Pitch, Roll
float DTerm[3]; // Kd * (dError / dtime), Yaw, Pitch, Roll
float Angle_PID[3]; // Angle for PID, Yaw, Pitch, Roll
unsigned int OutputPWM[4]; // OutputPWM = PIDMIX, Front_Left, Front_Right, Rear_Right, Rear_Left
float A, B, C;
#define PIDMIX(A,B,C) TargetValue[Throttle] + Angle_PID[Yaw]*A + Angle_PID[Pitch]*B + Angle_PID[Roll]*C
int Val_PWM[4]; // Needed PWM Value

//---------- Servo setting ----------------------------------------------------------------------
#include <Servo.h>//Using servo library to control ESC
Servo Esc[4]; //Creating a servo class with name as esc[]

////////// Arming ////////////////////////////////////////////////////////////////////////////////
//---------- Arming ------------------------------------------------------------------------------
int Start = 0; // Arming status, 0 = Stopped, 1 = Arming, 2 = Armed
int ArmStatus;

//---------- LED Blink ---------------------------------------------------------------------------
#define LED_PIN 7
boolean blinkState = false;




////////// Main Program (SETUP) //////////////////////////////////////////////////////////////////
void setup() {

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200); // Baud Rate

  //---------- Initialize PWM of servo output -------------------------------
  Init_PWM_Port();

  //---------- Initialize PCI -----------------------------------------------
  Init_PCI();
  
  //---------- Initialize MPU6050 -------------------------------------------
  Init_MPU6050(); // Initialize MPU6050

  //---------- Initialize LED -----------------------------------------------
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //---------- Definition Timer for PCI -------------------------------------
  now = micros();
  Prev_time = now;
  
  //---------- Sampling Timer for calculation of Quaternion -----------------
  sampling_timer = micros(); 

}

////////// Main Program (LOOP) ///////////////////////////////////////////////////////////////////
void loop() {
  
  //---------- Calculation IMU for Quaternion -------------------------------
  mpu6050_GetData(); // Get raw data
  mpu6050_updateQuaternion(); // Update raw data to Quaternion form

  now_quaternion = micros(); // Renew now_quaternion
  sampleFreq = (1000000.0f / (now_quaternion - lastUpdate)); // set integration time by time elapsed since last filter update
  lastUpdate = now_quaternion;
  
  MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg); // Compute data  
  mpu6050_getYawPitchRoll(); // Values of Yaw, Pitch, Roll

  //---------- Read Target Value from RC Receiver ---------------------------
  Target_Value();
  
  //---------- Calculation PID ----------------------------------------------
  PID_Control(); 

  //---------- Check Arming Status ------------------------------------------
  ArmStatus = Arming_Status();
  
  //---------- Motor Control ------------------------------------------------
  if(ArmStatus == 2) PWM_Control(); 
  if(ArmStatus == 0) {
    for(int i = 0 ; i < 4 ; i++) {
      Esc[i].writeMicroseconds(1000); //initialize the signal to 1000
    }
  }
  
  //---------- Serial Printing ----------------------------------------------
//  Serial.print("PWM_Width[Channel1] = "); Serial.print(PWM_Width[Channel1]);
//  Serial.print(" | PWM_Width[Channel2] = "); Serial.print(PWM_Width[Channel2]);
//  Serial.print(" | PWM_Width[Channel3] = "); Serial.print(PWM_Width[Channel3]);
//  Serial.print(" | PWM_Width[Channel4] = "); Serial.print(PWM_Width[Channel4]);
//  Serial.print(" | ArmStatus = "); Serial.println(ArmStatus);

//  Serial.print("Yaw = "); Serial.print(Angle[Yaw]);
//  Serial.print(" | Pitch = "); Serial.print(Angle[Pitch]);
//  Serial.print(" | Roll = "); Serial.println(Angle[Roll]); 

//  Serial.print("TargetValue[Yaw] = "); Serial.print(TargetValue[Yaw]);
//  Serial.print(" | TargetValue[Pitch] = "); Serial.print(TargetValue[Pitch]);
//  Serial.print(" | TargetValue[Roll] = "); Serial.print(TargetValue[Roll]);
//  Serial.print(" | TargetValue[Throttle] = "); Serial.println(TargetValue[Throttle]);

//  Serial.print("Error[Yaw] = "); Serial.print(Error[Yaw]);
//  Serial.print(" | Error[Pitch] = "); Serial.print(Error[Pitch]);
//  Serial.print(" | Error[Roll] = "); Serial.print(Error[Roll]);
//  Serial.print(" | dError[Yaw] = "); Serial.print(dError[Yaw]);
//  Serial.print(" | dError[Pitch] = "); Serial.print(dError[Pitch]);
//  Serial.print(" | dError[Roll] = "); Serial.println(dError[Roll]);
//  Serial.print(" | dInput[Yaw] = "); Serial.print(dInput[Yaw]);
//  Serial.print(" | dInput[Pitch] = "); Serial.print(dInput[Pitch]);
//  Serial.print(" | dInput[Roll] = "); Serial.println(dInput[Roll]);

//  Serial.print("PTerm[Yaw] = "); Serial.print(PTerm[Yaw]);
//  Serial.print(" | ITerm[Yaw] = "); Serial.print(ITerm[Yaw]);
//  Serial.print(" | DTerm[Yaw] = "); Serial.print(DTerm[Yaw]);
//  Serial.print(" | PTerm[Pitch] = "); Serial.print(PTerm[Pitch]);
//  Serial.print(" | ITerm[Pitch] = "); Serial.print(ITerm[Pitch]);
//  Serial.print(" | DTerm[Pitch] = "); Serial.println(DTerm[Pitch]);
//  Serial.print(" | PTerm[Roll] = "); Serial.print(PTerm[Roll]);
//  Serial.print(" | ITerm[Roll] = "); Serial.print(ITerm[Roll]);
//  Serial.print(" | DTerm[Roll] = "); Serial.println(DTerm[Roll]);
  
//  Serial.print("Angle_PID[Yaw] = "); Serial.print(Angle_PID[Yaw]);
//  Serial.print(" | Angle_PID[Pitch] = "); Serial.print(Angle_PID[Pitch]);
//  Serial.print(" | Angle_PID[Roll] = "); Serial.println(Angle_PID[Roll]);

//  Serial.print("FrontLeft = "); Serial.print(OutputPWM[0]);
//  Serial.print(" | FrontRight = "); Serial.print(OutputPWM[1]);
//  Serial.print(" | RearRight = "); Serial.print(OutputPWM[2]);
//  Serial.print(" | RearLeft = "); Serial.println(OutputPWM[3]);
  
//  Serial.print("FrontLeft = "); Serial.print(Val_PWM[0]);
//  Serial.print(" | FrontRight = "); Serial.print(Val_PWM[1]);
//  Serial.print(" | RearRight = "); Serial.print(Val_PWM[2]);
//  Serial.print(" | RearLeft = "); Serial.println(Val_PWM[3]);

  
  //---------- Sampling Timer for calculation of Quaternion -----------------
  while(micros() - sampling_timer < 4000); // Set the Loop Time
  sampling_timer = micros(); // Reset the sampling timer  
  
  //---------- Blink LED to indicate activity -------------------------------
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

}

/*  
 *  Added Functions
 */

////////// IMU MPU6050 ///////////////////////////////////////////////////////////////////////////
//---------- Functions of Quaternion -------------------------------------------------------------
void Init_MPU6050(){
  // MPU6050 Initializing & Reset
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00); // set to zero (wakes up the MPU-6050)

  // MPU6050 Clock Type
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01); // Selection Clock 'PLL with X axis gyroscope reference'

  // MPU6050 Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) for DMP
  //writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00); // Default is 1KHz // example 0x04 is 200Hz

  // MPU6050 Gyroscope Configuration Setting
  /* Wire.write(0x00); // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
     Wire.write(0x08); // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
     Wire.write(0x10); // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
     Wire.write(0x18); // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]   */
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18); // FS_SEL=3

  // MPU6050 Accelerometer Configuration Setting
  /* Wire.write(0x00); // AFS_SEL=0, Full Scale Range = +/- 2 [g]
     Wire.write(0x08); // AFS_SEL=1, Full Scale Range = +/- 4 [g]
     Wire.write(0x10); // AFS_SEL=2, Full Scale Range = +/- 8 [g]
     Wire.write(0x18); // AFS_SEL=3, Full Scale Range = +/- 10 [g] */
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x10); // AFS_SEL=2

  // MPU6050 DLPF(Digital Low Pass Filter)
  /*Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
    Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
    Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
    Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
    Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
    Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
    Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz */
  writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x00); //Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
}

void mpu6050_GetData() {
  uint8_t data_org[14]; // original data of accelerometer and gyro
  readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &data_org[0]);

  AcX = data_org[0] << 8 | data_org[1];  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY = data_org[2] << 8 | data_org[3];  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = data_org[4] << 8 | data_org[5];  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = data_org[6] << 8 | data_org[7];  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = data_org[8] << 8 | data_org[9];  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = data_org[10] << 8 | data_org[11];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = data_org[12] << 8 | data_org[13];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void mpu6050_updateQuaternion() {
  axg = (float)(AcX - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
  ayg = (float)(AcY - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
  azg = (float)(AcZ - MPU6050_AZOFFSET) / MPU6050_AZGAIN;
  gxrs = (float)(GyX - MPU6050_GXOFFSET) / MPU6050_GXGAIN * 0.01745329; //degree to radians
  gyrs = (float)(GyY - MPU6050_GYOFFSET) / MPU6050_GYGAIN * 0.01745329; //degree to radians
  gzrs = (float)(GyZ - MPU6050_GZOFFSET) / MPU6050_GZGAIN * 0.01745329; //degree to radians
  // Degree to Radians Pi / 180 = 0.01745329 0.01745329251994329576923690768489
  
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
 
  float norm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
  
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  
  // Normalise quaternion
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}


void mpu6050_getYawPitchRoll() {
//  Angle[Yaw] = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 57.29577951;
//  Angle[Roll] = -asin(2*q1*q3 + 2*q0*q2) * 57.29577951;
//  Angle[Pitch] = atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1) * 57.29577951;
//  Angle[Pitch] = atan2(2*q0*q1 + 2*q2*q3, 1 - 2*q1*q1 - 2*q2*q2) * 57.29577951;
//  Angle[Roll] = asin(2*q0*q2 - 2*q3*q1) * 57.29577951;
//  Angle[Yaw] = -atan2(2*q0*q3 + 2*q1*q2, 1 - 2*q2*q2 - 2*q3*q3) * 57.29577951;
  //Angle[Yaw] = -atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29577951;   
  Angle[Roll] = -asin(2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;
  Angle[Pitch] = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29577951;  

  Angle[Yaw] = - gzrs / 0.01745329; // Radian to Degree

  // Apply Complementary Filter to decrease noise
  for (int i = 0 ; i < 3 ; i++) {
    if(i==0) Angle[i] = 0.05 * Angle[i] + 0.95 * Prev_Angle[i]; // Yaw is 0, Pitch is 1, Roll is 2
    else Angle[i] = 1.0 * Angle[i] + 0.0 * Prev_Angle[i]; // Yaw is 0, Pitch is 1, Roll is 2
    Prev_Angle[i] = Angle[i];
  }
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

////////// PID Control ///////////////////////////////////////////////////////////////////////////
//---------- Functions of PID Control ------------------------------------------------------------
void PID_Control() {

  // Calculation Errors
  for(int i = 0 ; i < 3 ; i++) {
    Error[i] = TargetValue[i] - Angle[i]; // Error = Target Value from RC Receiver - Measured Angle from IMU
  }

  // P Term
  for(int i = 0 ; i < 3 ; i++) {
    PTerm[i] = Kp[i] * Error[i];
  }

  // dtime
  now = micros();
  dtime = (float) (now - Prev_time)/1000000.0;
  Prev_time = now; // Renew Previous Clock Time
  
  // I Term
  for(int i = 0 ; i < 3 ; i++) {
    ITerm[i] += Ki[i] * Error[i] * dtime;
    if (TargetValue[Throttle] < 1020) {
      ITerm[i] = 0;
    }
    ITerm[i] = constrain(ITerm[i],-500, 500);
  }

  // D Term & dError (= dInput)
  for(int i = 0 ; i < 3 ; i++) { // dError = Error - Prev_Error
    dError[i] = Error[i] - Prev_Error[i]; // Use this term instead of (dError = Error - Prev_Error)
    Prev_Error[i] = Error[i]; // Renew Previous Inputs
    DTerm[i] = Kd[i] * dError[i] / dtime;
  }
  
  // Total PID
  for(int i = 0 ; i < 3 ; i++) {
    Angle_PID[i] = PTerm[i] + ITerm[i] + DTerm[i];
  }

  // Motor Operation Condition
  if (TargetValue[Throttle] < 1020) {
    for(int i = 0 ; i < 3 ; i++) {
      Angle_PID[i] = 0;
    }
  }

}

////////// Arming ////////////////////////////////////////////////////////////////////////////////
//---------- Functions of Arming -----------------------------------------------------------------
int Arming_Status()
{  
  // When Status is Arming (Throttle is moved to Bottom & Yaw is moved to Right)
  if(Start == 0 && PWM_Width[Channel3] < 1020 && PWM_Width[Channel4] > 1980) Start = 1;

  // When Status is Armed (Throttle is on Bottom & Yaw is moved to Center)
  if(Start == 1 && PWM_Width[Channel3] < 1020 && PWM_Width[Channel4] < 1520) { 
    Start = 2;
    for (int i = 0 ; i < 3 ; i++) {
      Error[i] = 0;
      Prev_Error[i] = 0;
      dError[i] = 0;
      dInput[i] = 0;
      Prev_Input[i] = 0;
      ITerm[i] = 0;
    }
  }

  // When Status is Stopped (Throttle is moved to Bottom & Yaw is moved to Left)
  if(Start == 2 && PWM_Width[Channel3] < 1020 && PWM_Width[Channel4] < 1020) Start = 0;
  
  return Start;
}

////////// Output ////////////////////////////////////////////////////////////////////////////////
//---------- Functions of PWM Output -------------------------------------------------------------
void Init_PWM_Port() { // Initializing PWM Port
  for(int i = 0; i < 4; i++) {
    Esc[i].attach(PWM_Port[i]);
    Esc[i].writeMicroseconds(1000); //initialize the signal to 1000
  }  
}

void PWM_Control() { // Motor Speed Control
  // Output PWM
  OutputPWM[0] = PIDMIX(-1.0, 1.0, 1.0); // Front_Left, A = Yaw, B = Pitch, C = Roll
  OutputPWM[1] = PIDMIX(1.0, 1.0, -1.0); // Front_Right, A = Yaw, B = Pitch, C = Roll
  OutputPWM[2] = PIDMIX(-1.0, -1.0, -1.0); // Rear_Right, A = Yaw, B = Pitch, C = Roll
  OutputPWM[3] = PIDMIX(1.0, -1.0, 1.0); // Rear_Left, A = Yaw, B = Pitch, C = Roll

  // Data Range
  for(int i = 0 ; i < 4 ; i++) {
    OutputPWM[i] = constrain(OutputPWM[i], 1020, 2000);
    Esc[i].writeMicroseconds(OutputPWM[i]); // Output Esc value
  }

//  // PWM Output
//  for(int i = 0; i < 4; i++) {    
//    Val_PWM[i] = map(OutputPWM[i], 1020, 2000, 0, 255); //mapping val to minimum and maximum(Change if needed)
//    analogWrite(PWM_Port[i], Val_PWM[i]); // Output DC Motor PWM
//  }

}

////////// PCI ///////////////////////////////////////////////////////////////////////////////////
//---------- Functions of Pin Change Interrupt ---------------------------------------------------
void Init_PCI() {  
  // Pin to interrupt map:
  // D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
  // D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
  // A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
  // Reference is https://playground.arduino.cc/Main/PcInt

  PCICR  |= (1 << PCIE2);  // Set PCIE2 to enable PCMSK scan
  PCMSK2 |= (1 << PCINT18); // Set PCINT18 (digital input D2) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT19); // Set PCINT19 (digital input D3) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT20); // Set PCINT20 (digital input D4)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT21); // Set PCINT21 (digital input D5)to trigger an interrupt on state change
}

ISR(PCINT2_vect) {
  // ISR (Interrupt Service Routine) pinout
  // ISR (PCINT0_vect) pin change interrupt for D8 to D13
  // ISR (PCINT1_vect) pin change interrupt for A0 to A5
  // ISR (PCINT2_vect) pin change interrupt for D0 to D7

  // B (digital pin D8 to D13)
  // C (analog input pins A0 to A5)
  // D (digital pins D0 to D7)
  // References are 
  // https://www.arduino.cc/en/Reference/PortManipulation

  /*
  DDRD - The Port D Data Direction Register - read/write
  PORTD - The Port D Data Register - read/write
  PIND - The Port D Input Pins Register - read only
  *** Example ***
  DDRD = B11111110;  // sets Arduino pins 1 to 7 as outputs, pin 0 as input
  DDRD = DDRD | B11111100;  // this is safer as it sets pins 2 to 7 as outputs
                      // without changing the value of pins 0 & 1, which are RX & TX
  PORTD = B10101000; // sets digital pins 7,5,3 HIGH
  */
  
  Current_Time = micros(); // Definition of timer0
  
  /////////// channel 1 ///////////
  // Rising
  if(Trigger[Channel1] == 0 && PIND & B00000100 ) { // (Trigger[Channel1] is LOW) && (PCINT18(D2) is rising)
    Trigger[Channel1] = 1; // change trigger statement
    PWM_Timer[Channel1] = Current_Time; // Time of rising
  }
  // Falling
  else if(Trigger[Channel1] == 1 && !(PIND & B00000100) ) { // (Trigger[Channel1] is HIGH) && (PCINT18(D2) is falling)
    Trigger[Channel1] = 0; // change trigger statement
    PWM_Width[Channel1] = Current_Time - PWM_Timer[Channel1]; // Width of PWM of channel 1
    PWM_Width[Channel1] = constrain(PWM_Width[Channel1], 1000, 2000); // Limit data between 1000~2000
  }
  
  /////////// channel 2 ///////////
  // Rising
  if(Trigger[Channel2] == 0 && PIND & B00001000 ) { // (Trigger[Channel2] is LOW) && (PCINT19(D3) is rising)
    Trigger[Channel2] = 1; // change trigger statement
    PWM_Timer[Channel2] = Current_Time; // Time of rising
  }
  // Falling
  else if(Trigger[Channel2] == 1 && !(PIND & B00001000) ) { // (Trigger[Channel2] is HIGH) && (PCINT19(D3) is falling)
    Trigger[Channel2] = 0; // change trigger statement
    PWM_Width[Channel2] = Current_Time - PWM_Timer[Channel2]; // Width of PWM of channel 2
    PWM_Width[Channel2] = constrain(PWM_Width[Channel2], 1000, 2000); // Limit data between 1000~2000
  }
  
  /////////// channel 3 ///////////
  // Rising
  if(Trigger[Channel3] == 0 && PIND & B00010000 ) { // (Trigger[Channel3] is LOW) && (PCINT20(D4) is rising)
    Trigger[Channel3] = 1; // change trigger statement
    PWM_Timer[Channel3] = Current_Time; // Time of rising
  }
  // Falling
  else if(Trigger[Channel3] == 1 && !(PIND & B00010000) ) { // (Trigger[Channel3] is HIGH) && (PCINT20(D4) is falling)
    Trigger[Channel3] = 0; // change trigger statement
    PWM_Width[Channel3] = Current_Time - PWM_Timer[Channel3]; // Width of PWM of channel 3
    PWM_Width[Channel3] = constrain(PWM_Width[Channel3], 1000, 2000); // Limit data between 1000~2000
  }
  
  /////////// channel 4 ///////////
  // Rising
  if(Trigger[Channel4] == 0 && PIND & B00100000 ) { // (Trigger[Channel4] is LOW) && (PCINT21(D5) is rising)
    Trigger[Channel4] = 1; // change trigger statement
    PWM_Timer[Channel4] = Current_Time; // Time of rising
  }
  // Falling
  else if(Trigger[Channel4] == 1 && !(PIND & B00100000) ) { // (Trigger[Channel4] is HIGH) && (PCINT21(D5) is falling)
    Trigger[Channel4] = 0; // change trigger statement
    PWM_Width[Channel4] = Current_Time - PWM_Timer[Channel4]; // Width of PWM of channel 4
    PWM_Width[Channel4] = constrain(PWM_Width[Channel4], 1000, 2000); // Limit data between 1000~2000
  }
}

void Target_Value() {
  TargetValue[Yaw]      = mapfloat(PWM_Width[Channel4], 1000, 2000, -180, 180);
  TargetValue[Pitch]    = mapfloat(PWM_Width[Channel2], 1000, 2000, 45, -45);
  TargetValue[Roll]     = mapfloat(PWM_Width[Channel1], 1000, 2000, -45, 45);
  TargetValue[Throttle] = mapfloat(PWM_Width[Channel3], 1000, 2000, 1000, 2000);

  // Apply Complementary Filter to decrease noise
  for (int i = 0 ; i < 4 ; i++) {
    TargetValue[i] = 0.05 * TargetValue[i] + 0.95 * Prev_TargetValue[i]; // Yaw is 0, Pitch is 1, Roll is 2
    Prev_TargetValue[i] = TargetValue[i];
  }
}

float mapfloat(int x, int in_min, int in_max, int out_min, int out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
