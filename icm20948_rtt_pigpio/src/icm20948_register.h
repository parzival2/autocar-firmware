#ifndef ICM20948_REGISTER_H
#define ICM20948_REGISTER_H

#include <cstdint>

namespace ICM20948REG
{
///
/// USER BANK0 Register map.
///
static constexpr uint8_t I2C_ICM_ADDRESS		  = 0x68;
static constexpr uint8_t I2C_AK_ADDRESS			  = 0x0C;
static constexpr uint8_t I2C_ICM_WHO_AM_I		  = 0x00;
static constexpr uint8_t I2C_ICM_WHO_AM_I_ANSWER  = 0XEA;
static constexpr uint8_t I2C_ICM_USER_CTRL		  = 0x03;
static constexpr uint8_t I2C_ICM_LP_CONFIG		  = 0x05;
static constexpr uint8_t I2C_ICM_PWR_MGMT_1		  = 0x06; // Device defaults to the SLEEP mode
static constexpr uint8_t I2C_ICM_PWR_MGMT_2		  = 0x07;
static constexpr uint8_t I2C_ICM_INT_PIN_CFG	  = 0x0F;
static constexpr uint8_t I2C_ICM_INT_ENABLE		  = 0x10;
static constexpr uint8_t I2C_ICM_INT_ENABLE_1	 = 0x11; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_INT_ENABLE_2	 = 0x12; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_INT_ENABLE_3	 = 0x13; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_I2C_MST_STATUS   = 0x17;
static constexpr uint8_t I2C_ICM_INT_STATUS		  = 0x19;
static constexpr uint8_t I2C_ICM_INT_STATUS_1	 = 0x1A; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_INT_STATUS_2	 = 0x1B; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_INT_STATUS_3	 = 0x1C; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_DELAY_TIMEH	  = 0x28; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_DELAY_TIMEL	  = 0x29; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_ACCEL_XOUT_H	 = 0x2D;
static constexpr uint8_t I2C_ICM_ACCEL_XOUT_L	 = 0x2E;
static constexpr uint8_t I2C_ICM_ACCEL_YOUT_H	 = 0x2F;
static constexpr uint8_t I2C_ICM_ACCEL_YOUT_L	 = 0x30;
static constexpr uint8_t I2C_ICM_ACCEL_ZOUT_H	 = 0x31;
static constexpr uint8_t I2C_ICM_ACCEL_ZOUT_L	 = 0x32;
static constexpr uint8_t I2C_ICM_GYRO_XOUT_H	  = 0x33;
static constexpr uint8_t I2C_ICM_GYRO_XOUT_L	  = 0x34;
static constexpr uint8_t I2C_ICM_GYRO_YOUT_H	  = 0x35;
static constexpr uint8_t I2C_ICM_GYRO_YOUT_L	  = 0x36;
static constexpr uint8_t I2C_ICM_GYRO_ZOUT_H	  = 0x37;
static constexpr uint8_t I2C_ICM_GYRO_ZOUT_L	  = 0x38;
static constexpr uint8_t I2C_ICM_TEMP_OUT_H		  = 0x39;
static constexpr uint8_t I2C_ICM_TEMP_OUT_L		  = 0x3A;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_00 = 0x3B;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_01 = 0x3C;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_02 = 0x3D;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_03 = 0x3E;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_04 = 0x3F;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_05 = 0x40;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_06 = 0x41;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_07 = 0x42;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_08 = 0x43;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_09 = 0x44;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_10 = 0x45;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_11 = 0x46;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_12 = 0x47;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_13 = 0x48;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_14 = 0x49;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_15 = 0x4A;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_16 = 0x4B;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_17 = 0x4C;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_18 = 0x4D;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_19 = 0x4E;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_20 = 0x4F;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_21 = 0x50;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_22 = 0x51;
static constexpr uint8_t I2C_ICM_EXT_SENS_DATA_23 = 0x52;
static constexpr uint8_t I2C_ICM_FIFO_EN_1		  = 0x66;
static constexpr uint8_t I2C_ICM_FIFO_EN_2		  = 0x67; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_FIFO_RST		  = 0x68; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_FIFO_MODE		  = 0x69; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_FIFO_COUNTH	  = 0x70;
static constexpr uint8_t I2C_ICM_FIFO_COUNTL	  = 0x71;
static constexpr uint8_t I2C_ICM_FIFO_R_W		  = 0x72;
static constexpr uint8_t I2C_ICM_DATA_RDY_STATUS  = 0x74; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_FIFO_CFG		  = 0x76; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_REG_BANK_SEL	 = 0x7F; // Not found in MPU-9250

///
/// USER BANK1 Register map.
///
static constexpr uint8_t I2C_ICM_SELF_TEST_X_GYRO		 = 0x02;
static constexpr uint8_t I2C_ICM_SELF_TEST_Y_GYRO		 = 0x03;
static constexpr uint8_t I2C_ICM_SELF_TEST_Z_GYRO		 = 0x04;
static constexpr uint8_t I2C_ICM_SELF_TEST_X_ACCEL		 = 0x0E;
static constexpr uint8_t I2C_ICM_SELF_TEST_Y_ACCEL		 = 0x0F;
static constexpr uint8_t I2C_ICM_SELF_TEST_Z_ACCEL		 = 0x10;
static constexpr uint8_t I2C_ICM_XA_OFFSET_H			 = 0x14;
static constexpr uint8_t I2C_ICM_XA_OFFSET_L			 = 0x15;
static constexpr uint8_t I2C_ICM_YA_OFFSET_H			 = 0x17;
static constexpr uint8_t I2C_ICM_YA_OFFSET_L			 = 0x18;
static constexpr uint8_t I2C_ICM_ZA_OFFSET_H			 = 0x1A;
static constexpr uint8_t I2C_ICM_ZA_OFFSET_L			 = 0x1B;
static constexpr uint8_t I2C_ICM_TIMEBASE_CORRECTION_PLL = 0x28;

///
/// USER BANK2 Register map.
///
static constexpr uint8_t I2C_ICM_GYRO_SMPLRT_DIV = 0x00; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_GYRO_CONFIG_1   = 0x01; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_GYRO_CONFIG_2   = 0x02; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_XG_OFFSET_H	 = 0x03; // User-defined trim values for gyroscope
static constexpr uint8_t I2C_ICM_XG_OFFSET_L	 = 0x04;
static constexpr uint8_t I2C_ICM_YG_OFFSET_H	 = 0x05;
static constexpr uint8_t I2C_ICM_YG_OFFSET_L	 = 0x06;
static constexpr uint8_t I2C_ICM_ZG_OFFSET_H	 = 0x07;
static constexpr uint8_t I2C_ICM_ZG_OFFSET_L	 = 0x08;
static constexpr uint8_t I2C_ICM_ODR_ALIGN_EN	= 0x09; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_ACCEL_SMPLRT_DIV_1 = 0x10; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_ACCEL_SMPLRT_DIV_2 = 0x11; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_ACCEL_INTEL_CTRL   = 0x12; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_ACCEL_WOM_THR  = 0x13; // Not found in MPU-9250 (could be WOM_THR)
static constexpr uint8_t I2C_ICM_ACCEL_CONFIG   = 0x14;
static constexpr uint8_t I2C_ICM_ACCEL_CONFIG_2 = 0x15;
static constexpr uint8_t I2C_ICM_FSYNC_CONFIG   = 0x52; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_TEMP_CONFIG	= 0x53; // Not found in MPU-9250
static constexpr uint8_t I2C_ICM_MOD_CTRL_USR   = 0x54; // Not found in MPU-9250

///
/// USER BANK3 Register map
///
static constexpr uint8_t I2C_MST_ODR_CONFIG = 0x00; // Not found in MPU-9250
static constexpr uint8_t I2C_MST_CTRL		= 0x01;
static constexpr uint8_t I2C_MST_DELAY_CTRL = 0x02;
static constexpr uint8_t I2C_SLV0_ADDR		= 0x03;
static constexpr uint8_t I2C_SLV0_REG		= 0x04;
static constexpr uint8_t I2C_SLV0_CTRL		= 0x05;
static constexpr uint8_t I2C_SLV0_DO		= 0x06;
static constexpr uint8_t I2C_SLV1_ADDR		= 0x07;
static constexpr uint8_t I2C_SLV1_REG		= 0x08;
static constexpr uint8_t I2C_SLV1_CTRL		= 0x09;
static constexpr uint8_t I2C_SLV1_DO		= 0x0A;
static constexpr uint8_t I2C_SLV2_ADDR		= 0x0B;
static constexpr uint8_t I2C_SLV2_REG		= 0x0C;
static constexpr uint8_t I2C_SLV2_CTRL		= 0x0D;
static constexpr uint8_t I2C_SLV2_DO		= 0x0E;
static constexpr uint8_t I2C_SLV3_ADDR		= 0x0F;
static constexpr uint8_t I2C_SLV3_REG		= 0x10;
static constexpr uint8_t I2C_SLV3_CTRL		= 0x11;
static constexpr uint8_t I2C_SLV3_DO		= 0x12;
static constexpr uint8_t I2C_SLV4_ADDR		= 0x13;
static constexpr uint8_t I2C_SLV4_REG		= 0x14;
static constexpr uint8_t I2C_SLV4_CTRL		= 0x15;
static constexpr uint8_t I2C_SLV4_DO		= 0x16;
static constexpr uint8_t I2C_SLV4_DI		= 0x17;

static constexpr uint8_t READ_FLAG = 0x80;
///
/// USER_CTRL functions
///
enum ICM_USER_CTRL : uint8_t
{
    // Enable DMP
    DMP_ENABLE = 0x80,
    // Enable FIFO operation mode
    FIFO_ENABLE = 0x40,
    // I2C Master enable.
    I2C_MASTER_ENABLE = 0x20,
    // I2C Slave mode and put the serial interface in SPI mode.
    I2C_IF_DIS = 0x10,
    // Reset DMP module
    DMP_RST = 0x08,
    // Reset SRAM Module.
    SRAM_RST = 0x04,
    // Reset I2C master module
    I2C_MST_RST = 0x02
};

///
/// PWR_MGMT_1 Functions
///
enum ICM_PWR_MGMT_1 : uint8_t
{
    // Device reset - Resets the internal registers and restores to default settings.
    DEVICE_RESET = 0x80,
    // Sleep - Set the chip in sleep mode. When set, the chip is set to sleep mode (in sleep mode
    // all analog is powered off). Clearing the bit wakes the chip from sleep mode.
    SLEEP = 0x40,
    // LP_EN - Helps to reduce the digital current when
    // sensors are in LP mode. Please note that the sensors themselves are set in LP mode
    // by the LP_CONFIG register settings
    LP_EN = 0x20,
    // TEMP_DIS = When set to 1, this bit disables the temperature sensor.
    TEMP_DIS = 0x08,
    // CLKSEL[2:0]
    // 0: Internal 20 MHz oscillator
    // 1-5: Auto selects the best available clock source – PLL if ready, else use the Internal
    // oscillator 6: Internal 20 MHz oscillator 7: Stops the clock and keeps timing generator in
    // reset NOTE: CLKSEL[2:0] should be set to 1~5 to achieve full gyroscope performance.
    CLKSEL_INTERNAL   = 0x00,
    CLKSEL_AUTO		  = 0x01,
    CLKSEL_STOP_RESET = 0x07
};

///
/// INT_PIN_CFG
///
enum INT_PIN_CFG : uint8_t
{
    // INT1_ACTL -
    // 1 – The logic level for INT1 pin is active low.
    // 0 – The logic level for INT1 pin is active high
    INT1_ACTL_LOW = 0x80,
    // INT1_OPEN
    // 1 – INT1 pin is configured as open drain.
    // 0 – INT1 pin is configured as push-pull.
    INT1_OPEN_DRAIN = 0x40,
    // INT1_LATCH_EN
    // 1 – INT1 pin level held until interrupt status is cleared.
    // 0 – INT1 pin indicates interrupt pulse is width 50 µs.
    INT1_LATCH_EN_HELD_UNTIL_CLEARED = 0x20,
    // INT_ANYRD_2CLEAR
    // 1 – Interrupt status in INT_STATUS is cleared (set to 0) if any read operation is
    // performed.
    // 0 – Interrupt status in INT_STATUS is cleared (set to 0) only by reading INT_STATUS
    // register
    INT_ANYRD_2CLEAR_INT_STATUS = 0x10,
    // ACTL_FSYNC
    ACTL_FSYNC_ACTIVE_LOW = 0x08,
    // FSYNC_INT_MODE_EN
    FSYNC_INT_MODE_EN_INTERRUPT = 0x04,
    // BYPASS_EN
    BYPASS_EN = 0x02
};

///
/// INT_ENABLE_1
///
enum INT_ENABLE_1 : uint8_t
{
    // RAW_DATA_0_RDY_EN
    // 1 – Enable raw data ready interrupt from any sensor to propagate to interrupt
    // pin 1.
    // 0 – Function is disabled.
    RAW_DATA_0_RDY_EN = 0x01
};

///
/// REG_BANK_SEL
///
enum REG_BANK_SEL : uint8_t
{
    SELECT_USER_BANK_0 = 0x00,
    SELECT_USER_BANK_1 = 0x10,
    SELECT_USER_BANK_2 = 0x20,
    SELECT_USER_BANK_3 = 0x30
};

///
/// GYRO_SMPLRT_DIV
///
enum GYRO_SMPLRT_DIV : uint8_t
{
    GYRO_SMPLRT_DIV_225HZ = 0x04
};

///
/// GYRO_CONFIG_1
///
enum GYRO_CONFIG_1 : uint8_t
{
    // Enable Digital low-pass filter for Gyro values.
    GYRO_ENABLE_DLPF = 0x01,
    // GYRO_FS_SEL
    GYRO_FS_SEL_250  = 0x00,
    GYRO_FS_SEL_500  = 0x02,
    GYRO_FS_SEL_1000 = 0x04,
    GYRO_FS_SEL_2000 = 0x06,
    // GYRO_DLPFCFG
    GYRO_DLPFCFG_151BW_187NBW = 0x08,
    GYRO_DLPFCFG_119BW_154NBW = 0x10,
    GYRO_DLPFCFG_51BW_73NBW   = 0x18,
    GYRO_DLPFCFG_23BW_35NBW   = 0x20,
    GYRO_DLPFCFG_11BW_17NBW   = 0x28,
    GYRO_DLPFCFG_5BW_8NBW	 = 0x30,
    GYRO_DLPFCFG_361BW_376NBW = 0x38
};

///
/// ACCEL_SMPLRT_DIV_2
///
enum ACCEL_SMPLRT_DIV2 : uint8_t
{
    ACCEL_SMPLRT_DIV2_225HZ = 0x04
};

///
/// ACCEL_CONFIG
///
enum ACCEL_CONFIG : uint8_t
{
    // ACCEL_FCHOICE
    ACCEL_ENABLE_DLPF = 0x01,
    // ACCEL_FS_SEL
    ACCEL_FS_SEL_2G  = 0x00,
    ACCEL_FS_SEL_4G  = 0x02,
    ACCEL_FS_SEL_8G  = 0x04,
    ACCEL_FS_SEL_16G = 0x06,
    // ACCEL_DLPFCFG
    ACCEL_DLPFCFG_246BW_265NBW = 0x08,
    ACCEL_DLPFCFG_111BW_136NBW = 0x10,
    ACCEL_DLPFCFG_50BW_68NBW   = 0x18,
    ACCEL_DLPFCFG_23BW_34NBW   = 0x20,
    ACCEL_DLPFCFG_11BW_17NBW   = 0x28,
    ACCEL_DLPFCFG_5BW_8NBW	 = 0x30,
    ACCEL_DLPFCFG_473BW_499NBW = 0x38
};
} // namespace ICM20948REG

#endif // ICM20948_REGISTER_H
