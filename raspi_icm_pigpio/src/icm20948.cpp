#include "icm20948.h"
#include <ros/ros.h>
#include <unistd.h>

/**
 * @brief icm20948::icm20948 Constructor
 */
icm20948::icm20948()
    : mICMCurrentState(CommunicationState::UNINITIALIZED)
    , mPiHandle(0)
    , mI2cDeviceFileDescriptor(0)
    , mCurrentGyroScale(GyroScale::DPS_250)
    , mCurrentAccelScale(AccelScale::ACCEL_2G)
{
}

/**
 * @brief icm20948::initialize Initialize the communications and setup ICM20948 IMU.
 * This function should be called for the device communcations to start.
 */
const icm20948::CommunicationState& icm20948::initialize()
{
    int mPiHandle = pigpio_start(NULL, NULL);
    if(mPiHandle == PI_INIT_FAILED)
    {
        ROS_ERROR_STREAM("Failed to Initialize pigpio library");
        mICMCurrentState = CommunicationState::DEVICE_ERROR;
        return mICMCurrentState;
    }
    // Set the mode for the pin that will capture the interrupts
    set_mode(mPiHandle, PIGPIO_ICM_INTERRUPT_PIN, PI_INPUT);
    // Pull-up the pin
    set_pull_up_down(mPiHandle, PIGPIO_ICM_INTERRUPT_PIN, PI_PUD_UP);
    int interruptResult = callback_ex(mPiHandle, PIGPIO_ICM_INTERRUPT_PIN, RISING_EDGE,
                                      &icm20948::handleInterrupt, this);
    ROS_INFO_STREAM("The result of interrupt attachment : " << interruptResult);
    // Initialize the communication with the device.
    // TODO: Are we sure that it would be successful in only one attempt?
    mI2cDeviceFileDescriptor = i2c_open(mPiHandle, I2C_BUS_NUMBER, ICM20948REG::I2C_ICM_ADDRESS, 0);
    // If the returned ID is negative, then there is an error.
    if(mI2cDeviceFileDescriptor < 0)
    {
        mICMCurrentState = CommunicationState::DEVICE_ERROR;
        return mICMCurrentState;
    }
    else
    {
        mICMCurrentState = CommunicationState::COMMUNICATION_SUCCESSFUL;
        // Select the userbank 0
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_REG_BANK_SEL,
                            ICM20948REG::REG_BANK_SEL::SELECT_USER_BANK_0);
        // Reset the device
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_PWR_MGMT_1,
                            ICM20948REG::ICM_PWR_MGMT_1::DEVICE_RESET);
        usleep(10000);
        // We start with PWR_MGMT_1 register which will switch on the device.
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_PWR_MGMT_1,
                            ICM20948REG::ICM_PWR_MGMT_1::CLKSEL_AUTO);
        // First check whether the IMU is connected.
        // Can be checked by reading the WHO_AM_I register.
        int whoAmIReturn =
            i2c_read_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_WHO_AM_I);
        ROS_INFO_STREAM("[icm20948::initialize] The WHO_AM_I register value is : " << whoAmIReturn);
        if(whoAmIReturn == ICM20948REG::I2C_ICM_WHO_AM_I_ANSWER)
        {
            // Set the current state of the device.
            mICMCurrentState = CommunicationState::I2C_DEVICE_FOUND;
            // We will start initializing the device by setting the parameters that we need most.
            // Select the user bank 2 for further configuration
            i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                ICM20948REG::I2C_ICM_REG_BANK_SEL,
                                ICM20948REG::REG_BANK_SEL::SELECT_USER_BANK_2);
            int userbankSelected = i2c_read_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                                      ICM20948REG::I2C_ICM_REG_BANK_SEL);
            ROS_INFO_STREAM(
                "[icm20948::initialize] The current userbank selected is : " << userbankSelected);

            // GYRO_SMPLRT_DIV
            i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                ICM20948REG::I2C_ICM_GYRO_SMPLRT_DIV,
                                ICM20948REG::GYRO_SMPLRT_DIV::GYRO_SMPLRT_DIV_225HZ);
            // GYRO_CONFIG_1
            i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                ICM20948REG::I2C_ICM_GYRO_CONFIG_1,
                                ICM20948REG::GYRO_CONFIG_1::GYRO_DLPFCFG_361BW_376NBW |
                                    ICM20948REG::GYRO_CONFIG_1::GYRO_FS_SEL_250 |
                                    ICM20948REG::GYRO_CONFIG_1::GYRO_ENABLE_DLPF);

            // ACCEL_SMPLRT_DIV_2
            i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                ICM20948REG::I2C_ICM_ACCEL_SMPLRT_DIV_2,
                                ICM20948REG::ACCEL_SMPLRT_DIV2::ACCEL_SMPLRT_DIV2_225HZ);
            // ACCEL_CONFIG
            i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                ICM20948REG::I2C_ICM_ACCEL_CONFIG,
                                ICM20948REG::ACCEL_CONFIG::ACCEL_DLPFCFG_473BW_499NBW |
                                    ICM20948REG::ACCEL_CONFIG::ACCEL_FS_SEL_2G |
                                    ICM20948REG::ACCEL_CONFIG::ACCEL_ENABLE_DLPF);

            // Switch to userbank 0
            i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                ICM20948REG::I2C_ICM_REG_BANK_SEL,
                                ICM20948REG::REG_BANK_SEL::SELECT_USER_BANK_0);
            // Enable DATA_RDY interrupt
            i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                ICM20948REG::I2C_ICM_INT_ENABLE_1,
                                ICM20948REG::INT_ENABLE_1::RAW_DATA_0_RDY_EN);
            // Interrupt pin
            int interruptEnable1Reg = i2c_read_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                                                         ICM20948REG::I2C_ICM_INT_ENABLE_1);
            ROS_INFO_STREAM("[icm20948::initialize] The register value of I2C_ICM_INT_ENABLE is == "
                            << interruptEnable1Reg);
        }
    }
}

/**
 * @brief icm20948::probeDevice Probe the device to find if the device is still available.0
 */
void icm20948::probeDevice()
{
    bool canBeProbed = ((mICMCurrentState == CommunicationState::COMMUNICATION_SUCCESSFUL) ||
                        (mICMCurrentState == CommunicationState::I2C_DEVICE_FOUND));
    if(canBeProbed)
    {
        int whoAmIReturn =
            i2c_read_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_WHO_AM_I);
        // ROS_INFO_STREAM("[icm20948::proveDevice] The return from the device is : " <<
        // whoAmIReturn);
        if(whoAmIReturn == ICM20948REG::I2C_ICM_WHO_AM_I_ANSWER)
        {
            mICMCurrentState = CommunicationState::I2C_DEVICE_FOUND;
        }
    }
    else
    {
        mICMCurrentState = CommunicationState::COMMUNICATION_SUCCESSFUL;
    }
}

/**
 * @brief icm20948::getCommunicationState Returns the current communication state of the device.
 * @return Returns the current communication state with the imu sensor.
 */
const icm20948::CommunicationState& icm20948::getCommunicationState() const
{
    return mICMCurrentState;
}

/**
 * @brief icm20948::handleInterrupt Static function that handles the interrupt.
 * @param imuPointer The `this` pointer of this class, so that we can call other member functions of
 * this class.
 */
void icm20948::handleInterrupt(int pi, uint32_t gpio, uint32_t level, uint32_t tick,
                               void* imuPointer)
{
    icm20948* thisPointer = reinterpret_cast<icm20948*>(imuPointer);
    ROS_INFO_STREAM("Got interrupt level : " << level);
    if(level == FALLING_EDGE)
    {
        thisPointer->acquireImuReadings();
    }
}

/**
 * @brief icm20948::acquireImuReadings This will acquire IMU readings from the connected I2C
 * device(in this case its ICM20948 sensor).
 */
void icm20948::acquireImuReadings()
{
    // First we create a buffer to hold the values.
    // We will be reading 1 byte everytime but the actual resolution is 16 bits. So to hold all the
    // 6 bytes, we will need a buffer.
    char accelByteBuffer[6];
    std::array<int16_t, 3> rawAccelBuffer = {0, 0, 0};
    // ACCEL_X
    i2c_read_i2c_block_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_XOUT_H,
                            accelByteBuffer, 6);
    rawAccelBuffer[0] = (accelByteBuffer[0] << 8) | accelByteBuffer[1];
    // ACCEL_Y
    rawAccelBuffer[1] = (accelByteBuffer[2] << 8) | accelByteBuffer[3];
    // ACCEL_Z
    rawAccelBuffer[2] = (accelByteBuffer[4] << 8) | accelByteBuffer[5];
    // Now we convert the values into m/sec2
    mCurrentImuMessage.linear_acceleration.x =
        (rawAccelBuffer[0] / ACCEL_2G) * icm20948::ACCEL_DUE_TO_GRAVITY;
    mCurrentImuMessage.linear_acceleration.y =
        (rawAccelBuffer[1] / ACCEL_2G) * icm20948::ACCEL_DUE_TO_GRAVITY;
    mCurrentImuMessage.linear_acceleration.z =
        (rawAccelBuffer[2] / ACCEL_2G) * icm20948::ACCEL_DUE_TO_GRAVITY;
    // // Gyroscope messages
    char gyroByteBuffer[6];
    std::array<int16_t, 3> rawGyroBuffer = {0, 0, 0};
    i2c_read_i2c_block_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_GYRO_XOUT_H,
                            gyroByteBuffer, 6);
    // GYRO_X
    rawGyroBuffer[0] = (gyroByteBuffer[0] << 8) | gyroByteBuffer[1];
    // GYRO_Y
    rawGyroBuffer[1] = (gyroByteBuffer[2] << 8) | gyroByteBuffer[3];
    // GYRO_Z
    rawGyroBuffer[2] = (gyroByteBuffer[4] << 8) | gyroByteBuffer[5];
    // Set the values in the IMU message
    mCurrentImuMessage.angular_velocity.x =
        (rawGyroBuffer[0] / GYRO_DPS_250) * DEGREES_TO_RAD_FACTOR;
    mCurrentImuMessage.angular_velocity.y =
        (rawGyroBuffer[1] / GYRO_DPS_250) * DEGREES_TO_RAD_FACTOR;
    mCurrentImuMessage.angular_velocity.z =
        (rawGyroBuffer[2] / GYRO_DPS_250) * DEGREES_TO_RAD_FACTOR;
    mCurrentImuMessage.header.stamp = ros::Time::now();
    mSetImuValueFunction(mCurrentImuMessage);
}

/**
 * @brief icm20948::setImuValueFunction The function that will be called when the interrupt is
 * called from the IMU sensor.
 * @param imuValueFunction The function to be called when interrupt happens.
 */
void icm20948::setImuValueFunction(SetImuValues& imuValueFunction)
{
    mSetImuValueFunction = imuValueFunction;
}

/**
 * @brief icm20948::cleanup Cleans up the pigpio interface. It resets the used DMA channels and
 * releases memory.
 */
void icm20948::cleanup()
{
    i2c_close(mPiHandle, mI2cDeviceFileDescriptor);
    pigpio_stop(mPiHandle);
}
