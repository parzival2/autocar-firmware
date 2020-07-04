#include "icm20948_rtt_pigpio-component.hpp"
#include <rtt/Component.hpp>

Icm20948_rtt_pigpio::Icm20948_rtt_pigpio(std::string const& name)
    : TaskContext(name)
    , mPiHandle(0)
    , mI2cDeviceFileDescriptor(0)
    , mInterruptInPort()
    , mInterruptOutPort()
{
    // Important to keep the port names consistent as this will be the name we use while writing our
    // ops script.
    this->ports()
        ->addEventPort("interruptIn", mInterruptInPort)
        .doc("Interrupt port. Will be waken up when an event is fired.");
    this->ports()->addPort("interruptOut", mInterruptOutPort).doc("Output port for interrupt.");
}

bool Icm20948_rtt_pigpio::configureHook()
{
    int mPiHandle = pigpio_start(NULL, NULL);
    if(mPiHandle == PI_INIT_FAILED)
    {
        Orocos::log(Orocos::Error)
            << "[Icm20948_rtt_pigpio::configureHook] Cannot initialize pigpio interface."
            << Orocos::endlog();
        return false;
    }
    // Set the mode for the pin that will capture the interrupts
    set_mode(mPiHandle, PIGPIO_ICM_INTERRUPT_PIN, PI_INPUT);
    // Pull-up the pin
    set_pull_up_down(mPiHandle, PIGPIO_ICM_INTERRUPT_PIN, PI_PUD_UP);
    int interruptResult = callback_ex(mPiHandle, PIGPIO_ICM_INTERRUPT_PIN, RISING_EDGE,
                                      &Icm20948_rtt_pigpio::handleInterrupt, this);
    Orocos::log(Orocos::Info) << "[Icm20948_rtt_pigpio::configureHook] callback_ex result : "
                              << interruptResult << Orocos::endlog();
    // Initialize the communication with the device.
    // TODO: Are we sure that it would be successful in only one attempt?
    mI2cDeviceFileDescriptor = i2c_open(mPiHandle, I2C_BUS_NUMBER, ICM20948REG::I2C_ICM_ADDRESS, 0);
    // If the returned ID is negative, then there is an error.
    if(mI2cDeviceFileDescriptor < 0)
    {
        Orocos::log(Orocos::Error)
            << "[Icm20948_rtt_pigpio::configureHook] Cannot communicate with I2C device."
            << interruptResult << Orocos::endlog();
        return false;
    }
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
    Orocos::log(Orocos::Info) << "[Icm20948_rtt_pigpio::configureHook] WHO_AM_I return result : "
                              << whoAmIReturn << Orocos::endlog();
    if(whoAmIReturn == ICM20948REG::I2C_ICM_WHO_AM_I_ANSWER)
    {
        Orocos::log(Orocos::Info) << "[Icm20948_rtt_pigpio::configureHook] ICM20948 found."
                                  << Orocos::endlog();
        // We will start initializing the device by setting the parameters that we need most.
        // Select the user bank 2 for further configuration
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_REG_BANK_SEL,
                            ICM20948REG::REG_BANK_SEL::SELECT_USER_BANK_2);
        i2c_read_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_REG_BANK_SEL);
        // GYRO_SMPLRT_DIV
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                            ICM20948REG::I2C_ICM_GYRO_SMPLRT_DIV,
                            ICM20948REG::GYRO_SMPLRT_DIV::GYRO_SMPLRT_DIV_225HZ);
        // GYRO_CONFIG_1
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_GYRO_CONFIG_1,
                            ICM20948REG::GYRO_CONFIG_1::GYRO_DLPFCFG_361BW_376NBW |
                                ICM20948REG::GYRO_CONFIG_1::GYRO_FS_SEL_250 |
                                ICM20948REG::GYRO_CONFIG_1::GYRO_ENABLE_DLPF);

        // ACCEL_SMPLRT_DIV_2
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor,
                            ICM20948REG::I2C_ICM_ACCEL_SMPLRT_DIV_2,
                            ICM20948REG::ACCEL_SMPLRT_DIV2::ACCEL_SMPLRT_DIV2_225HZ);
        // ACCEL_CONFIG
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_CONFIG,
                            ICM20948REG::ACCEL_CONFIG::ACCEL_DLPFCFG_473BW_499NBW |
                                ICM20948REG::ACCEL_CONFIG::ACCEL_FS_SEL_2G |
                                ICM20948REG::ACCEL_CONFIG::ACCEL_ENABLE_DLPF);

        // Switch to userbank 0
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_REG_BANK_SEL,
                            ICM20948REG::REG_BANK_SEL::SELECT_USER_BANK_0);
        // Enable DATA_RDY interrupt
        i2c_write_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_INT_ENABLE_1,
                            ICM20948REG::INT_ENABLE_1::RAW_DATA_0_RDY_EN);
        // Interrupt pin
        i2c_read_byte_data(mPiHandle, mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_INT_ENABLE_1);
        return true;
    }
}

bool Icm20948_rtt_pigpio::startHook()
{
    return true;
}

void Icm20948_rtt_pigpio::updateHook()
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
        (rawAccelBuffer[0] / ACCEL_2G) * Icm20948_rtt_pigpio::ACCEL_DUE_TO_GRAVITY;
    mCurrentImuMessage.linear_acceleration.y =
        (rawAccelBuffer[1] / ACCEL_2G) * Icm20948_rtt_pigpio::ACCEL_DUE_TO_GRAVITY;
    mCurrentImuMessage.linear_acceleration.z =
        (rawAccelBuffer[2] / ACCEL_2G) * Icm20948_rtt_pigpio::ACCEL_DUE_TO_GRAVITY;
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
    Orocos::log(Orocos::Info) << "[Acceleration - m/s2] : "
                              << "ACCEL_X : " << mCurrentImuMessage.linear_acceleration.x
                              << ", ACCEL_Y : " << mCurrentImuMessage.linear_acceleration.y
                              << ", ACCEL_Z : " << mCurrentImuMessage.linear_acceleration.z
                              << Orocos::endlog();
}

void Icm20948_rtt_pigpio::stopHook() {}

void Icm20948_rtt_pigpio::cleanupHook()
{
    i2c_close(mPiHandle, mI2cDeviceFileDescriptor);
    pigpio_stop(mPiHandle);
}

/**
 * @brief icm20948::handleInterrupt Static function that handles the interrupt.
 * @param imuPointer The `this` pointer of this class, so that we can call other member functions of
 * this class.
 */
void Icm20948_rtt_pigpio::handleInterrupt(int pi, uint32_t gpio, uint32_t level, uint32_t tick,
                                          void* imuPointer)
{
    Icm20948_rtt_pigpio* thisPointer = reinterpret_cast<Icm20948_rtt_pigpio*>(imuPointer);
    if(level == FALLING_EDGE)
    {
        // TODO: Is this a good idea to have two ports in the same class?
        // thisPointer->updateHook();
        // thisPointer->engine()->getActivity()->trigger();
        thisPointer->mInterruptOutPort.write(1);
    }
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Icm20948_rtt_pigpio)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Icm20948_rtt_pigpio)
