#include "icm20948_rtt_imu-component.hpp"
#include <iostream>
#include <rtt/Component.hpp>

Icm20948_rtt_imu::Icm20948_rtt_imu(std::string const& name)
    : TaskContext(name)
    , mICMCurrentState(CommunicationState::UNINITIALIZED)
    , mI2cDeviceFileDescriptor(0)
    , mCurrentGyroScale(GyroScale::DPS_250)
    , mCurrentAccelScale(AccelScale::ACCEL_2G)
    , mInterruptInPort()
    , mInterruptOutPort()
{
    this->ports()
        ->addEventPort("interruptIn", mInterruptInPort)
        .doc("Interrupt port. Will be waken up when an event is fired.");
    this->ports()->addPort("interruptOut", mInterruptOutPort).doc("Output port for interrupt.");
}

bool Icm20948_rtt_imu::configureHook()
{
    // This would be the correct call. There is another call wiringPiSetupGpio() which assumes that
    // we would be using the Broadcom numbering scheme and doesnt not work with wiringpi pinouts.
    // More info can be found here : https://projects.drogon.net/raspberry-pi/wiringpi/functions/
    wiringPiSetup();
    // Pulldown the Pin22
    pinMode(22, INPUT);
    pullUpDnControl(22, PUD_UP);
    int interruptResult =
        wiringPiISR(22, INT_EDGE_RISING, &Icm20948_rtt_imu::handleInterrupt, this);
    Orocos::log(Orocos::Info) << "[Icm20948_rtt_imu::configureHook] Interrupt return result : "
                              << interruptResult << Orocos::endlog();
    mI2cDeviceFileDescriptor = wiringPiI2CSetup(ICM20948REG::I2C_ICM_ADDRESS);
    // If the returned ID is negative, then there is an error.
    if(mI2cDeviceFileDescriptor < 0)
    {
        mICMCurrentState = CommunicationState::DEVICE_ERROR;
        return false;
    }
    else
    {
        mICMCurrentState = CommunicationState::COMMUNICATION_SUCCESSFUL;
        // Select the userbank 0
        wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_REG_BANK_SEL,
                             ICM20948REG::REG_BANK_SEL::SELECT_USER_BANK_0);
        // Reset the device
        wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_PWR_MGMT_1,
                             ICM20948REG::ICM_PWR_MGMT_1::DEVICE_RESET);
        usleep(10000);
        // We start with PWR_MGMT_1 register which will switch on the device.
        wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_PWR_MGMT_1,
                             ICM20948REG::ICM_PWR_MGMT_1::CLKSEL_AUTO);
        // First check whether the IMU is connected.
        // Can be checked by reading the WHO_AM_I register.
        int whoAmIReturn =
            wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_WHO_AM_I);
        if(whoAmIReturn == ICM20948REG::I2C_ICM_WHO_AM_I_ANSWER)
        {
            // Set the current state of the device.
            mICMCurrentState = CommunicationState::I2C_DEVICE_FOUND;
            // We will start initializing the device by setting the parameters that we need most.
            // Select the user bank 2 for further configuration
            wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_REG_BANK_SEL,
                                 ICM20948REG::REG_BANK_SEL::SELECT_USER_BANK_2);
            int userbankSelected =
                wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_REG_BANK_SEL);
            // GYRO_SMPLRT_DIV
            wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_GYRO_SMPLRT_DIV,
                                 ICM20948REG::GYRO_SMPLRT_DIV::GYRO_SMPLRT_DIV_225HZ);
            // GYRO_CONFIG_1
            wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_GYRO_CONFIG_1,
                                 ICM20948REG::GYRO_CONFIG_1::GYRO_DLPFCFG_361BW_376NBW |
                                     ICM20948REG::GYRO_CONFIG_1::GYRO_FS_SEL_250 |
                                     ICM20948REG::GYRO_CONFIG_1::GYRO_ENABLE_DLPF);

            // ACCEL_SMPLRT_DIV_2
            wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_SMPLRT_DIV_2,
                                 ICM20948REG::ACCEL_SMPLRT_DIV2::ACCEL_SMPLRT_DIV2_225HZ);
            // ACCEL_CONFIG
            wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_CONFIG,
                                 ICM20948REG::ACCEL_CONFIG::ACCEL_DLPFCFG_473BW_499NBW |
                                     ICM20948REG::ACCEL_CONFIG::ACCEL_FS_SEL_2G |
                                     ICM20948REG::ACCEL_CONFIG::ACCEL_ENABLE_DLPF);

            // Switch to userbank 0
            wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_REG_BANK_SEL,
                                 ICM20948REG::REG_BANK_SEL::SELECT_USER_BANK_0);
            // Enable DATA_RDY interrupt
            wiringPiI2CWriteReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_INT_ENABLE_1,
                                 ICM20948REG::INT_ENABLE_1::RAW_DATA_0_RDY_EN);
            // Interrupt pin
            int interruptEnable1Reg =
                wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_INT_ENABLE_1);
            Orocos::log(Orocos::Info) << "[Icm20948_rtt_imu::configureHook] Is interrupt enabled : "
                                      << interruptEnable1Reg << Orocos::endlog();
        }
    }
    return true;
}

bool Icm20948_rtt_imu::startHook()
{
    Orocos::log(Orocos::Info) << "[Icm20948_rtt_imu::startHook] Starthook has been called."
                              << Orocos::endlog();
    return true;
}

void Icm20948_rtt_imu::updateHook()
{
    // First we create a buffer to hold the values.
    // We will be reading 1 byte everytime but the actual resolution is 16 bits. So to hold all the
    // 6 bytes, we will need a buffer.
    std::array<uint8_t, 2> lowHighAccelBuffer = {0, 0};
    std::array<int16_t, 3> rawAccelBuffer	 = {0, 0, 0};
    // ACCEL_X
    lowHighAccelBuffer[0] =
        wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_XOUT_H);
    lowHighAccelBuffer[1] =
        wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_XOUT_L);
    rawAccelBuffer[0] = (lowHighAccelBuffer[0] << 8) | lowHighAccelBuffer[1];
    // ACCEL_Y
    lowHighAccelBuffer[0] =
        wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_YOUT_H);
    lowHighAccelBuffer[1] =
        wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_YOUT_L);
    rawAccelBuffer[1] = (lowHighAccelBuffer[0] << 8) | lowHighAccelBuffer[1];
    // ACCEL_Z
    lowHighAccelBuffer[0] =
        wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_ZOUT_H);
    lowHighAccelBuffer[1] =
        wiringPiI2CReadReg8(mI2cDeviceFileDescriptor, ICM20948REG::I2C_ICM_ACCEL_ZOUT_L);
    rawAccelBuffer[2] = (lowHighAccelBuffer[0] << 8) | lowHighAccelBuffer[1];
    Orocos::log(Orocos::Info) << "[Icm20948_rtt_imu::updateHook] \n ACCEL_X : " << rawAccelBuffer[0]
                              << "\n ACCEL_Y : " << rawAccelBuffer[1]
                              << "\n ACCEL_Z : " << rawAccelBuffer[2] << Orocos::endlog();
}

void Icm20948_rtt_imu::stopHook()
{
    Orocos::log(Orocos::Info) << "[Icm20948_rtt_imu::stopHook] stopHook has been called."
                              << Orocos::endlog();
}

void Icm20948_rtt_imu::cleanupHook()
{
    Orocos::log(Orocos::Info) << "[Icm20948_rtt_imu::cleanupHook] cleanupHook has been called."
                              << Orocos::endlog();
}

/**
 * @brief Icm20948_rtt_imu::handleInterrupt Static function that handles the interrupt.
 * @param imuPointer The `this` pointer of this class, so that we can call other member functions of
 * this class.
 */
void Icm20948_rtt_imu::handleInterrupt(void* imuPointer)
{
    Orocos::log(Orocos::Info)
        << "[Icm20948_rtt_imu::handleInterrupt] Handleinterrupt has been called : "
        << Orocos::endlog();
    Icm20948_rtt_imu* thisPointer = reinterpret_cast<Icm20948_rtt_imu*>(imuPointer);
    if(thisPointer)
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
 * ORO_LIST_COMPONENT_TYPE(Icm20948_rtt_imu)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Icm20948_rtt_imu)
