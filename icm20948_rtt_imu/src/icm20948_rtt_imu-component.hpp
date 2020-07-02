#ifndef OROCOS_ICM20948_RTT_IMU_COMPONENT_HPP
#define OROCOS_ICM20948_RTT_IMU_COMPONENT_HPP

#include <rtt/RTT.hpp>
// Include the original header files if we detect that we are in arm environment.
#ifdef __arm__
#include <functional>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#else
#include "wiringPi.h"
#include "wiringPiI2C.h"
#endif
#include "icm20948_register.h"
#include <sensor_msgs/Imu.h>

class Icm20948_rtt_imu : public RTT::TaskContext
{
  public:
    // Gyro
    static constexpr float GYRO_DPS_250  = 131.0;
    static constexpr float GYRO_DPS_500  = 65.5;
    static constexpr float GYRO_DPS_1000 = 32.8;
    static constexpr float GYRO_DPS_2000 = 16.4;
    // Accel
    static constexpr float ACCEL_2G  = 16384.0;
    static constexpr float ACCEL_4G  = 8192.0;
    static constexpr float ACCEL_8G  = 4096.0;
    static constexpr float ACCEL_16G = 2048.0;
    /**
     * @brief The State enum The current state to indicate whether the communication with sensor has
     * been successful or not.
     */
    enum class CommunicationState : uint8_t
    {
        // After all the configuration is done
        INITIALIZED = 0,
        // Only the communication to the device has been successful.
        COMMUNICATION_SUCCESSFUL = 1,
        // There is a communication failure.
        COMMUNICATION_FAILURE = 2,
        // The Device has been found successfully.
        I2C_DEVICE_FOUND = 3,
        // Uninitialized. Will be initialized to this when first initialized using constructor.
        UNINITIALIZED = 4,
        // Cannot communicate with the device. Might be linux driver error.
        DEVICE_ERROR = 5,
    };
    Icm20948_rtt_imu(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    static void handleInterrupt(void* thisClassPointer);
    // The full-scale selection of the Gyro.
    enum class GyroScale : uint8_t
    {
        DPS_250  = 0,
        DPS_500  = 1,
        DPS_1000 = 2,
        DPS_2000 = 3
    };
    // The full-scale selection of the Accelerometer.
    enum class AccelScale : uint8_t
    {
        ACCEL_2G  = 0,
        ACCEL_4G  = 1,
        ACCEL_8G  = 2,
        ACCEL_16G = 3
    };
    /**
     * @brief mICMCurrentState The current state of communications with the device.
     */
    CommunicationState mICMCurrentState;

    /**
     * @brief mI2cDeviceId The wiringpi library returns a device ID which should be used for all
     * communications with the device.
     */
    int mI2cDeviceFileDescriptor;

    /**
     * @brief mCurrentImuMessage The current imu messages that has been filled by reading over I2C
     * when the interrupt has been fired.
     */
    sensor_msgs::Imu mCurrentImuMessage;

    /**
     * @brief mCurrentGyroScale The value with which incoming values will be scaled to get angular
     * velocity in degrees per second.
     */
    GyroScale mCurrentGyroScale;

    /**
     * @brief mCurrentAccelScale The value with which incoming values will be scaled to get
     * acceleration in g.
     */
    AccelScale mCurrentAccelScale;

    /**
     * @brief mInterruptInPort The port which wakes up on the event. This will actually
     * be an event which will wake up thread and call updateHook method.
     * This port will be connected with the out port just defined below.
     */
    Orocos::InputPort<int> mInterruptInPort;

    /**
     * @brief mInterruptOutPort The port into which some information will be written when the
     * interrupt happens.
     */
    Orocos::OutputPort<int> mInterruptOutPort;
};
#endif
