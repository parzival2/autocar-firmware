#ifndef ICM20948_H
#define ICM20948_H
#include "icm20948_register.h"
#ifdef __arm__
#include <wiringPi.h>
#include <wiringPiI2C.h>
#else
#include <raspi_icm_imu/wiringPi.h>
#include <raspi_icm_imu/wiringPiI2C.h>
#endif
// ROS related headers
#include <functional>
#include <sensor_msgs/Imu.h>

class icm20948
{
  public:
    using SetImuValues = std::function<void(const sensor_msgs::Imu&)>;
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
    // Acceleration due to gravity for converting g's to m/sec^2.
    static constexpr float ACCEL_DUE_TO_GRAVITY = 9.80665;
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

    /**
     * @brief AXIS_RANGE The number of axis in each of the sensor. Gyro and Accelerometer both
     * measure angular velocity and acceleration in X,Y and Z axes respectively.
     */
    static constexpr uint8_t AXIS_RANGE = 3;

    icm20948();
    const CommunicationState& initialize();
    void probeDevice();
    const CommunicationState& getCommunicationState() const;
    static void handleInterrupt(void* imuPointer);
    void setImuValueFunction(SetImuValues& imuValueFunction);

  private:
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

    void acquireImuReadings();

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

    SetImuValues mSetImuValueFunction;
};

#endif // ICM20948_H
