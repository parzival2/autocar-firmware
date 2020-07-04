#ifndef OROCOS_ICM20948_RTT_PIGPIO_COMPONENT_HPP
#define OROCOS_ICM20948_RTT_PIGPIO_COMPONENT_HPP

#include "icm20948_register.h"
#include <rtt/RTT.hpp>
#ifdef __arm__
#include <pigpiod_if2.h>
#else
#include "pigpiod_if2.h"
#endif
#include <sensor_msgs/Imu.h>

class Icm20948_rtt_pigpio : public RTT::TaskContext
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
    // Acceleration due to gravity for converting g's to m/sec^2.
    static constexpr float ACCEL_DUE_TO_GRAVITY  = 9.80665;
    static constexpr float DEGREES_TO_RAD_FACTOR = 0.0174533;
    // GPIO number
    // It varies depending on the library we are using
    // pigpio - GPIO 6
    // wiringpi - GPIO 22
    static constexpr uint8_t PIGPIO_ICM_INTERRUPT_PIN = 6;
    // Needed only for pigpio
    // The Waveshare sensehat uses P2 and P3 pins for I2C which is I2C1 bus on Raspberry pi.
    static constexpr uint8_t I2C_BUS_NUMBER = 1;
    Icm20948_rtt_pigpio(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    static void handleInterrupt(int pi, uint32_t gpio, uint32_t level, uint32_t tick,
                                void* imuPointer);
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
     * @brief mPiHandle The handle that will be returned when pigpio is connected to daemon.
     */
    int mPiHandle;

    /**
     * @brief mI2cDeviceId The wiringpi library returns a device ID which should be used for all
     * communications with the device.
     */
    int mI2cDeviceFileDescriptor;

    /**
     * @brief mCurrentImuMessage The current IMU message that we have extracted from I2C device.
     */
    sensor_msgs::Imu mCurrentImuMessage;

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
