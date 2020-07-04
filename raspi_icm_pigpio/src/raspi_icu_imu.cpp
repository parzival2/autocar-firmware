#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#ifdef __arm__
#include "icm20948.h"
#include <iostream>
#endif

int main(int argc, char** argv)
{
    // Initialize the ros node.
    ros::init(argc, argv, "wiringpi_start_example");

    // Nodehandle
    ros::NodeHandle nodeHandle  = ros::NodeHandle();
    ros::Publisher imuPublisher = nodeHandle.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
#ifdef __arm__
    icm20948 icmImuDevice;

    icm20948::SetImuValues setFunction = static_cast<icm20948::SetImuValues>(
        [&imuPublisher](const sensor_msgs::Imu& imuValues) { imuPublisher.publish(imuValues); });
    icmImuDevice.setImuValueFunction(setFunction);
    icmImuDevice.initialize();
    while(ros::ok())
    {
        icmImuDevice.probeDevice();
        icm20948::CommunicationState state = icmImuDevice.getCommunicationState();
        ROS_INFO_STREAM(" The current state of the device : " << static_cast<int>(state));
        ros::Duration(1.0).sleep();
    }
    icmImuDevice.cleanup();
    return 0;
#endif
}
