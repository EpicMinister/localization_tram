#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "piksi_rtk_msgs/ImuRawMulti.h"

ros::Publisher imu_pub;

void imu_raw(const sensor_msgs::Imu imu_raw) {

    sensor_msgs::Imu imu_msg;

    //change x and y for orientation in train and remove gravitation
    imu_msg.linear_acceleration.x = imu_raw.linear_acceleration.z + 0.32;
    imu_msg.linear_acceleration.y = imu_raw.linear_acceleration.y + 0.42;
    imu_msg.linear_acceleration.z = imu_raw.linear_acceleration.x + 9.77;

    imu_msg.angular_velocity.x = imu_raw.angular_velocity.z;
    imu_msg.angular_velocity.y = imu_raw.angular_velocity.y;
    imu_msg.angular_velocity.z = imu_raw.angular_velocity.x;

    imu_msg.header = imu_raw.header;

    
    imu_pub.publish(imu_msg);
}

//Run the Node
int main(int argc, char **argv) {
    ros::init(argc, argv, "converter");
    ros::NodeHandle nh;
    
    ros::Subscriber imu_raw_sub = nh.subscribe("/piksi/imu", 10, &imu_raw);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/corrected", 10);
    
    ros::spin();

    return 0;
}

