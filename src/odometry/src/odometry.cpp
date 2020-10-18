 #include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "piksi_rtk_msgs/ReceiverState_V2_4_1.h"
#include "odometry/MaasOdometry.h"
#include "plc_tcp_server/TramStatus.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher maas_odom_pub, odom_pub;
sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
geometry_msgs::Pose pose;
odometry::MaasOdometry maas_odom;
ros::Time last_stamp_imu;
ros::Time last_stamp_velocity;

bool rest_imu = false;
bool reset_velocity = false;

//way
double imu_way = 0;
double velocity_way = 0;

//imu velocities
double v_x = 0;
double v_y = 0;
double v_z = 0;

//Bezugspunkt
double lat_diff = 49.7;
double long_diff = 8.55;

bool fixed_rtk = false;
std::vector<geometry_msgs::Pose> map;

//calculate distance between two Points
double calc_distance(geometry_msgs::Pose a, geometry_msgs::Pose b) {  
    return std::sqrt(std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2));   
}

//calaculate Position vis navsat message
geometry_msgs::Pose calc_position(sensor_msgs::NavSatFix gnss) {
    geometry_msgs::Pose pose;
    pose.position.x = (gnss.longitude - long_diff) * 40075160 * std::cos(gnss.latitude * 3.1415 / 180) / 360;
    pose.position.y = (gnss.latitude - lat_diff) * 40008000 / 360;
    return pose;
}

//calculate orientation from map and position
geometry_msgs::Pose calc_orientation(std::vector<geometry_msgs::Pose> map, geometry_msgs::Pose start, double distance_toleranz) {
    
    geometry_msgs::Pose pose;
    tf2::Quaternion quaternion;
    quaternion.setRPY( 0, 0, 0 );
    pose.orientation = tf2::toMsg(quaternion);
    ROS_ERROR("1 %f", calc_distance(map[0], map[map.size()-1]));
    if(calc_distance(map[0], map[map.size()-1]) < 20){
        return pose;
    }     
    
    //iterate over map
    for (int i = 0; i < map.size(); i++){
        ROS_ERROR("2");
        geometry_msgs::Pose test;

        //compare every point with the distance
        for (int ii = i; i < map.size(); ii++){
        ROS_ERROR("3 %f", calc_distance(map[i], map[ii]));
                if (calc_distance(map[i], map[ii]) > 4.55){
                     
                    //calculate the first joint
                    double x = map[ii].position.x - map[i].position.x;
                    double y = map[ii].position.y - map[i].position.y;

                    x = x/std::sqrt(std::pow(x,2)+std::pow(y,2));
                    y = y/std::sqrt(std::pow(x,2)+std::pow(y,2));

                    test.position.x = map[i].position.x + x*(4.55+1.8);
                    test.position.y = map[i].position.y + y*(4.55+1.8);
                    
                    break;
                }
        }
    
ROS_ERROR("4 %f", calc_distance(test, start));
        //if joint has right distance, calulate orientation
        if (calc_distance(test, start) > 5.825){
            double yaw = atan2(test.position.x-start.position.x, test.position.y-start.position.y);
            tf2::Quaternion quaternion;
            quaternion.setRPY( 0, 0, yaw );
            pose.orientation = tf2::toMsg(quaternion);
            break;
        }
    }
    
    return pose;

    
}

//Gnss Callback
void cb_gnss(const sensor_msgs::NavSatFix gnss_odom) {
    pose = calc_position(gnss_odom);
    map.push_back(pose);
    pose.orientation = calc_orientation(map, pose, 0).orientation;
}

//GNSS State Callback
void cb_gnss_state(const piksi_rtk_msgs::ReceiverState_V2_4_1 gnss_state) {
    if (gnss_state.fix_mode != "FIXED_RTK"){
        fixed_rtk = false;
        return;
    }
    fixed_rtk = true;
}

//Madgwick Callback
void cb_imu_madgwick(const sensor_msgs::Imu imu_madgwick) {
    /*
    TODO
*/
}

//IMU callback, calulate way
void cb_imu(const sensor_msgs::Imu imu_odom) {

    //calculate dt
    static ros::Time stamp;
    static ros::Time last_imu_stamp = stamp;
    stamp = imu_odom.header.stamp;
    double dt = (stamp - last_imu_stamp).toSec();
    last_imu_stamp = stamp;

    //calculate way
    v_y = v_y + imu_odom.linear_acceleration.y * dt;
    v_x = v_x + imu_odom.linear_acceleration.x * dt;
    v_z = v_z + imu_odom.linear_acceleration.z * dt;
    
    double v_abs = std::sqrt(v_y*v_y + v_x*v_x + v_z*v_z);
     
    imu_way = imu_way + v_abs * dt;

}

//Velocity callback, calulate way
void cb_twist(const plc_tcp_server::TramStatus tram) {

    //calcuöate dt
    static ros::Time stamp;
    static ros::Time last_velocity_stamp = stamp;
    stamp = tram.header.stamp;
    double dt = (stamp - last_velocity_stamp).toSec();
    last_velocity_stamp = stamp;

    //calculate way
    velocity_way = velocity_way + tram.velocity * dt;

}


//Publish Messages and broadcast the transform
int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;

    odom.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    
    ros::Subscriber odom_twist_sub = nh.subscribe("/tram_velocity", 10, cb_twist);
    ros::Subscriber odom_gnss_sub = nh.subscribe("/piksi/navsatfix_best_fix", 10, cb_gnss);
    ros::Subscriber odom_imu_sub = nh.subscribe("/piksi/imu", 10, cb_imu);
    ros::Subscriber odom_imu_madgwick_sub = nh.subscribe("/imu/madgwick", 10, cb_imu_madgwick);
    maas_odom_pub = nh.advertise<odometry::MaasOdometry>("/odom/debug", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();

        //entscheiden

        //Publish Transform
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(
                tf::Vector3(pose.position.x, pose.position.y,
                            pose.position.z));
        transform.setRotation(tf::Quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w));
        ros::Time tfTime;
        tfTime.fromSec(odom.header.stamp.toSec());
        br.sendTransform(tf::StampedTransform(transform, tfTime, "odom", "base_link"));
  
        maas_odom_pub.publish(maas_odom);
        odom_pub.publish(odom);

        loop_rate.sleep();
    }
    

    return 0;
}

