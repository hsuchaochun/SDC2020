# include <iostream>
# include <cmath>
# include "ros/ros.h"
# include "sensor_msgs/Imu.h"
# include "visualization_msgs/Marker.h"
# include "geometry_msgs/PoseWithCovarianceStamped.h"
# include "nav_msgs/Odometry.h"
# include <Eigen/Dense>
using namespace Eigen;

ros::Subscriber sub_imu;
ros::Subscriber sub_comOdom;
ros::Subscriber sub_zedOdom;
ros::Publisher pub_imu;
ros::Publisher pub_marker;

Eigen::Vector3d wb;
Eigen::Matrix3d c;
Eigen::Matrix3d b;

Eigen::Vector3d sg;
Eigen::Vector3d gg;
Eigen::Vector3d ab;
Eigen::Vector3d ag;
Eigen::Vector3d vg;

Eigen::Matrix3d imu2cam_trans;
Eigen::Matrix3d cam2zed_trans;

double last_time = 0;

std::vector<geometry_msgs::Point> imu_points;
std::vector<geometry_msgs::Point> combined_points;
std::vector<geometry_msgs::Point> zed_points;


void imuData2zedOdom(const sensor_msgs::Imu::ConstPtr& msg){

    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // ROS_INFO("Imu Angular velocity x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    // ROS_INFO("Imu Linear acceleration x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    Eigen::Quaterniond imu_orien_Q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d imu_angV(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Eigen::Vector3d imu_linA(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    
    Eigen::Matrix3d imu_orien_M(imu_orien_Q);

    // std::cout << "Imu orientation (Quaterniond): \n" << imu_orien_Q << std::endl;
    // std::cout << "Imu orientation (Matrix): \n" << imu_orien_M << std::endl;
    // std::cout << "Imu Angular velocity: \n" << imu_angV << std::endl;
    // std::cout << "Imu Linear acceleration: \n" << imu_linA << std::endl;
    // std::cout << "Rotation matrix from IMU's frame to Camera's frame:  \n" << imu2cam_trans << std::endl;
    // std::cout << "Rotation matrix from Camera's frame to ZED odometry's frame:  \n" << cam2zed_trans << std::endl;

    Eigen::Matrix3d zed_orien_M(cam2zed_trans * imu2cam_trans * imu_orien_M);
    Eigen::Vector3d zed_angV(cam2zed_trans * imu2cam_trans * imu_angV);
    Eigen::Vector3d zed_linA(cam2zed_trans * imu2cam_trans * imu_linA);

    Eigen::Quaterniond zed_orien_Q(zed_orien_M);

    // std::cout << "ZED orientation (Quaterniond): \n" << imu_orien_Q << std::endl;
    // std::cout << "ZED orientation (Matrix): \n" << imu_orien_M << std::endl;
    // std::cout << "ZED Angular velocity: \n" << imu_angV << std::endl;
    // std::cout << "ZED Linear acceleration: \n" << imu_linA << std::endl;
    
    sensor_msgs::Imu pub_msg;
    pub_msg.header = msg->header;
    pub_msg.orientation.w = zed_orien_Q.w();
    pub_msg.orientation.x = zed_orien_Q.x();
    pub_msg.orientation.y = zed_orien_Q.y();
    pub_msg.orientation.z = zed_orien_Q.z();
    pub_msg.angular_velocity.x = zed_angV(0);
    pub_msg.angular_velocity.y = zed_angV(1);
    pub_msg.angular_velocity.z = zed_angV(2);
    pub_msg.linear_acceleration.x = zed_linA(0);
    pub_msg.linear_acceleration.y = zed_linA(1);
    pub_msg.linear_acceleration.z = zed_linA(2);

    pub_imu.publish(pub_msg);
}

void imu_data_callBack(const sensor_msgs::Imu::ConstPtr& msg){

    imuData2zedOdom(msg);

    if(last_time == (double)0){
        gg << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        c = Eigen::Matrix3d::Identity();
        sg = Eigen::Vector3d::Zero();
        vg = Eigen::Vector3d::Zero();
        last_time = (double)(msg->header.stamp.toNSec())/1E9;
        return;
    }

    double delta_t = (double)(msg->header.stamp.toNSec())/1E9 - last_time;
    last_time = (double)(msg->header.stamp.toNSec())/1E9;

    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // ROS_INFO("Imu Angular velocity x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    // ROS_INFO("Imu Linear acceleration x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    wb << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    ab << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    b <<    0, -1*wb(2)*delta_t, wb(1)*delta_t,
            wb(2)*delta_t, 0, -1*wb(0)*delta_t, 
            -1*wb(1)*delta_t, wb(0)*delta_t, 0;
    // std::cout << "ab: \n" << ab << std::endl;
    // std::cout << "wb: \n" << wb << std::endl;
    // std::cout << "b: \n" << b << std::endl;
    
    double sigma = wb.norm() * delta_t;
    c = c * (Eigen::Matrix3d::Identity() + b*(sin(sigma)/sigma) + b*b*(((double)1-cos(sigma))/(sigma*sigma)));
    ag = c * ab;
    vg = vg + (ag-gg)*delta_t;
    sg = sg + vg*delta_t;
    // std::cout << "sigma: \n" << sigma << std::endl;
    // std::cout << "c: \n" << c << std::endl;
    // std::cout << "ag: \n" << ag << std::endl;
    // std::cout << "(ag-gg): \n" << (ag-gg) << std::endl;
    // std::cout << "vg: \n" << vg << std::endl;
    // std::cout << "vg*delta_t: \n" << vg*delta_t << std::endl;
    // std::cout << "sg: \n" << sg << std::endl;

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/marker_frame";
    line_strip.header.stamp = msg->header.stamp;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    static uint16_t id = 0;
    line_strip.id = id++;
    line_strip.scale.x = 0.1;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.x = 0;
    line_strip.pose.orientation.y = 0;
    line_strip.pose.orientation.z = 0;
    line_strip.pose.orientation.w = 1;

    geometry_msgs::Point p;
    Eigen::Vector3d sg_tf = cam2zed_trans * imu2cam_trans * sg;
    p.x = sg_tf(0);
    p.y = sg_tf(1);
    p.z = sg_tf(2);

    imu_points.push_back(p);
    line_strip.points = imu_points;

    if(imu_points.size() > 1) pub_marker.publish(line_strip);
    
}

void com_odom_callBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/marker_frame";
    line_strip.header.stamp = msg->header.stamp;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    static uint16_t id = 0;
    line_strip.id = id++;
    line_strip.scale.x = 0.1;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.x = 0;
    line_strip.pose.orientation.y = 0;
    line_strip.pose.orientation.z = 0;
    line_strip.pose.orientation.w = 1;

    geometry_msgs::Point p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;
    
    combined_points.push_back(p);
    line_strip.points = combined_points;

    if(combined_points.size() > 1) pub_marker.publish(line_strip);

}

void zed_odom_callBack(const nav_msgs::Odometry::ConstPtr& msg){

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/marker_frame";
    line_strip.header.stamp = msg->header.stamp;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    static uint16_t id = 0;
    line_strip.id = id++;
    line_strip.scale.x = 0.1;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.x = 0;
    line_strip.pose.orientation.y = 0;
    line_strip.pose.orientation.z = 0;
    line_strip.pose.orientation.w = 1;
    
    geometry_msgs::Point p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;
    
    zed_points.push_back(p);
    line_strip.points = zed_points;

    if(zed_points.size() > 1) pub_marker.publish(line_strip);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_data_process");
    ros::NodeHandle n;
    sub_imu = n.subscribe("/imu/data", 1, imu_data_callBack);
    sub_comOdom = n.subscribe("/robot_pose_ekf/odom_combined", 1, com_odom_callBack);
    sub_zedOdom = n.subscribe("/zed/odom", 1, zed_odom_callBack);
    pub_imu = n.advertise<sensor_msgs::Imu>("/imu_data", 1);
    pub_marker = n.advertise<visualization_msgs::Marker>("path_marker", 1);

    imu2cam_trans <<    0.0225226, 0.999745, 0.0017194, 
                        0.0648765, -0.00317777, 0.997888, 
                        0.997639, -0.0223635, -0.0649315;
    cam2zed_trans <<    0, 0, 1, 
                        -1, 0, 0, 
                        0, -1, 0;

    printf("initialize done\n");
    
    ros::spin();

    return 0;
}