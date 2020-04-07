# include <iostream>
# include <cmath>
# include "ros/ros.h"
# include "sensor_msgs/Imu.h"
# include "visualization_msgs/Marker.h"
# include <Eigen/Dense>
using namespace std;

ros::Publisher pub;
ros::Subscriber sub;

Eigen::Vector3d wb;
Eigen::Matrix3d c;
Eigen::Matrix3d b;

Eigen::Vector3d sg;
Eigen::Vector3d gg;
Eigen::Vector3d ab;
Eigen::Vector3d ag;
Eigen::Vector3d vg;

double last_time = 0;

std::vector<geometry_msgs::Point> points;

void imu_data_callBack(const sensor_msgs::Imu::ConstPtr& msg){
    if(last_time == (double)0){
        gg(0) = msg->linear_acceleration.x;
        gg(1) = msg->linear_acceleration.y;
        gg(2) = msg->linear_acceleration.z;
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

    wb(0) = msg->angular_velocity.x;
    wb(1) = msg->angular_velocity.y;
    wb(2) = msg->angular_velocity.z;
    // std::cout << "wb: \n" << wb << std::endl;
    ab(0) = msg->linear_acceleration.x;
    ab(1) = msg->linear_acceleration.y;
    ab(2) = msg->linear_acceleration.z;
    // std::cout << "ab: \n" << ab << std::endl;
    b(0, 0) = 0;
    b(0, 1) = -1*wb(2)*delta_t;
    b(0, 2) = wb(1)*delta_t;
    b(1, 0) = wb(2)*delta_t;
    b(1, 1) = 0;
    b(1, 2) = -1*wb(0)*delta_t; 
    b(2, 0) = -1*wb(1)*delta_t;
    b(2, 1) = wb(0)*delta_t;
    b(2, 2) = 0;
    // std::cout << "b: \n" << b << std::endl;
    double sigma = wb.norm() * delta_t;
    // std::cout << "sigma: \n" << sigma << std::endl;
    c = c * (Eigen::Matrix3d::Identity() + b*(sin(sigma)/sigma) + b*b*(((double)1-cos(sigma))/(sigma*sigma)));
    // std::cout << "c: \n" << c << std::endl;
    ag = c * ab;
    // std::cout << "ag: \n" << ag << std::endl;
    vg = vg + (ag-gg)*delta_t;
    // std::cout << "(ag-gg): \n" << (ag-gg) << std::endl;
    // std::cout << "vg: \n" << vg << std::endl;
    sg = sg + vg*delta_t;
    // std::cout << "vg*delta_t: \n" << vg*delta_t << std::endl;
    // std::cout << "sg: \n" << sg << std::endl;


    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/marker_frame";
    line_strip.header.stamp = ros::Time::now();
    static uint16_t id=0;
    line_strip.id = id++;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.x = 0;
    line_strip.pose.orientation.y = 0;
    line_strip.pose.orientation.z = 0;
    line_strip.pose.orientation.w = 1;
    geometry_msgs::Point p;
    p.x = sg(0);
    p.y = sg(1);
    p.z = sg(2);
    points.push_back(p);
    line_strip.points = points;

    if(points.size() > 1) pub.publish(line_strip);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_data_process");
    ros::NodeHandle n;
    pub = n.advertise<visualization_msgs::Marker>("path_marker", 1);
    sub = n.subscribe("/imu/data", 1, imu_data_callBack);
    ros::spin();

    return 0;
}