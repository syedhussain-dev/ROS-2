#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

class KalmanFilter : public rclcpp::Node{
private:
    rclcpp::create_publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::create_subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::create_subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void odomCallback(const nav_msg::msg::Odometry& odom);
    void imuCallback(const sensor_msgs::msg::Imu& imu);

    statePrediction();
    measurementUpdate();

public:
    KalmanFilter(const std::string& name)
}

#endif