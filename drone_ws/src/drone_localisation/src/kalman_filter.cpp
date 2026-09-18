#include "drone_localisation/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string& name) : Node(name){

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/mellinger_controller/odom",10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("mellinger_controller/odom",10,std::bind(&KalmanFilter::odomCallback,this,_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/out",10,std::bind(&KalmanFilter::imuCallback,this,_1))
}

KalmanFilter::odomCallback(const nav_msgs::msg::Odometry& odom){
    
}

KalmanFilter::imuCallback(const sensor_msgs::msg::Imu& imu){

}

KalmanFilter::statePrediction(){

}

KalmanFilter::measurementUpdate(){

}

int main(int argc, char* argv){
    rclcpp::init(argc, argv);
    // // auto node = std::make_shared<KalmanFilter>("kalman_filter");
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}