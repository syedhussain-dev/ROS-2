#!/usr/bin/env python3
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import rclpy
from math import cos, sin, sqrt, pi
from numpy import zeros, array
from rclpy.node import Node
from scipy.spatial.transform import Rotation

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")
        self.get_logger().info("Using Python Implementation of Kalman Filter")
        self.declare_parameter("a",6378137.0)
        self.declare_parameter("1/f",298.257223563)

        self.a = self.get_parameter("a").get_parameter_value().double_value
        self.f_1 = self.get_parameter("f").get_parameter_value().double_value
        self.e2 = 2*self.f - self.f**2

        self.prev_linear_acc_x = 0.0
        self.prev_linear_acc_y = 0.0
        self.prev_linear_acc_z = 0.0

        self.linear_acc_x = 0.0
        self.linear_acc_y = 0.0
        self.linear_acc_z = 0.0

        self.R_ = zeros([3,3])

        self.deltaTime = 0.01
        self.first_gps_reading = True

        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.N = 0.0
        self.B_ = zeros([3,3])
        self.enu = zeros([3])

        self.imu_sub_ = self.create_subscription(Imu,"/imu/out",self.imuCallback,10)
        self.gps_sub_ = self.create_subscription(NavSatFix,"/gps/out",self.gpsCallback,10)
        self.odom_pub_ = self.create_publisher(Odometry, "mellinger_controller/odom", 10)


    def imuCallback(self, msg:Imu):
        self.linear_acc_x = msg.linear_acceleration.x
        self.linear_acc_y = msg.linear_acceleration.y
        self.linear_acc_z = msg.linear_acceleration.z

        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.R_ = Rotation.from_quat(quat=quaternion).as_matrix()

    def gpsCallback(self, msg:NavSatFix):
        lat = self.d_to_r(msg.latitude)
        long = self.d_to_r(msg.longitude)
        alt = msg.altitude
        self.N = self.a / sqrt(1 - self.e2 * sin(lat)**2)
        if self.first_gps_reading:
            self.B_ = [[-sin(long),cos(long),0.0],
                       [-sin(lat)*cos(long),-sin(lat)*sin(long),cos(lat)],
                       [cos(lat)*cos(long),cos(lat)*sin(long),sin(lat)]]
            self.x0 = (self.N + alt)*cos(lat)*cos(long)
            self.y0 = (self.N + alt)*cos(lat)*sin(long)
            self.z0 = (self.N *(1 - self.e2) + alt)*sin(lat)
            self.first_gps_reading = False

        x = (self.N + alt)*cos(lat)*cos(long)
        y = (self.N + alt)*cos(lat)*sin(long)
        z = (self.N *(1 - self.e2) + alt)*sin(lat)
        delta_x = x - self.x0
        delta_y = y - self.y0
        delta_z = z - self.z0

        self.enu = self.B_ @ array([delta_x, delta_y, delta_z])
        
    def measurementUpdate():
        pass

    def statePrediction():
        pass

    def d_to_r(value:float): # Degrees to radians
        value * pi / 180
        return value

def main():
    rclpy.init()
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
