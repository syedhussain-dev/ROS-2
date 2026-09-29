#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from math import cos, sin, sqrt, pi
from numpy import zeros, array, transpose, identity, block
from numpy.linalg import inv, norm
from rclpy.node import Node
from scipy.spatial.transform import Rotation

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")
        self.get_logger().info("Using Python Implementation of Kalman Filter")
        self.declare_parameter("a",6378137.0)
        self.declare_parameter("1/f",298.257223563)
        self.declare_parameter("stddev_acc",1.7e-2)
        self.declare_parameter("stddev_lat",1.5)
        self.declare_parameter("stddev_long",1.5)
        self.declare_parameter("stddev_alt",3.0)

        self.a = self.get_parameter("a").get_parameter_value().double_value
        self.inv_f = self.get_parameter("1/f").get_parameter_value().double_value
        f = 1 / self.inv_f
        self.e2 = 2*f - f**2
        self.stddev_lat = self.get_parameter("stddev_lat").get_parameter_value().double_value
        self.stddev_long = self.get_parameter("stddev_long").get_parameter_value().double_value
        self.stddev_alt = self.get_parameter("stddev_alt").get_parameter_value().double_value
        self.stddev_acc = self.get_parameter("stddev_acc").get_parameter_value().double_value

        self.prev_vel_ = zeros([3])
        self.prev_r_ = zeros([3])
        self.linear_acc = zeros([3])
        self.pred_vel_ = zeros([3])
        self.pred_r_ = zeros([3])
        self.quaternion = zeros([4])
        self.ang_vel = zeros([3])

        self.R_ = zeros([3,3])

        self.dt = 0.01
        self.first_gps_reading = True

        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.N = 0.0
        self.T_ = zeros([3,3])
        self.enu = zeros([3])

        self.state_covar = zeros([6,6])
        self.pred_state_covar = zeros([6,6])

        self.H = array(
            [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]]
        )
        F1d = array([[1.0, self.dt],[ 0.0, 1.0]])
        Z = array([[0.0,0.0],[0.0,0.0]])
        self.F = block([
            [F1d, Z, Z],
            [Z, F1d, Z],
            [Z, Z, F1d]
        ])
        Q1_state = array([[(self.dt**4)/4,(self.dt**3)/2],[(self.dt**3)/2,self.dt**2]]) * self.stddev_acc ** 2
        self.Q = block([
            [Q1_state, Z, Z],
            [Z, Q1_state, Z],
            [Z, Z, Q1_state]
        ])
        self.B = array([
            [0.5*self.dt**2,0.0,0.0],
            [self.dt,0.0,0.0],
            [0.0,0.5*self.dt**2,0.0],
            [0.0,self.dt,0.0],
            [0.0,0.0,0.5*self.dt**2],
            [0.0,0.0,self.dt]
        ])
        
        self.R = array(
            [[self.stddev_lat**2, 0.0, 0.0],
            [0.0, self.stddev_long**2, 0.0],
            [0.0, 0.0, self.stddev_alt**2]]
        )

        self.X = zeros([6])
        self.X_pred = zeros([6])
        
        self.imu_sub_ = self.create_subscription(Imu,"/imu/out",self.imuCallback,10)
        self.gps_sub_ = self.create_subscription(NavSatFix,"/gps/out",self.gpsCallback,10)
        self.odom_pub_ = self.create_publisher(Odometry, "/mellinger_controller/odom", 10)

        self.msg = Odometry()
        self.timer = self.create_timer(self.dt, self.controlLoop)

    def controlLoop(self):
        self.measurementUpdate()
        self.msg.pose.pose.orientation.x = self.quaternion[0]
        self.msg.pose.pose.orientation.y = self.quaternion[1]
        self.msg.pose.pose.orientation.z = self.quaternion[2]
        self.msg.pose.pose.orientation.w = self.quaternion[3]
        self.msg.twist.twist.angular.x = self.ang_vel[0]
        self.msg.twist.twist.angular.y = self.ang_vel[1]
        self.msg.twist.twist.angular.z = self.ang_vel[2]

        self.msg.pose.pose.position.x = self.X[0]
        self.msg.twist.twist.linear.x = self.X[1]
        self.msg.pose.pose.position.y = self.X[2]
        self.msg.twist.twist.linear.y = self.X[3]
        self.msg.pose.pose.position.z = self.X[4]
        self.msg.twist.twist.linear.z = self.X[5]
        
        self.odom_pub_.publish(self.msg)

    def imuCallback(self, msg:Imu):
        self.linear_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        self.quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        quaternion_norm = norm(self.quaternion)
        if quaternion_norm != 0.0:
            self.R_ = Rotation.from_quat(self.quaternion).as_matrix()

    def gpsCallback(self, msg:NavSatFix):
        lat = d_to_r(msg.latitude)
        long = d_to_r(msg.longitude)
        alt = msg.altitude
        self.N = self.a / sqrt(1 - self.e2 * sin(lat)**2)
        if self.first_gps_reading:
            self.T_ = [[-sin(long),cos(long),0.0],
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

        self.enu = self.T_ @ array([delta_x, delta_y, delta_z])
        
    def measurementUpdate(self):
        self.statePrediction()
        y = self.enu - self.H @ self.X_pred
        K = self.pred_state_covar @ transpose(self.H) @ inv(self.H @ self.pred_state_covar @ transpose(self.H) + self.R)
        self.X = self.X_pred + K @ y
        self.state_covar = (identity(6) - K @ self.H) @ self.pred_state_covar

    def statePrediction(self):
        odom_acc = self.R_ @ self.linear_acc
        odom_acc[2] -= 9.81
        self.X_pred = self.F @ self.X + self.B @ odom_acc
        self.pred_state_covar = self.F @ self.state_covar @ transpose(self.F) + self.Q

def d_to_r(value:float): # Degrees to radians
    value *= (pi / 180)
    return value

def main():
    rclpy.init()
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
