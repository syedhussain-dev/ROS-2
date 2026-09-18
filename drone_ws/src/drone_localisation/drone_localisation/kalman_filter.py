#!/usr/bin/env python3
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import ParameterType
from math import cos, sin, sqrt, pi
from numpy import zeros, array, transpose, identity
from numpy.linalg import inv
from rclpy.node import Node
from scipy.spatial.transform import Rotation

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")
        self.get_logger().info("Using Python Implementation of Kalman Filter")
        self.declare_parameter("a",6378137.0)
        self.declare_parameter("1/f",298.257223563)
        self.declare_parameter("stddev_acc",1.7e-2)

        self.a = self.get_parameter("a").get_parameter_value().double_value
        self.inv_f = self.get_parameter("1/f").get_parameter_value().double_value
        f = 1 / self.inv_f
        self.e2 = 2*f - f**2

        self.prev_vel_ = zeros([3])
        self.prev_r_ = zeros([3])
        self.linear_acc = zeros([3])
        self.pred_vel_ = zeros([3])
        self.pred_r_ = zeros([3])
        self.quaternion = zeros([4])
        self.ang_vel = zeros([3])

        self.R_ = zeros([3,3])

        self.deltaTime = 0.01
        self.first_gps_reading = True

        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.N = 0.0
        self.T_ = zeros([3,3])
        self.enu = zeros([3])

        self.state_covar = zeros([6,6])
        self.prev_state_covar = zeros([6,6])
        self.state_trans = array([
            [1, 0, 0, self.deltaTime, 0, 0],
            [0, 1, 0, 0, self.deltaTime, 0],
            [0, 0, 1, 0, 0, self.deltaTime],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        self.pose_matrix = array(
            [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]
        )
        self.vel_matrix = array(
            [[0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
        )
        self.gps_noise_covar = array(
            [[2.25, 0.0, 0.0],
            [0.0, 2.25, 0.0],
            [0.0, 0.0, 9.0]]
        )
        self.stddev_acc = self.get_parameter("stddev_acc").get_parameter_value().double_value
        self.var_acc = self.stddev_acc ** 2
        q00 = 0.25 * self.deltaTime**4 * self.var_acc
        q01 = 0.5 * self.deltaTime**3 * self.var_acc
        q11 = self.deltaTime**2 * self.var_acc

        self.process_noise_covar = array([
            [q00, 0,   0,   q01, 0,   0],
            [0,   q00, 0,   0,   q01, 0],
            [0,   0,   q00, 0,   0,   q01],
            [q01, 0,   0,   q11, 0,   0],
            [0,   q01, 0,   0,   q11, 0],
            [0,   0,   q01, 0,   0,   q11]
        ])

        self.imu_sub_ = self.create_subscription(Imu,"/imu/out",self.imuCallback,10)
        self.gps_sub_ = self.create_subscription(NavSatFix,"/gps/out",self.gpsCallback,10)
        self.odom_pub_ = self.create_publisher(Odometry, "mellinger_controller/odom", 10)

        self.msg = Odometry()
        self.timer = self.create_timer(0.01, self.controlLoop)

    def controlLoop(self):
        r_, vel_ = self.measurementUpdate()
        self.msg.pose.pose.position.x = r_[0]
        self.msg.pose.pose.position.y = r_[1]
        self.msg.pose.pose.position.z = r_[2]
        self.msg.pose.pose.orientation.x = self.quaternion[0]
        self.msg.pose.pose.orientation.y = self.quaternion[1]
        self.msg.pose.pose.orientation.z = self.quaternion[2]
        self.msg.pose.pose.orientation.w = self.quaternion[3]
        self.msg.twist.twist.linear.x = vel_[0]
        self.msg.twist.twist.linear.y = vel_[1]
        self.msg.twist.twist.linear.z = vel_[2]
        self.msg.twist.twist.angular.x = self.ang_vel[0]
        self.msg.twist.twist.angular.y = self.ang_vel[1]
        self.msg.twist.twist.angular.z = self.ang_vel[2]
        self.odom_pub_.publish(self.msg)

    def imuCallback(self, msg:Imu):
        self.linear_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.ang_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        self.quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.R_ = Rotation.from_quat(self.quaternion).as_matrix()

    def gpsCallback(self, msg:NavSatFix):
        lat = self.d_to_r(msg.latitude)
        long = self.d_to_r(msg.longitude)
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
        y_pose = self.enu - self.pose_matrix @ array([self.pred_r_, self.pred_vel_]).flatten()
        K_pose = self.state_covar @ transpose(self.pose_matrix) @ inv(self.pose_matrix @ self.state_covar @ transpose(self.pose_matrix) + self.gps_noise_covar)
        r_new = self.pred_r_ + (K_pose @ y_pose)[:3]
        vel_pred = self.vel_matrix @ array([self.pred_r_, self.pred_vel_]).flatten()
        self.state_covar = (identity(6) - K_pose @ self.pose_matrix) @ self.prev_state_covar
        self.prev_state_covar = self.state_covar
        self.prev_vel_ = vel_pred
        self.prev_r_ = r_new
        return r_new, vel_pred


    def statePrediction(self):
        odom_acc = self.R_ @ self.linear_acc
        odom_acc[2] -= 9.81
        for i in range(3):
            self.pred_vel_[i] = self.prev_vel_[i] + odom_acc[i] * self.deltaTime
            self.pred_r_[i] = self.prev_r_[i] + (self.pred_vel_[i] * self.deltaTime) + (0.5 * odom_acc[i] * self.deltaTime**2)
        self.state_covar = self.state_trans @ self.prev_state_covar @ transpose(self.state_trans) + self.process_noise_covar


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
