#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from numpy import cross, dot, linalg, array, eye, zeros, transpose
from math import cos, sin, sqrt
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation


class MellingerController(Node):
    def __init__(self):
        super().__init__("mellinger_controller")

        self.declare_parameter("mass", 0.5)
        self.declare_parameter("g", 9.81)
        self.declare_parameter("L",0.12021)
        self.declare_parameter("kF", 3e-5)
        self.declare_parameter("kM", 1.1e-6)
        self.declare_parameter("Kv",9.35)
        self.declare_parameter("Kp",24.7)
        self.declare_parameter("Kr",2.5)
        self.declare_parameter("Kw",0.8)
        self.declare_parameter("show_logs",False)
        self.declare_parameter("logging_interval",50)

        self.mass_ = self.get_parameter("mass").get_parameter_value().double_value
        self.g_ = self.get_parameter("g").get_parameter_value().double_value
        self.L_ = self.get_parameter("L").get_parameter_value().double_value
        self.kF_ = self.get_parameter("kF").get_parameter_value().double_value
        self.kM_ = self.get_parameter("kM").get_parameter_value().double_value
        self.show_logs = self.get_parameter("show_logs").get_parameter_value().bool_value
        self.logging_interval = self.get_parameter("logging_interval").get_parameter_value().integer_value

        self.get_logger().info("Using a mass of %.2f"%self.mass_)
        self.get_logger().info("Using a gravitational field strength of %.2f"%self.g_)
        self.get_logger().info("Using a Length difference between CM and Rotors of %.2f"%self.L_)
        self.get_logger().info("Using a Thrust Coefficient of %.5f"%self.kF_)
        self.get_logger().info("Using a Drag Coefficient of %.5f"%self.kM_)

        # Gets the time right now in nanoseconds
        self.Kv_ = eye(3) * self.get_parameter("Kv").get_parameter_value().double_value
        # Controls the velocity correction, larger parameter means more correction
        self.Kp_ = eye(3) * self.get_parameter("Kp").get_parameter_value().double_value
        # Controls the position correction, larger parameter means more correction
        self.Kr_ = eye(3) * self.get_parameter("Kr").get_parameter_value().double_value
        # Controls the altitude correction, larger parameter means more correction
        self.Kw_ = eye(3) * self.get_parameter("Kw").get_parameter_value().double_value
        # Controls the angular velocity correction, larger parameter means more correction

        self.motor_pub_ = self.create_publisher(Actuators,"/quadcopter/simple_velocity_controller/commands",10)
        # Sends the angular velocities to Gazebo [s1, s2, s3, s4]
        self.odom_sub_ = self.create_subscription(Odometry, "/mellinger_controller/odom",self.odomCallback, 10)
        # Receives position, velocity, orientation and angular velocity
        self.velocity_sub_ = self.create_subscription(TwistStamped, "mellinger_controller/cmd_vel",self.velCallback, 10)
        # Sends the desired trajectory velocity and yaw

        self.M_ = array([
            [self.kF_,self.kF_,self.kF_,self.kF_],
            [0,-(self.kF_*self.L_),0,(self.kF_*self.L_)],
            [-(self.kF_*self.L_),0,(self.kF_*self.L_),0],
            [-self.kM_,self.kM_,-self.kM_,self.kM_]
        ])
        # Array matrix from (2)

        self.M_inv_ = linalg.inv(self.M_)
        # Inverse of matrix M

        self.r_ = zeros([3]) # Current position
        self.vel_ = zeros([3]) # Current velocity
        self.R_ = eye(3) # Current rotation matrix
        self.w_ = zeros([3]) # Current angular velocity

        self.vel_T_ = zeros([3]) # Desired velocity
        self.r_T_ = zeros([3]) # Desired position
        self.yaw_T_ = 0.0 # Desired yaw
        self.yaw_rate_T_ = 0.0# Desired yaw rate
        self.log_iterator = 0
        self.cmd_vel_received = False

        self.broadcaster_ = TransformBroadcaster(self)
        self.transform_stamp_ = TransformStamped()
        self.transform_stamp_.header.frame_id = "odom"
        self.transform_stamp_.child_frame_id = "root"

        self.get_logger().info("Python Implementation of Mellinger Controller Running...")
        self.timer_ = self.create_timer(0.01, self.controlLoop) # Timer callbacks take no msg arguement
        
    def controlLoop(self): # Function that executes the full mellinger pipeline
        if not self.cmd_vel_received: return
        self.log_iterator += 1
        if self.log_iterator % self.logging_interval == 0 and self.show_logs:
            self.get_logger().info(f"Yaw Rate T = {self.yaw_rate_T_}")
        self.yaw_T_ += self.yaw_rate_T_ * 0.01
        # Desired yaw
        
        i_B = self.R_[:,0]
        h = array([i_B[0], i_B[1], 0])
        h = h / linalg.norm(h)
        h_T = array([cos(self.yaw_T_),sin(self.yaw_T_),0])

        Fdes = -self.Kp_ @ (self.r_ - self.r_T_) - self.Kv_ @ (self.vel_ - self.vel_T_) + (array([0,0,self.mass_*self.g_]))
        Pdes = dot(Fdes,self.R_[:,2])
        Rdes = zeros((3,3))
        Rdes[:,2] = Fdes / linalg.norm(Fdes)
        Rdes[:,0] = cross(cross(array([0,0,1]),h_T),Rdes[:,2]) / linalg.norm(cross(cross(array([0,0,1]),h_T),Rdes[:,2]))
        Rdes[:,1] = cross(Rdes[:,2],Rdes[:,0])

        temp_arr = (transpose(Rdes) @ self.R_ - transpose(self.R_) @ Rdes)
        eR = 0.5 * array([temp_arr[2,1],temp_arr[0,2],temp_arr[1,0]])

        h_w = zeros(3)
        W_T = zeros(3)
        W_T[0] =  0.0 # dot(-h_w,self.r_T_[:,1])
        W_T[1] = 0.0 # dot(h_w,self.r_T_[:,0])
        W_T[2] = (W_T[1] * dot(Rdes[:,2], cross(array([0,0,1]), h_T)) + self.yaw_rate_T_ * dot(h_T, Rdes[:,0])) / dot(cross(array([0,0,1]), h_T), Rdes[:,1])        
        eW = self.w_ - W_T
        tdes = -self.Kr_ @ eR - self.Kw_ @ eW
        rotor_speed_sq = self.M_inv_ @ array([Pdes, tdes[0],tdes[1],tdes[2]])
        
        rotor_speeds = array([
            sqrt(max(0,rotor_speed_sq[0])),
            sqrt(max(0,rotor_speed_sq[1])),
            sqrt(max(0,rotor_speed_sq[2])),
            sqrt(max(0,rotor_speed_sq[3]))
        ])

        if self.log_iterator % self.logging_interval == 0 and self.show_logs:
            self.get_logger().info(f"R_ =\n{self.R_}")
            self.get_logger().info(f"eR = {eR}")
            self.get_logger().info(f"eW = {eW}")
            self.get_logger().info(f"Tdes = {tdes}")
            #self.get_logger().info(f"W_T = {W_T}")
            self.get_logger().info(f"r_T_[0] = {self.r_T_[0]}")
            self.get_logger().info(f"vel_T_[0] = {self.vel_T_[0]}")
            self.get_logger().info(f"Fdes[0] = {Fdes[0]}")
            #self.get_logger().info(f"Pdes = {Pdes}")
            self.get_logger().info(f"Rdes = {Rdes}")
            #self.get_logger().info(f"rotor_speed_sq = {rotor_speed_sq}")
            self.get_logger().info(f"rotor_speeds = {rotor_speeds}")

        msg = Actuators()
        rs = rotor_speeds.tolist()
        msg.velocity = [rs[0],rs[1],rs[2],rs[3]]
        self.motor_pub_.publish(msg)

    def odomCallback(self, msg:Odometry):
        self.r_ = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.vel_ = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w]
        self.R_ = Rotation.from_quat(quaternion).as_matrix()
        self.transform_stamp_.transform.rotation.x = quaternion[0]
        self.transform_stamp_.transform.rotation.y = quaternion[1]
        self.transform_stamp_.transform.rotation.z = quaternion[2]
        self.transform_stamp_.transform.rotation.w = quaternion[3]
        self.transform_stamp_.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster_.sendTransform(self.transform_stamp_)

    def velCallback(self, msg:TwistStamped):
        self.vel_T_ = array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        # Array of the desired velocities in the xyz axis' 
        self.yaw_rate_T_ = msg.twist.angular.z
        # Desired yaw rate
        if self.yaw_rate_T_ <= 0.0366 and self.yaw_rate_T_ >= -0.0366: 
            self.yaw_rate_T_ = 0.0
        self.cmd_vel_received = True


def main(args=None):
    rclpy.init(args=args)
    # Initialising rclpy
    node = MellingerController()
    # Creating a node from the class we made
    rclpy.spin(node=node)
    # Spin the node so it stays active
    node.destroy_node()
    # Destroy the node once during keyboard interruption
    rclpy.shutdown()
    # Shutdown rclpy when no nodes are active

if __name__ == "__main__":
    main()
    # If running this file in a terminal whatever function is here will run