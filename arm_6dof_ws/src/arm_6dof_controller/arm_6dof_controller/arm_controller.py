#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from numpy import array, cross, dot, linalg, identity, sin, cos, transpose, column_stack, zeros

class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller_node")

        self.declare_parameter("show_logs",False)
        self.declare_parameters(
            namespace="bottom_joint",
            parameters=[
                ("origin",[0.0,0.0,0.0]),
                ("axis",[0.0,0.0,0.0])
            ]
        )
        self.declare_parameters(
            namespace="shoulder_joint",
            parameters=[
                ("origin",[0.0,0.0,0.0]),
                ("axis",[0.0,0.0,0.0])
            ]
        )
        self.declare_parameters(
            namespace="elbow_joint",
            parameters=[
                ("origin",[0.0,0.0,0.0]),
                ("axis",[0.0,0.0,0.0])
            ]
        )
        self.declare_parameters(
            namespace="wrist_rotating_joint",
            parameters=[
                ("origin",[0.0,0.0,0.0]),
                ("axis",[0.0,0.0,0.0])
            ]
        )
        self.declare_parameters(
            namespace="wrist_deviating_joint",
            parameters=[
                ("origin",[0.0,0.0,0.0]),
                ("axis",[0.0,0.0,0.0])
            ]
        )
        self.declare_parameters(
            namespace="end_effector_joint",
            parameters=[
                ("origin",[0.0,0.0,0.0]),
                ("axis",[0.0,0.0,0.0])
            ]
        )

        self.p_bottom = self.get_parameter("bottom_joint.origin").value
        self.a_bottom = self.get_parameter("bottom_joint.axis").value
        self.p_shoulder = self.get_parameter("shoulder_joint.origin").value
        self.a_shoulder = self.get_parameter("shoulder_joint.axis").value
        self.p_elbow = self.get_parameter("elbow_joint.origin").value
        self.a_elbow = self.get_parameter("elbow_joint.axis").value
        self.p_wrist_rotating = self.get_parameter("wrist_rotating_joint.origin").value
        self.a_wrist_rotating = self.get_parameter("wrist_rotating_joint.axis").value
        self.p_wrist_deviating = self.get_parameter("wrist_deviating_joint.origin").value
        self.a_wrist_deviating = self.get_parameter("wrist_deviating_joint.axis").value
        self.p_end_effector = self.get_parameter("end_effector_joint.origin").value
        self.a_end_effector = self.get_parameter("end_effector_joint.axis").value

        self.qi = array([1,6])
        self.ai = [self.a_bottom,self.a_shoulder,self.a_elbow,self.a_wrist_rotating,self.a_wrist_deviating,
                   self.a_end_effector]
        self.pi = [self.p_bottom,self.p_shoulder,self.p_elbow,self.p_wrist_rotating,self.p_wrist_deviating,
                   self.p_end_effector]
        self.Ri = array([1,6])
        self.I = identity(3)

        self.names = ["bottom_joint","shoulder_joint","elbow_joint","wrist_rotating_joint",
                      "wrist_deviating_joint","end_effector_joint"]
        
        self.dt = 0.01

        self.current_pose = zeros([3])
        self.current_angles = zeros([3])
        self.target_pose = array([3])
        self.target_angles = array([3])

        self.jacobian = array([6,6])

        self.pose_sub_ = self.create_subscription(TwistStamped,"/arm_controller/cmd_vel",self.velCallback,10)
        self.joint_sub_ = self.create_subscription(JointState,"/arm_controller/joint_states",self.jointCallback,10)
        self.joint_pub_ = self.create_publisher(JointTrajectory,"/arm_controller/outputs",10)

        self.vel_received = False
        self.log_iterator = 0
        self.show_logs = self.get_parameter("show_logs").get_parameter_value().bool_value
        self.timer = self.create_timer(self.dt, self.controlLoop)
        self.msg = JointTrajectory()
        self.point = JointTrajectoryPoint()

    def controlLoop(self):
        if not self.vel_received: return

        self.Ri[:] = self.I*cos(self.qi[:]) + skew(self.ai[:])*sin(self.qi[:]) 
        + self.ai*transpose(self.ai)*(1-cos(self.qi[:]))

        self.ai[:] @= self.Ri[:]
        Ji = [[self.ai[:]*(self.target-self.pi[:])],[self.ai[:]]]
        self.jacobian = column_stack((Ji[0],Ji[1],Ji[2],Ji[3],Ji[4],Ji[5]))
        delta_x = array([
            self.target_pose[0] - self.current_pose[0],
            self.target_pose[1] - self.current_pose[1],
            self.target_pose[2] - self.current_pose[2],
            self.target_angles[0] - self.current_angles[0],
            self.target_angles[1] - self.current_angles[1],
            self.target_angles[2] - self.current_angles[2],
        ])
        delta_q = linalg.inv(self.jacobian) @ delta_x
        self.qi[:] += delta_q
        if self.show_logs and self.log_iterator % 30 == 0:
            self.get_logger().info(f"Current pose: {self.current_pose}")
            self.get_logger().info(f"Current angles: {self.current_angles}")
            self.get_logger().info(f"\nCurrent jacobian: {self.jacobian}")
            self.get_logger().info(f"qi: {self.qi}")
        self.point.positions = self.qi
        self.msg.points.append(self.point)
        self.joint_pub_.publish(self.msg)
        self.current_pose = self.target_pose
        self.current_angles = self.target_angles
        self.log_iterator += 1

    def jointCallback(self, msg:JointState):
        names_to_idx = {name:idx for name,idx in enumerate(msg.name)}
        for joint in self.names:
            if joint in names_to_idx:
                idx = names_to_idx[joint]
                self.qi.append(msg.position[idx])

    def velCallback(self, msg:TwistStamped):
        msg.header.frame_id = "base_link"
        self.target_pose += array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]) * self.dt
        self.target_angles
        self.vel_received = True

def skew(a):
    x, y, z = a
    return array(object=[
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0]
    ],dtype=float)

def main():
    rclpy.init()
    node = ArmController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()