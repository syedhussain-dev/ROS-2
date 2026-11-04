#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from numpy import array, linalg, identity, sin, cos, zeros, outer, cross

class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller_node")

        self.declare_parameter("show_logs",False)
        self.declare_parameter("bottom_joint.origin",[0.0, 0.0, 0.0])
        self.declare_parameter("bottom_joint.axis",[0.0, 0.0, 0.0])
        self.declare_parameter("shoulder_joint.origin",[0.0, 0.0, 0.0])
        self.declare_parameter("shoulder_joint.axis",[0.0, 0.0, 0.0])
        self.declare_parameter("elbow_joint.origin",[0.0, 0.0, 0.0])
        self.declare_parameter("elbow_joint.axis",[0.0, 0.0, 0.0])
        self.declare_parameter("wrist_rotating_joint.origin",[0.0, 0.0, 0.0])
        self.declare_parameter("wrist_rotating_joint.axis",[0.0, 0.0, 0.0])
        self.declare_parameter("wrist_deviating_joint.origin",[0.0, 0.0, 0.0])
        self.declare_parameter("wrist_deviating_joint.axis",[0.0, 0.0, 0.0])

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

        self.qi = zeros(shape=[5])
        self.ai = array([self.a_bottom,self.a_shoulder,self.a_elbow,self.a_wrist_rotating,self.a_wrist_deviating])
        self.pi = array([self.p_bottom,self.p_shoulder,self.p_elbow,self.p_wrist_rotating,self.p_wrist_deviating])
        self.Ri = zeros(shape=[5,3,3])
        self.I = identity(3)
        self.pe = zeros(shape=[3])

        self.names = ["bottom_joint","shoulder_joint","elbow_joint","wrist_rotating_joint",
                      "wrist_deviating_joint"]
        self.idxs = []
        
        self.dt = 0.01
        self.x_dot = zeros(shape=[6])
        self.end_effector_rot = 0.0

        self.jacobian = zeros(shape=[6,5])

        self.end_eff_rot_sub_ = self.create_subscription(Float64,"/arm_controller/end_eff_rot",self.rotCallback,10)
        self.vel_sub_ = self.create_subscription(TwistStamped,"/arm_controller/cmd_vel",self.velCallback,10)
        self.joint_sub_ = self.create_subscription(JointState,"/arm_controller/joint_states",self.jointCallback,10)
        self.joint_pub_ = self.create_publisher(JointTrajectory,"/arm_controller/outputs",10)

        self.vel_received = False
        self.joints_received = False
        self.log_iterator = 0
        self.show_logs = self.get_parameter("show_logs").get_parameter_value().bool_value
        self.timer = self.create_timer(self.dt, self.controlLoop)

    def controlLoop(self):
        if not self.vel_received: return
        p = zeros(shape=[3])
        R = self.I
        n = len(self.names)
        world_axis = zeros(shape=self.ai.shape)
        world_origins = zeros(shape=self.p.shape)
        for i in range(n):
            world_axis[i] = R @ self.ai[i]
            world_origins[i] = p.copy()
            self.Ri[i] = (
                self.I * cos(self.qi[i]) +
                skew(self.ai[i]) * sin(self.qi[i]) +
                outer(self.ai[i],self.ai[i] * (1-cos(self.qi[i])))
            )
            R @= self.Ri[i]
            p += R @ self.pi[i]
        self.pe = p
        for i in range(n):
            self.jacobian[0:3,i] = cross(
                world_axis[i],self.pe-world_origins[i]
            )
            self.jacobian[3:,i] = world_axis[i]
        q_dot = linalg.pinv(self.jacobian) @ self.x_dot
        q_target += self.qi + q_dot * self.dt
        if self.show_logs and self.log_iterator % 30 == 0:
            self.get_logger().info(f"x_dot: {self.x_dot}")
            self.get_logger().info(f"q_dot: {q_dot}")
            self.get_logger().info(f"\nCurrent jacobian: {self.jacobian}")
            self.get_logger().info(f"qi: {self.qi}")
        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        point.positions.append(self.end_effector_rot)
        msg = JointTrajectory()
        msg.joint_names = list(self.names)
        msg.joint_names.append("end_effector_joint")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.points = [point]
        self.joint_pub_.publish(msg)
        self.log_iterator += 1

    def jointCallback(self, msg:JointState):
        positions = dict(zip(msg.name,msg.position))
        if not all(name in positions for name in self.names): return
        self.qi = array(
            [positions[name] for name in self.names],
            dtype=float
        )
        self.joint_received = True
                
    def velCallback(self, msg:TwistStamped):
        msg.header.frame_id = "base_link"
        self.x_dot = array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,
                      msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z])
        self.vel_received = True

    def rotCallback(self, msg:Float64):
        self.end_effector_rot = msg.data

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