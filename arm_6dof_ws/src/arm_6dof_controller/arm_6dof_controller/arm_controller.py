#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from numpy import array, linalg, identity, sin, cos, zeros, outer, cross, all, append

class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")

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

        self.qi = zeros(shape=[6])
        self.ai = array([self.a_bottom,self.a_shoulder,self.a_elbow,self.a_wrist_rotating,self.a_wrist_deviating])
        self.pi = array([self.p_bottom,self.p_shoulder,self.p_elbow,self.p_wrist_rotating,self.p_wrist_deviating])
        self.Ri = zeros(shape=[5,3,3])
        self.q_target = zeros(shape=[6])
        self.I = identity(3)
        self.pe = zeros(shape=[3])

        self.names = ["bottom_joint","shoulder_joint","elbow_joint","wrist_rotating_joint",
                      "wrist_deviating_joint","end_effector_joint"]
        
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
        self.point = JointTrajectoryPoint()
        self.msg = JointTrajectory()
        self.timer = self.create_timer(self.dt, self.controlLoop)

    def controlLoop(self):
        if not self.vel_received or not self.joints_received: return
        p = zeros(shape=[3])
        R = self.I.copy()
        n = len(self.names)-1 
        # End effector joint is not included in forward kinematics.
        # Manual control of the joint is wanted.
        world_axis = zeros((n,3))
        world_origins = zeros((n,3))
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
        q_dot = append(q_dot,self.end_effector_rot)
        self.q_target[:5] += q_dot[:5] * self.dt
        self.q_target[5] += q_dot[5] * self.dt
        if self.show_logs and self.log_iterator % 30 == 0:
            self.get_logger().info(f"x_dot: {self.x_dot}")
            self.get_logger().info(f"q_dot: {q_dot}")
            self.get_logger().info(f"\nCurrent jacobian: {self.jacobian}")
            self.get_logger().info(f"qi: {self.qi}")
            self.get_logger().info(f"q_target: {self.q_target}")
        self.point.positions = self.q_target.tolist()
        self.point.time_from_start = Duration(
            seconds=0.05
        ).to_msg()
        self.msg.joint_names = list(self.names)
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.points = [self.point]
        self.joint_pub_.publish(self.msg)
        self.log_iterator += 1

    def jointCallback(self, msg:JointState):
        positions = dict(zip(msg.name,msg.position))
        if not all(name in positions for name in self.names): return
        self.qi = array(
            [positions[name] for name in self.names],
            dtype=float
        )
        if not self.joints_received:
            self.q_target = self.qi.copy()
        self.joints_received = True
                
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