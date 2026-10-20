#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ClawController(Node):
    def __init__(self):
        super().__init__("claw_controller_node")

        self.declare_parameter("show_logs",True)
        self.show_logs = self.get_parameter("show_logs").get_parameter_value().bool_value

        self.dt = 0.01
        self.ang_vel = 0.0
        self.log_iterator = 0
        self.idx = 0
        self.cmd_received = False

        self.claw_pub_ = self.create_publisher(JointTrajectory,"/arm_controller/claw_output",10)
        self.claw_sub_ = self.create_subscription(JointState,"/arm_controller/claw_state",self.jointCallback,10)
        self.controller_sub_ = self.create_subscription(Float64,"/arm_controller/cmd_claw",self.clawCallback,10)

        self.timer = self.create_timer(self.dt,self.controlLoop)
        self.msg = JointTrajectory()
        self.point = JointTrajectoryPoint()

    def controlLoop(self):
        if not self.cmd_received: return
        self.q += self.ang_vel * self.dt
        if self.show_logs and self.log_iterator % 30 == 0:
            self.get_logger().info(f"ang_vel: {self.ang_vel}")
            self.get_logger().info(f"q: {self.q}")
        self.point.positions = self.q
        self.msg.points.append(self.point)
        self.claw_pub_.publish(self.msg)
        self.log_iterator += 1

    def jointCallback(self, msg:JointState):
        if self.log_iterator == 0:
            name_to_idx = {name:idx for name,idx in enumerate(msg.name)}
            self.idx = name_to_idx["claw1_joint"]
        self.q = msg.position[self.idx]

    def clawCallback(self, msg:Float64):
        self.ang_vel = msg.data
        self.cmd_received = True

def main():
    rclpy.init()
    node = ClawController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()