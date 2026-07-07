#!/usr/bin/python3

import random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector
from geometry_msgs.msg import TwistStamped

from tof_node import TimeOfFlightNode, extract_args

def ros2_string(s: str) -> String:
    output = String()
    output.data = s
    return output


class SimpleTofNode(Node):
    def __init__(self, robot_name: str, min_bump_timeout: int=5, max_bump_timeout: int=20, scan_min_m: float=0.5, ir_max: int=40):
        super().__init__(f"{robot_name}_SimpleTof")
        self.topic_name = f"{robot_name}/SimpleTof_msg"
        self.create_subscription(LaserScan, f"{robot_name}/scan", self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, f"{robot_name}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.create_subscription(IrIntensityVector, f"{robot_name}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.msg_pub = self.create_publisher(String, self.topic_name, qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_callback)
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.min_bump_timeout = min_bump_timeout
        self.max_bump_timeout = max_bump_timeout
        self.timeout_counts_left = 0
        self.hazard_counts = {"Scan": 0, "IR": 0, "Bump": 0}
        self.scan_min_m = scan_min_m
        self.ir_max = ir_max

    def found_hazard(self, label: str):
        self.timeout_counts_left = random.randint(self.min_bump_timeout, self.max_bump_timeout)
        self.hazard_counts[label] += 1
        hazard_str = f"Scan hazards: {self.hazard_counts['Scan']}\nIR hazards: {self.hazard_counts['IR']}\nBump hazards:{self.hazard_counts['Bump']}"
        self.msg_pub.publish(ros2_string(hazard_str))

    def timer_callback(self):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()

        if self.timeout_counts_left > 0:
            self.timeout_counts_left -= 1
            t.twist.angular.z = 0.5
        else:
            t.twist.linear.x = 0.5

        self.motors.publish(t)

    def scan_callback(self, scan: LaserScan):
        if scan.ranges[0] < self.scan_min_m:
            self.found_hazard("Scan")        

    def ir_callback(self, msg: IrIntensityVector):
        for reading in msg.readings:
            if reading.value > self.ir_max:
                self.found_hazard("IR")

    def bump_callback(self, bumps: HazardDetectionVector):
        for bump in bumps.detections:
            if 'bump' in bump.header.frame_id:
                self.found_hazard("Bump")


if __name__ == '__main__':
    values = {'--mode': 1, '--delay': 0.1, '--max-object-distance': None}
    extract_args(values)
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(TimeOfFlightNode(values['--robot'], values['--max-object-distance'], values['--mode'], values['--delay']))
    executor.add_node(SimpleTofNode(values['--robot']))
    
    while True:
        executor.spin_once()

