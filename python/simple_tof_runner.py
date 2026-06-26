#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector
from geometry_msgs.msg import TwistStamped

from tof_node import TimeOfFlightNode, extract_args

class SimpleTofNode(Node):
    def __init__(self, robot_name: str):
        super().__init__(f"{robot_name}_SimpleTof")
        self.create_subscription(LaserScan, f"{robot_name}/scan", self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, f"{robot_name}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.create_subscription(IrIntensityVector, f"{robot_name}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_callback)
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.bump_timeout_duration = 5
        self.timeout_counts_left = 0

    def found_hazard(self):
        self.timeout_counts_left = self.bump_timeout_duration

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
        if scan.ranges[0] < 0.5:
            print("Scan hazard")
            self.found_hazard()        

    def ir_callback(self, msg: IrIntensityVector):
        for reading in msg.readings:
            if reading.value > 40:
                print("IR hazard")
                self.found_hazard()

    def bump_callback(self, bumps: HazardDetectionVector):
        for bump in bumps.detections:
            if 'bump' in bump.header.frame_id:
                print("Bump hazard")
                self.found_hazard()


if __name__ == '__main__':
    values = {'--mode': 1, '--delay': 0.1, '--max-object-distance': None}
    extract_args(values)
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(TimeOfFlightNode(values['--robot'], values['--max-object-distance'], values['--mode'], values['--delay']))
    executor.add_node(SimpleTofNode(values['--robot']))
    
    while True:
        executor.spin_once()

