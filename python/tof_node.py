#!/usr/bin/python3

# Documentation: https://qwiic-vl53l1x-py.readthedocs.io/en/latest/apiref.html

import qwiic_vl53l1x
import sys, math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor


class TimeOfFlightNode(Node):
    def __init__(self, robot_name: str, max_object_distance: int, distance_mode: int, delay: float):
        super().__init__(f"{robot_name}_TimeOfFlightNode")
        self.max_object_distance = max_object_distance
        tof_topic = f"{robot_name}_bitslam_obstacles"
        self.pub = self.create_publisher(String, tof_topic, qos_profile_sensor_data)
        print(f"Publishing on {tof_topic}")
        self.timer = self.create_timer(delay, self.timer_callback)
        self.tof = qwiic_vl53l1x.QwiicVL53L1X()
        self.tof.sensor_init()
        self.tof.set_distance_mode(distance_mode)
        self.tof.start_ranging()

    def timer_callback(self):
        if self.tof.check_for_data_ready():
            distance = self.tof.get_distance()
            if distance > 0 and (self.max_object_distance is None or distance < self.max_object_distance):
                output = String()
                output.data = f"(object,{distance/1000},0.0,0.05,{math.pi / 40})"
                self.pub.publish(output)
            self.tof.clear_interrupt()


def extract_args(values: dict[str,any]):
    for arg in sys.argv[1:]:
        parts = arg.split('=')
        if parts[0] == 'mode':
            values[parts[0]] = 1 if parts[1] == 'short' else 2
        elif parts[0] == 'delay':
            values[parts[0]] = float(parts[1])
        elif parts[0] == 'max-object-distance':
            values[parts[0]] = int(parts[1])
        elif parts[0] == 'robot':
            values[parts[0]] = parts[1]



if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Usage: tof_node.py robot=robotname [mode=(short|long)] [delay=0.1s] [max-object-distance=mm]")
    else:
        values = {'mode': 1, 'delay': 0.1, 'max-object-distance': None}
        extract_args(values)
        rclpy.init()
        executor = MultiThreadedExecutor()
        executor.add_node(TimeOfFlightNode(values['robot'], values['max-object-distance'], values['mode'], values['delay']))

        while True:
            executor.spin_once()
