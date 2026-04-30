import subprocess
import sys
from tof_node import TimeOfFlightNode, extract_args

import rclpy
from rclpy.executors import MultiThreadedExecutor

if __name__ == '__main__':
    values = {'mode': 1, 'delay': 0.1, 'max-object-distance': None}
    extract_args(values)
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(TimeOfFlightNode(values['robot'], values['max-object-distance'], values['mode'], values['delay']))
    subprocess.Popen(["../target/release/bump_turn_runner", f"--robot={sys.argv[1]}"])

    while True:
        executor.spin_once()

