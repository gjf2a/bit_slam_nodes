import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class MsgNode(Node):
    def __init__(self, robot: str, node_name: str, topic: str, msg: str):
        super(f"{robot}/{node_name}")
        self.pub = self.create_publisher(String, topic, qos_profile_sensor_data)
        self.msg = msg
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.posted = False

    def timer_callback(self):
        if not self.posted:
            output = String()
            output.data = self.msg
            self.pub.publish(output)
            self.posted = True


def post_msg(robot: str, node_name: str, topic: str, msg: str):
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(MsgNode(robot, node_name, topic, msg))
    executor.spin_once()
    rclpy.shutdown()