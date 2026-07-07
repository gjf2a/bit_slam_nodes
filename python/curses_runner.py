from typing import List
import curses, traceback

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class CursesNode(Node):
    def __init__(self, print_topic: str, first_line: int, stdscr):
        super().__init__(f"CursesNode_{print_topic}")
        self.create_subscription(String, print_topic, self.callback, qos_profile_sensor_data)
        self.first_line = first_line
        self.stdscr = stdscr

    def callback(self, msg: String):
        lines = msg.data.split('\n')
        for i, line in enumerate(lines):
            self.stdscr.addstr(i + self.first_line, 0, line)


def run_curses_nodes(stdscr, nodes: List[Node]):
    executor = MultiThreadedExecutor()
    startup(stdscr, executor, nodes)
    run_loop(stdscr, executor)
    shutdown(stdscr, executor, nodes)


def startup(stdscr, executor: MultiThreadedExecutor, nodes: List[Node]):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    for n in nodes:
        executor.add_node(n)


def run_loop(stdscr, executor: MultiThreadedExecutor):
    while True:
        try:
            executor.spin_once()
            k = stdscr.getch()
            if k != -1 and chr(k) == 'q':
                return
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())


def shutdown(stdscr, executor: MultiThreadedExecutor, nodes: List[Node]):
    executor.shutdown()
    for n in nodes:
        n.destroy_node()   

    curses.nocbreak()
    curses.echo()
    stdscr.refresh()