import curses
import rclpy

from curses_runner import run_curses_nodes, CursesPrintNode

from tof_node import TimeOfFlightNode, extract_args
from simple_tof_runner import SimpleTofNode

def main(stdscr):
    values = {'--mode': 1, '--delay': 0.1, '--max-object-distance': None}
    extract_args(values)

    rclpy.init()
    tof_node = TimeOfFlightNode(values['--robot'], values['--max-object-distance'], values['--mode'], values['--delay'])
    simple_tof_node = SimpleTofNode(values['--robot'])
    tof_msg_node = CursesPrintNode(simple_tof_node.topic_name, 10, stdscr)
    run_curses_nodes(stdscr, [tof_node, simple_tof_node, tof_msg_node])
    rclpy.shutdown()


if __name__ == '__main__':
    curses.wrapper(main)