import curses
import rclpy

from curses_runner import run_curses_nodes

from tof_node import TimeOfFlightNode, extract_args
from simple_tof_runner import SimpleTofNode

def main(stdscr):
    values = {'--mode': 1, '--delay': 0.1, '--max-object-distance': None}
    extract_args(values)

    rclpy.init()
    tof_node = TimeOfFlightNode(values['--robot'], values['--max-object-distance'], values['--mode'], values['--delay'])
    simple_tof_node = SimpleTofNode(values['--robot'])
    run_curses_nodes(stdscr, [tof_node, simple_tof_node])
    rclpy.shutdown()


if __name__ == '__main__':
    curses.wrapper(main)