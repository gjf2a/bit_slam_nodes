import sys
import single_message

if __name__ == '__main__':
    robot = sys.argv[1] if len(sys.argv) > 1 else ""
    single_message.post_msg(robot, "StopNode", "bitslam_explorer_stop", "stop")