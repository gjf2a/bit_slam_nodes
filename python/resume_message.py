import sys
import send_message

if __name__ == '__main__':
    robot = sys.argv[1] if len(sys.argv) > 1 else ""
    send_message.post_msg(robot, "StopNode", f"{robot}/bitslam_explorer_stop", "resume")
