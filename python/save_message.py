import sys
import single_message

def extract_args() -> dict[str,any]: 
    values = {}
    for arg in sys.argv[1:]:
        parts = arg.split('=')
        if parts[0] == '--robot':
            values[parts[0]] = parts[1]
        elif parts[0] == '--at':
            values[parts[0]] = parts[1]
    return values


if __name__ == '__main__':
    args = extract_args()
    robot = args.get("--robot", "")
    if "--at" in args:
        msg = f"at {args['--at']}"
    else:
        msg = "save"
    single_message.post_msg(robot, "SaveNode", f"{robot}/save_bitslam", msg)
