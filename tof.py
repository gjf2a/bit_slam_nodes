#!/usr/bin/python3

import qwiic_vl53l1x
import time
import sys

def tof_loop(num_readings: int, delay):
    tof = qwiic_vl53l1x.QwiicVL53L1X()
    init = tof.sensor_init()
    #if init != 0:
    #    print(f"Failed to init with code {init}")
    #    sys.exit(init)
    tof.set_distance_mode(1)
    tof.start_ranging()
    time.sleep(0.1)
    for _ in range(num_readings):
        if tof.check_for_data_ready():
            print(tof.get_distance(), tof.get_range_status())
            tof.clear_interrupt()
        else:
            print("Not ready")
        time.sleep(delay)
    tof.stop_ranging()


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Usage: tof num_readings [delay=0.1]")
    else:
        num_readings = int(sys.argv[1])
        delay = sys.argv[2] if len(sys.argv) >= 3 else 0.1
        tof_loop(num_readings, delay)
