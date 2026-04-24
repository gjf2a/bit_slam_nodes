#!/usr/bin/python3

# Documentation: https://qwiic-vl53l1x-py.readthedocs.io/en/latest/apiref.html

import qwiic_vl53l1x
import time
import sys

def tof_loop(num_readings: int, distance_mode: int, delay: float):
    tof = qwiic_vl53l1x.QwiicVL53L1X()
    tof.sensor_init()
    tof.set_distance_mode(distance_mode)
    tof.start_ranging()
    time.sleep(delay)
    for i in range(num_readings):
        if tof.check_for_data_ready():
            print(f"{tof.get_distance():5}mm; status {tof.get_range_status()}; {i+1}/{num_readings}")
            tof.clear_interrupt()
        else:
            print("Not ready")
        time.sleep(delay)
    tof.stop_ranging()


def args(values: dict[str,any]):
    for arg in sys.argv[1:]:
        parts = arg.split('=')
        if parts[0] == 'num_readings':
            values[parts[0]] = int(parts[1])
        elif parts[0] == 'mode':
            values[parts[0]] = 1 if parts[1] == 'short' else 2
        elif parts[0] == 'delay':
            values[parts[0]] = float(parts[1])



if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Usage: tof.py [num_readings=100] [mode=(short|long)] [delay=0.1s]")
    else:
        values = {'num_readings': 100, 'mode': 1, 'delay': 0.1}
        args(values)
        tof_loop(values['num_readings'], values['mode'], values['delay'])
