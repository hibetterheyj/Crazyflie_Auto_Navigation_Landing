# -*- coding: utf-8 -*-
import logging
import time
from threading import Timer
import datetime as dt

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
import os

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()

    if len(available) > 0:
        print('Crazyflies found!')
        for i in available:
            print(i[0])
    else:
        print('No Crazyflies found, cannot run example')
