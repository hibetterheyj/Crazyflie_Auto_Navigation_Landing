# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import logging
import time
from threading import Timer
import datetime as dt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import numpy as np
import os
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link id and disconnects after 5s.
    """

    def __init__(self, link_id):
        """ Initialize and run the example with the specified link_id """

        self.count = 0

        # Initialize cf object
        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Initialize log variable
        self.logs = np.zeros([100000,12])

        # Fly a square
        self.fly_square(link_id)

    def _connected(self, link_id):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('range.front', 'float')
        self._lg_stab.add_variable('range.back', 'float')
        self._lg_stab.add_variable('range.up', 'float')
        self._lg_stab.add_variable('range.left', 'float')
        self._lg_stab.add_variable('range.right', 'float')
        self._lg_stab.add_variable('range.zrange', 'float') # down
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('stateEstimate.x', 'float')  # estimated X coordinate
        self._lg_stab.add_variable('stateEstimate.y', 'float')  # estimated Y coordinate
        self._lg_stab.add_variable('stateEstimate.z', 'float')  # estimated Z coordinate

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def fly_square(self, id):
        """ Example of simple logico to make the drone fly in a square 
        trajectory at fixed speed"""

        # Sync with drone
        with SyncCrazyflie(id, cf=self._cf) as scf:
            # Send position commands
            with PositionHlCommander(scf) as pc:
                pc.up(1.0)
                # print('Moving up')
                pc.forward(1.0)
                # print('Moving forward')
                pc.left(1.0)
                # print('Moving left')
                pc.back(1.0)
                # print('Moving back')
                pc.right(1.0)
                # print('Moving right')

        self._disconnected

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        
        # Save info into log variable
        for idx, i in enumerate(list(data)):
            self.logs[self.count][idx] = data[i]

        self.count += 1

    def _connection_failed(self, link_id, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_id, msg))
        self.is_connected = False

    def _connection_lost(self, link_id, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_id, msg))

    def _disconnected(self, link_id):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_id)
        self.is_connected = False
        
        # Get timestamp
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(),'..','logs',filename)
        np.savetxt(filepath, self.logs, delimiter=',')


def connect_to_first_found():
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        return
    else:
        print('No Crazyflies found, cannot run example')

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # uncomment to connect to the first available drone
    # id = connect_to_first_found()
    # uncomment to connect to the selected drone
    id = 'radio://0/80/2M/E7E7E7E7E7'

    # Run example class
    if id is not None:
        le = LoggingExample(id)
    else: 
        print('Please provide a valid ID')

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
