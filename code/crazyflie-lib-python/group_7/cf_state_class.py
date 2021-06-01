from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig

class Estimate:
    EST_X = 'stateEstimate.x'
    EST_Y = 'stateEstimate.y'
    EST_Z = 'stateEstimate.z'
    EST_YAW = 'stabilizer.yaw'
    BATTERY = 'pm.vbat'

    def __init__(self, crazyflie, rate_ms=20, zranger=False):
        if isinstance(crazyflie, SyncCrazyflie):
            self._cf = crazyflie.cf
        else:
            self._cf = crazyflie
        self._log_config = self._create_log_config(rate_ms)
        self._est_x = None
        self._est_y = None
        self._est_z = None
        self._est_yaw = None
        # battery value
        self._battery_volt = None

    def _create_log_config(self, rate_ms):
        log_config = LogConfig('estimation', rate_ms)
        log_config.add_variable(self.EST_X)
        log_config.add_variable(self.EST_Y)
        log_config.add_variable(self.EST_Z)
        log_config.add_variable(self.EST_YAW)
        # battery value
        log_config.add_variable(self.BATTERY)
        log_config.data_received_cb.add_callback(self._data_received)
        return log_config

    def start(self):
        self._cf.log.add_config(self._log_config)
        self._log_config.start()

    def _data_received(self, timestamp, data, logconf):
        self._est_x = data[self.EST_X]
        self._est_y = data[self.EST_Y]
        self._est_z = data[self.EST_Z]
        self._est_yaw = data[self.EST_YAW]
        # battery value
        self._battery_volt = data[self.BATTERY]

    def stop(self):
        self._log_config.delete()

    @property
    def est_x(self):
        return self._est_x

    @property
    def est_y(self):
        return self._est_y

    @property
    def est_z(self):
        return self._est_z

    @property
    def est_yaw(self):
        return self._est_yaw

    @property
    def battery_volt(self):
        return self._battery_volt

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
