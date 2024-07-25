import time
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Initial position and heading
start_position = (0.0, 0.0, 0.4, 0)  # x, y, z, yaw

# Fixed velocity (m/s)
velocity = 0.05  # adjust this value as needed

# Time interval (s)
dt = 1

# Control inputs (angular velocity in radians/s)
angular_velocity_sequence = [0.0,0.0,0.0,0.0,-math.pi/2,0.0,0.0,0.0,0.0,0.0,0.0]

# Number of steps for each control input
steps_per_control = 1

# Safety limits
max_change = 1.0  # maximum allowed change in x or y (m)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def run_sequence(scf, start_position, velocity, angular_velocity_sequence, dt, steps_per_control, max_change):
    cf = scf.cf
    commander = cf.high_level_commander

    # Take off to the initial height
    print('Taking off')
    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)  # Wait for the Crazyflie to take off

    x, y, z, yaw = start_position
    initial_x, initial_y = x, y

    for angular_velocity in angular_velocity_sequence:
        print('Setting angular velocity {}'.format(angular_velocity))
        for _ in range(steps_per_control):
            # Update position based on Dubins car dynamics
            x += velocity * math.cos(yaw) * dt
            y += velocity * math.sin(yaw) * dt
            yaw += angular_velocity * dt

            # Check if the total change exceeds the limit
            if abs(x - initial_x) > max_change or abs(y - initial_y) > max_change:
                print('Exceeded safety limit. Landing...')
                cf.commander.send_position_setpoint(x, y, 0, math.degrees(yaw))
                time.sleep(0.1)
                cf.commander.send_stop_setpoint()
                return

            cf.commander.send_position_setpoint(x, y, z, math.degrees(yaw))
            time.sleep(dt)

    # Land after completing the sequence
    print('Landing')
    commander.land(0.0, 0.5)
    time.sleep(3.0)  # Wait for the Crazyflie to land

    commander.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        run_sequence(scf, start_position, velocity, angular_velocity_sequence, dt, steps_per_control, max_change)
