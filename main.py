from time import time, sleep
import threading
import json

import numpy as np
from mpu6050 import mpu6050

from calibration import calibrate_or_get_from_file
from ton_lowpass_realtime import butter_signal

if __name__ == '__main__':
    np.set_printoptions(
        precision=3,
        sign=' ',
        suppress=True,
    )


def accel_data_to_list(accel_data):
    return [accel_data.get('x'), accel_data.get('y'), accel_data.get('z')]


def listen_to_accel(port: int = 0x68) -> tuple[dict, callable]:
    sensor = mpu6050(port)
    gravity = [0, 0, sensor.GRAVITIY_MS2 * -1]

    def get_accel_data_as_list():
        accel_data = sensor.get_accel_data()
        return accel_data_to_list(accel_data)

    calibration_rotation, calibration_offsets = calibrate_or_get_from_file(sensor)

    data = {
        'velocity': [0, 0, 0],
        'position': [0, 0, 0],
    }
    data_channel = {'interrupt': False}

    def update_data():
        timestamp = time()
        if __name__ == '__main__':
            elapsed = 0
            collected_data = {
                'timestamp': [],
                'acceleration': [],
                'velocity': [],
                'position': [],
            }
            print('go')

        while (not data_channel['interrupt']) and (__name__ == '__main__' and elapsed < 5):
            dirty_accelerometer_data = get_accel_data_as_list()

            accelerometer_data = dirty_accelerometer_data
            # rotate vector according to calibration
            accelerometer_data = calibration_rotation.dot(accelerometer_data)
            # remove sensor fault
            accelerometer_data = np.add(calibration_offsets, accelerometer_data)

            # subtract gravity
            acceleration = np.subtract(accelerometer_data, gravity)

            new_timestamp = time()
            time_delta_from_last_reading = new_timestamp - timestamp
            if __name__ == '__main__':
                elapsed += time_delta_from_last_reading
            timestamp = new_timestamp

            # smoothing
            acceleration = butter_signal(10.0, time_delta_from_last_reading, acceleration)

            # https://www.real-world-physics-problems.com/rectilinear-motion.html
            displacement_term_1 = np.multiply(np.multiply(0.5, acceleration), time_delta_from_last_reading ** 2)
            displacement_term_0 = np.multiply(data['velocity'], time_delta_from_last_reading)
            data['position'] = np.add(data['position'], displacement_term_0, displacement_term_1)

            velocity_delta = np.multiply(acceleration, time_delta_from_last_reading)
            data['velocity'] = np.add(data['velocity'], velocity_delta)

            if __name__ == '__main__':
                collected_data['timestamp'].append(elapsed)
                collected_data['acceleration'].append(acceleration)
                collected_data['velocity'].append(data['velocity'].tolist())
                collected_data['position'].append(data['position'].tolist())

        if __name__ == '__main__':
            with open('output.json', 'w') as outfile:
                outfile.write(json.dumps(collected_data))

    read_thread = threading.Thread(target=update_data)
    read_thread.start()

    def stop():
        # send interrupt signal
        data_channel['interrupt'] = True
        # wait for thread to terminate
        read_thread.join()

    return data, stop


if __name__ == '__main__':
    listen_to_accel()
