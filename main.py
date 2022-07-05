from time import time, sleep
import threading
import json

from mpu6050 import mpu6050
import numpy as np

from calibration import calibrate_rotation, calibrate_offsets

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

    calibration_rotation = calibrate_rotation(sensor)
    calibration_offsets = calibrate_offsets(sensor, 1000)

    data = {
        'velocity': [0, 0, 0],
        'position': [0, 0, 0],
    }
    data_channel = {'interrupt': False}

    def update_data():
        timestamp = time()
        smoothing_amount = 100
        last_readings = [[0, 0, 0] for x in range(0, smoothing_amount)]
        elapsed = 0
        collected_data = {
            'timestamp': [],
            'acceleration': [],
            'velocity': [],
            'position': [],
        }
        print('go')

        while (not data_channel['interrupt']) and elapsed < 5:
            dirty_accelerometer_data = get_accel_data_as_list()

            accelerometer_data = dirty_accelerometer_data
            # rotate vector according to calibration
            accelerometer_data = calibration_rotation.dot(accelerometer_data)
            # remove sensor fault
            accelerometer_data = np.add(calibration_offsets, accelerometer_data)

            # subtract gravity
            acceleration = np.subtract(accelerometer_data, gravity)

            # smoothing
            last_readings = last_readings[1:]
            last_readings.append(acceleration)

            acceleration = np.mean(last_readings, axis=0)

            new_timestamp = time()
            time_delta_from_last_reading = new_timestamp - timestamp
            elapsed += time_delta_from_last_reading
            timestamp = new_timestamp

            # https://www.real-world-physics-problems.com/rectilinear-motion.html
            displacement_term_1 = np.multiply(np.multiply(0.5, acceleration), time_delta_from_last_reading ** 2)
            displacement_term_0 = np.multiply(data['velocity'], time_delta_from_last_reading)
            data['position'] = np.add(data['position'], displacement_term_0, displacement_term_1)

            velocity_delta = np.multiply(acceleration, time_delta_from_last_reading)
            data['velocity'] = np.add(data['velocity'], velocity_delta)

            # print(acceleration)
            # print(data['velocity'])
            # print(data['position'])
            collected_data['timestamp'].append(elapsed)
            collected_data['acceleration'].append(acceleration.tolist())
            collected_data['velocity'].append(data['velocity'].tolist())
            collected_data['position'].append(data['position'].tolist())

        # print(collected_data)

        with open("output.json", "w") as outfile:
            outfile.write(json.dumps(collected_data))

    # update_data()

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
