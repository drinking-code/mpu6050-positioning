from time import time, sleep
import threading

from mpu6050 import mpu6050
import numpy as np

from calibration import calibrate_rotation


def accel_data_to_list(accel_data):
    return [accel_data.get('x'), accel_data.get('y'), accel_data.get('z')]


def listen_to_accel(port: int = 0x68) -> tuple[dict, callable]:
    sensor = mpu6050(port)
    gravity = [0, 0, sensor.GRAVITIY_MS2 * -1]

    def get_accel_data_as_list():
        accel_data = sensor.get_accel_data()
        return accel_data_to_list(accel_data)

    calibration = calibrate_rotation(sensor)

    data = {
        'velocity': [0, 0, 0],
        'position': [0, 0, 0],
    }
    data_channel = {'interrupt': False}

    def update_data():
        timestamp = time()

        while not data_channel['interrupt']:
            new_timestamp = time()
            dirty_accelerometer_data = get_accel_data_as_list()
            # rotate vector according to calibration
            accelerometer_data = calibration.dot(dirty_accelerometer_data)

            # subtract gravity
            acceleration = np.subtract(accelerometer_data, gravity)

            time_delta_from_last_reading = new_timestamp - timestamp

            # probably wrong ):
            data['velocity'] = np.multiply(acceleration, time_delta_from_last_reading)
            # https://www.researchgate.net/post/Distance_position_from_accelerometer_MPU6050
            displacement = np.multiply(np.multiply(0.5, acceleration), time_delta_from_last_reading ** 2)
            data['position'] = np.add(data['position'], displacement)

            # print(np.round(acceleration, 3))
            # print(np.round(data['velocity'], 3))
            # print(np.round(data['position'], 3))

            timestamp = new_timestamp

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
