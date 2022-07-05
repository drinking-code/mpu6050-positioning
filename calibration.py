import numpy as np


def accel_data_to_list(accel_data):
    return [accel_data.get('x'), accel_data.get('y'), accel_data.get('z')]


def get_n_samples(sensor, n):
    samples = []
    for i in range(0, n):
        sensor_data = sensor.get_accel_data()
        samples.append(accel_data_to_list(sensor_data))
    return np.array(samples)


gravity = np.array([])
rotation_calibration_result = np.array([])
offsets_calibration_result = np.array([])


# calibrate: get offset (rotation matrix) of current reading from stand still (gravity)
# leave robot still on a level plane for calibration
# calibrated_data = calibration_result.dot(accelerometer_data)
def calibrate_rotation(sensor, calibration_samples_amount=300):
    global gravity
    gravity = np.array([0, 0, sensor.GRAVITIY_MS2 * -1])
    calibration_ground_truth = gravity[:]

    def rotation_matrix_from_vectors(vec1, vec2):
        # https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
        """ Find the rotation matrix that aligns vec1 to vec2
        :param vec1: A 3d "source" vector
        :param vec2: A 3d "destination" vector
        :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        """
        a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rotation_matrix

    calibration_samples = get_n_samples(sensor, calibration_samples_amount)
    calibration_sample = np.mean(calibration_samples, axis=0)
    global rotation_calibration_result
    rotation_calibration_result = rotation_matrix_from_vectors(calibration_sample, calibration_ground_truth)
    return rotation_calibration_result


# calibrate: get offsets (sensor fault) of current reading from stand still (no acceleration)
# leave accelerometer still on a level plane for calibration
# calibrated_data = np.add(calibration_result, accelerometer_data)
def calibrate_offsets(sensor, calibration_samples_amount=500):
    global gravity
    calibration_ground_truth = np.array([0, 0, 0])
    calibration_samples = get_n_samples(sensor, calibration_samples_amount)
    # apply rotation correction
    calibration_samples = np.array(list(map(lambda a: rotation_calibration_result.dot(a), calibration_samples)))
    # average offsets
    calibration_sample = np.mean(calibration_samples, axis=0)
    calibration_sample = np.subtract(calibration_sample, gravity)
    global offsets_calibration_result
    offsets_calibration_result = np.subtract(calibration_ground_truth, calibration_sample)
    return offsets_calibration_result
