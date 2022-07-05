import numpy as np


def accel_data_to_list(accel_data):
    return [accel_data.get('x'), accel_data.get('y'), accel_data.get('z')]


# calibrate: get offset (rotation matrix) of current reading from stand still (gravity)
# leave robot still on a level plane for calibration
def calibrate_rotation(sensor, calibration_samples_amount=300):
    gravity = [0, 0, sensor.GRAVITIY_MS2 * -1]
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

    calibration_samples = {'x': [], 'y': [], 'z': []}
    for i in range(0, calibration_samples_amount):
        sensor_data = sensor.get_accel_data()
        for ax in ['x', 'y', 'z']:
            calibration_samples.get(ax).append(sensor_data.get(ax))
    calibration_sample = {
        'x': np.average(calibration_samples.get('x')),
        'y': np.average(calibration_samples.get('y')),
        'z': np.average(calibration_samples.get('z')),
    }
    calibration_sample = accel_data_to_list(calibration_sample)
    return rotation_matrix_from_vectors(calibration_sample, calibration_ground_truth)
