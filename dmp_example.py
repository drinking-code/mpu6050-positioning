from dmp import MPU6050
from time import time
import json

from ton_lowpass_realtime import butter_vector_dmp_signal

i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = -5489
y_accel_offset = -1441
z_accel_offset = 1305
x_gyro_offset = -2
y_gyro_offset = -72
z_gyro_offset = -5
enable_debug_output = True

mpu = MPU6050(i2c_bus, device_address,
              # x_accel_offset, y_accel_offset,
              # z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              # enable_debug_output
              )

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
print('Status: ' + hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
print('Packet size: ' + packet_size)
FIFO_count = mpu.get_FIFO_count()
print('FiFo count: ' + FIFO_count)

count = 0
FIFO_buffer = [0] * 64

FIFO_count_list = list()
timestamp = time()
collected_data = {
    'timestamp': [],
    'rotation': [],
}
elapsed = 0
good_to_go = False

while count < 2000:
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()

    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
        # print('overflow!')
    # Check if fifo data is ready
    elif mpu_int_status & 0x02:
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        linear_acceleration = mpu.DMP_get_linear_accel(accel, grav)

        new_timestamp = time()
        time_delta_from_last_reading = new_timestamp - timestamp
        elapsed += time_delta_from_last_reading
        timestamp = new_timestamp

        if elapsed > 20 and not good_to_go:
            good_to_go = True
            print('probably good to go')

        # eliminate spikes
        roll_pitch_yaw = butter_vector_dmp_signal(time_delta_from_last_reading, roll_pitch_yaw)

        collected_data['timestamp'].append(elapsed)
        collected_data['rotation'].append([roll_pitch_yaw.x, roll_pitch_yaw.y, roll_pitch_yaw.z])

        if count % 100 == 0:
            print('acceleration_X: ' + str(linear_acceleration.x))
            print('acceleration_Y: ' + str(linear_acceleration.y))
            print('acceleration_Z: ' + str(linear_acceleration.z))
            # print('roll: ' + str(roll_pitch_yaw.x))
            # print('pitch: ' + str(roll_pitch_yaw.y))
            # print('yaw: ' + str(roll_pitch_yaw.z))
        count += 1

with open('output_rotation.json', 'w') as outfile:
    outfile.write(json.dumps(collected_data))
