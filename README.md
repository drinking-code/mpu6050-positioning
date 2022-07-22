# Positioning with MPU 6050

> ⚠️ not very accurrate ⚠️

Code to calculate position from acceleration and rotation data of an MPU 6050

# Usage
Run `python calibration.py` while leaving the IMU in stable position on a level plane to calibrate.  
Then, run `python main.py` or use the `listen_to_accel()` function from `main.py` to start positioning. Leave the IMU in the exact same position from the calibration for the first 20 seconds (this is for the calibration of the DMP). The function returns a tuple with the data and a stop function. The data is continuously updated.
