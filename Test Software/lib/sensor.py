# type: ignore
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
from adafruit_motorkit import MotorKit

i2c = board.I2C()  # uses board.SCL and board.SDA
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)

def report():
    #print(mag.magnetic)
    report = {"mag":mag.magnetic,"accel":accel_gyro.acceleration,"gyro":accel_gyro.gyro,"temp":accel_gyro.temperature}
    return report