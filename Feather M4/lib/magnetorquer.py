# type: ignore
import board
from adafruit_motorkit import MotorKit

i2c = board.I2C()  # uses board.SCL and board.SDA
kit = MotorKit(i2c=i2c)

def run(port, throttle):
    if port == 1:
        kit.motor1.throttle = throttle
    elif port == 2:
        kit.motor2.throttle = throttle
    elif port == 3:
        kit.motor3.throttle = throttle
    elif port == 4:
        kit.motor4.throttle = throttle
    #print(f"Running magnetorquer {port} at {throttle*100}% throttle.")
    return