# type: ignore
import board
import magnetorquer
import wheel
import time
import sensor
import actuate

STEP = .1

while True:
    report = sensor.report()
    time.sleep(STEP)