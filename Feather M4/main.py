# type: ignore
import board
import magnetorquer
import wheel
import time
import sensor


STEP = .1

while True:
    magnetorquer.run(1, 0)
    wheel.run(1, 0)
    print(sensor.report()["mag"])
    time.sleep(STEP)