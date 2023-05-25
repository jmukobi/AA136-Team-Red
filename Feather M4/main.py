# type: ignore
import board
import magnetorquer
import wheel
import time
import sensor
import actuate

STEP = .1

while True:
    #run wheel
    #Args: wheel number(1-3), throttle (percent from 0-1)
    wheel.run(1, 1)

    #run magnetorquer
    #Args: magnetorquer number(1-3), throttle (percent from 0-1)
    magnetorquer.run(1, 1)
    time.sleep(STEP)