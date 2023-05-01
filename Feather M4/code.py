# type: ignore

import time
import board
import pwmio

# LED setup for most CircuitPython boards:
pwm = pwmio.PWMOut(board.D12, frequency=5000, duty_cycle=0)
# LED setup for QT Py M0:
# led = pwmio.PWMOut(board.SCK, frequency=5000, duty_cycle=0)

print("Running")
while True:
    gas = 0
    for i in range(100):
        gas += 1/100
        val = int(gas*65535)
        print(val)
        pwm.duty_cycle = val
        time.sleep(.1)
