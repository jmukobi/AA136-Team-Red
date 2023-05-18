# type: ignore
import board
import pwmio
import digitalio

#Define IO
pwm1 = pwmio.PWMOut(board.A3, frequency=5000, duty_cycle=1)
dir1 = digitalio.DigitalInOut(board.A4)
dir1.direction = digitalio.Direction.OUTPUT

pwm2 = pwmio.PWMOut(board.A3, frequency=5000, duty_cycle=1)
dir2 = digitalio.DigitalInOut(board.A4)
dir2.direction = digitalio.Direction.OUTPUT

pwm3 = pwmio.PWMOut(board.A3, frequency=5000, duty_cycle=1)
dir3 = digitalio.DigitalInOut(board.A4)
dir3.direction = digitalio.Direction.OUTPUT

def run(wheel, throttle):
    #Set wheel to throttle value
    val = int(abs(throttle)*65535)

    if wheel == 1:
        pwm1.duty_cycle = val
        if throttle > 0:
            dir1.value = True
        else:
            dir1.value = False

    elif wheel == 2:
        pwm2.duty_cycle = val
        if throttle > 0:
            dir2.value = True
        else:
            dir2.value = False

    elif wheel == 3:
        pwm3.duty_cycle = val
        if throttle > 0:
            dir3.value = True
        else:
            dir3.value = False

    return