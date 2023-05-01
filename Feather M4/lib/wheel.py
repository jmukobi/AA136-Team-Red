# type: ignore
import board
import pwmio

pwm0 = pwmio.PWMOut(board.D12, frequency=5000, duty_cycle=0)

def run(wheel, throttle):
    val = int(throttle*65535)
    pwm0.duty_cycle = val
    return