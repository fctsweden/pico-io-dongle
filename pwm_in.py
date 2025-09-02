import machine
from machine import Pin, SPI
from machine import Timer
import time

# Define variables for PWM input reading
MIN_2_SEC          = 60
PPR                = 2   #pulses per revolution
pwm_cnt            = 0
rpm                = 0

pwm_in_list={}

timer = None


# Callback function for the interrupt
def pwm_callback(pin_obj):
    if(pwm_in_list is not None):
        pwm_in = pwm_in_list[pin_obj]   # pin.id() returns the integer number
        pwm_in.pwm_cnt += 1

# Function to perform every 1 second
def pwm_handle_task(timer):    
    # f = 1 / t
    for pin_obj, pwm_in in pwm_in_list.items():
        pwm_in.rpm = ( pwm_in.pwm_cnt / PPR) * MIN_2_SEC
        pwm_in.pwm_cnt = 0

class pwm_in:

    def __init__(self, pin = 1):
        global timer
        # Setup GPIO pin
        self.pin_obj = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_DOWN)   # Replace 16 with your GPIO pin number
        self.pin_obj.irq(trigger=machine.Pin.IRQ_FALLING, handler=pwm_callback)
        self.pwm_cnt = 0
        self.rpm = 0
        pwm_in_list[self.pin_obj] = self
        if(timer is None):
            # Create a Timer object
            timer = Timer()
            # Configure the timer to call the callback every 1 second (1000 ms)
            timer.init(period=1000, mode=Timer.PERIODIC, callback=pwm_handle_task)


    def pwm_read(self):
        return int(self.rpm)

