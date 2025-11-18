import machine
from machine import Pin
from machine import Timer

# HW refer e0035

# Function to perform every 1 second

class ShortCircuit:

    def __init__(self):

        #Short circuit Interface
        self.pulse = machine.PWM(machine.Pin(21, Pin.PULL_UP))
        self.pulse.freq(10000)
        self.pulse.duty_ns(int(50000)) # 50%
       
        self.ENABLE = Pin(3, Pin.OUT, Pin.PULL_UP)
        self.A0 = Pin(6, Pin.OUT)
        self.A1 = Pin(7, Pin.OUT)
        self.A2 = Pin(8, Pin.OUT)
        # Create a Timer object
        self.short_timer = Timer()

    def short_circuit_off(self, timer):
        self.A0.value(1)
        self.A1.value(1)
        self.A2.value(1)
        # deactivate en pin
        self.ENABLE.value(1)

    def short(self, cell, time):
        if (cell == 6):
            self.A0.value(0)
            self.A1.value(0)
            self.A2.value(0)
        elif (cell == 5):
            self.A0.value(1)
            self.A1.value(0)
            self.A2.value(0)
        elif (cell == 4):
            self.A0.value(0)
            self.A1.value(1)
            self.A2.value(0)
        elif (cell == 3):
            self.A0.value(1)
            self.A1.value(1)
            self.A2.value(0)
        elif (cell == 2):
            self.A0.value(0)
            self.A1.value(0)
            self.A2.value(1)
        elif (cell == 1):
            self.A0.value(1)
            self.A1.value(0)
            self.A2.value(1)
        else:
            pass
        # enable output, active LOW
        self.ENABLE.value(0)
        self.short_timer.init(period=time * 1000, mode=Timer.ONE_SHOT, callback=self.short_circuit_off)
