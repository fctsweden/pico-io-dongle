# Pico-io dongle
This is a simple python script that turns a raspberry pi pico into a usb io extender. 

You can flash your pico with the standard micropython uf2 image and then copy the main.py from this repo to it
or flash a prebuilt uf2 image that contains it all (see Releases in this repo)

# Using from mpremote/terminal
Run mpremote or a terminal program like "minicom -D /dev/ttyACM0" to connect to the pico.

Type "help" and you should see this help
```
Pico IO bridge
Supported commands:
gpiow <pin> <value> (writes a gpio pin)
gpior <pin> (reads a gpio pin)
adc <adc pin> (reads an adc pin value, valid pins are 0,1,2)
pwm <pin> <freq> <duty percent>
i2c <freq> instantiate i2c interface on pins 21(SCL) and 20(SDA)
i2cscan
i2cread <address> <memaddr> <length>
```

The "pin" numbers are all gpio numbers. 

# Using from you script
If you want to use the pico-io board from a python script, have a look at client.py

It connects to the serial port and issues the above commands, returning the results.

```
p=pico_io("/dev/ttyACM0")
print(p.gpioread("0"))
print(p.gpiowrite("0",1))
print(p.gpioread("0"))
print(p.adc(0))
print(p.pwm(25,1000,50))
```


# Creating an uf2 file
use picotool to save a copy of a connected pico to an image file:

    picotool save out.uf2

