print("Pico IO bridge")

import machine
from machine import Pin

def printHelp():
    print("Supported commands:")
    print("gpiow <pin> <value> # Set GPIO pin to value")
    print("gpior <pin> # Read GPIO pin")
    print("adc <pin> # Read ADC pin. Use either gpio numbers or the adc channel numbers (0-4). Channel 4 is the internal temp sensor")
    print("pwm <pin> <frequency in Hz> <duty percent>")
    print("i2c <frequency in Hz> # Instantiate i2c interface on pins 21(SCL) and 20(SDA)")
    print("i2cscan # Scan I2C bus for devices")
    print("i2cread <address> <memaddr> <length>")
printHelp()
machine.Pin("LED",machine.Pin.OUT).value(1)

#currently active pwm pins, dict of pin -> pwm object
pwmpins={}

while True:

    try:
        line=input()
        cmdline=line.split()
        if cmdline[0]=="exit":
            break
        elif cmdline[0]=="gpiow":
            if (len(cmdline)!=3):
                print("Usage: gpiow <pin> <value>")
                continue
            pin=cmdline[1]
            try:
                #try to parse first as integer
                pin=int(pin)
            except:
                pass
            value=int(cmdline[2])
            print("Setting pin",pin,"to",value)
            p=machine.Pin(pin,machine.Pin.OUT)
            p.value(value)

        elif cmdline[0]=="gpior":
            if (len(cmdline)!=2):
                print("Usage: gpior <pin>")
                continue
            pin=int(cmdline[1])
            print("Reading pin",pin)
            p=machine.Pin(pin,machine.Pin.IN)
            print(f"gpior:{pin}", p.value())
        elif cmdline[0]=="adc":
            if (len(cmdline)!=2):
                print("Usage: adc <pin>")
                continue
            pin=int(cmdline[1])
            print("Reading ADC pin",pin)
            adc=machine.ADC(pin)
            print(f"adc:{pin}", adc.read_u16())
        elif cmdline[0]=="pwm":
            if (len(cmdline)!=4):
                print("Usage: pwm <pin> <freq> <duty percent>")
                continue
            pin=int(cmdline[1])
            freq=int(cmdline[2])
            duty=int(cmdline[3])
            if (duty<0 or duty>100):
                print("pwm:err Duty cycle must be between 0 and 100",)
                continue
            print("Setting PWM pin",pin,"to",freq,"Hz",duty,"%")
            period_us=1000000/freq
            duty_ns = duty*period_us*1000/100
            print("Period:",period_us,"us, Duty:",duty_ns,"ns")
            if pin in pwmpins:
                pwm=pwmpins[pin]
                print("Reusing existing PWM object",pwm.freq(),pwm.duty_ns())
            else:
                pwm=machine.PWM(machine.Pin(pin))
            pwmpins[pin]=pwm
            pwm.freq(freq)
            pwm.duty_ns(int(duty_ns))
            print("pwm:",pin,freq,duty)
        elif cmdline[0]=="i2c":
            if (len(cmdline)!=2):
                print("Usage: i2c <freq>")
                continue
            freq=int(cmdline[1])
            print("Setting I2C frequency to",freq)
            i2c=machine.I2C(0, scl=Pin(21), sda=Pin(20), freq=freq)
        elif cmdline[0]=="i2cscan":
            if not 'i2c' in locals():
                print("I2C not instantiated, use i2c command")
                continue
            print("Scanning I2C bus")
            devices=i2c.scan()
            for device in devices:
                print("Device at address",hex(device))
            print("Scan complete, found ",len(devices),"devices")
        elif cmdline[0]=="i2cread":
            if (len(cmdline)!=4):
                print("Usage: i2cread <address> <memaddr> <length>")
                continue
            address=int(cmdline[1],0)
            memaddr=int(cmdline[2],0)
            length=int(cmdline[3])
            print("Reading",length,"bytes from",hex(address))
            data=i2c.readfrom_mem(address,memaddr, length)
            print("i2cread:",data)
        elif cmdline[0]=="i2cwrite":
            if (len(cmdline)<3):
                print("Usage: i2cwrite <address> <data>")
                continue
            address=int(cmdline[1],16)
            data=bytearray([int(x,16) for x in cmdline[2:]])
            print("Writing",len(data),"bytes to",hex(address))
            acks=i2c.writeto(address,data)
            print("i2cwrite:ACK:",acks)
        elif cmdline[0]=="i2c_readreg":
            if (len(cmdline)!=4):
                print("Usage: i2c_readreg <address> <register> <length>")
                continue
            address=int(cmdline[1],16)
            register=int(cmdline[2],16)
            length=int(cmdline[3])
            print("Reading",length,"bytes from register",hex(register),"of",hex(address))
            data=i2c.readfrom_mem(address,register,length)
            print("i2c_readreg:",data)
        elif cmdline[0]=="i2c_writereg":
            if (len(cmdline)<4):
                print("Usage: i2c_writereg <address> <register> <data>")
                continue
            address=int(cmdline[1],16)
            register=int(cmdline[2],16)
            data=bytearray([int(x,16) for x in cmdline[3:]])
            print("Writing",len(data),"bytes to register",hex(register),"of",hex(address))
            i2c.writeto_mem(address,register,data)
            
        else:
            print("Unknown command")
            printHelp()

    except EOFError:
        break
    except Exception as e:
        print("Error:",e)
        continue