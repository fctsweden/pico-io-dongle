
import machine
from machine import Pin
import time
from machine import Timer
from spi_io import spi_io

# pi pico IO version
version = "2.2"

#currently active pwm pins, dict of pin -> pwm object
pwmpins={}
i2cinstances={}
spi_instances={}

# Define variables for PWM input reading
PWM_IN_PIN     = 10
pwm_high_time = 0
pwm_low_time = 0
last_time = 0
pwm_frequency = 0
pwm_duty_cycle = 0

# Define ADC pin
# adc init 
adc0=machine.ADC(Pin(26))
adc1=machine.ADC(Pin(27))
adc2=machine.ADC(Pin(28))


print(f"Pico IO bridge verion {version}")

def printHelp():
    print("Supported commands:")
    print("version  # Read pi IO software version")
    print("gpiow <pin> <value> # Set GPIO pin to value")
    print("gpior <pin> # Read GPIO pin")
    print("adc <pin> # Read ADC pin. Use either gpio numbers or the adc channel numbers (0-4). Channel 4 is the internal temp sensor")
    print("pwm <pin> <frequency in Hz> <duty percent>")
    print("pwmr <pin> # Read PWM pin. Currently support pwmr 10 only")    
    print("i2cselect <instance> # Select I2C instance 0 or 1 for following commands.")
    print("i2c <freq> [instance] # Instantiate i2c interface. If instance is not provided, I2C0 is used.  Pin 21(SCL) and 20(SDA) are used for I2C0, 19(SCL) and 18(SDA) are used for I2C1.")
    print("i2cscan # Scan I2C bus for devices")
    print("i2cread <address> <length>")
    print("i2cwrite <address> <data>")
    print("i2c_readreg <address> <register> <length>")
    print("i2c_writereg <address> <register> <data>")
    print("spi_init <instance> <freq>")
    print("spi_write <instance> <data>")
    print("spi_read <instance> <command>")

# Callback function for the interrupt
def pwm_callback(pin):
    global pwm_high_time, pwm_low_time, last_time

    current_time = time.ticks_us()  # Get the current time in microseconds
    if pin.value():  # Rising edge (signal goes high)
        pwm_low_time = time.ticks_diff(current_time, last_time)  # Time spent low
        last_time = current_time
    else:  # Falling edge (signal goes low)
        pwm_high_time = time.ticks_diff(current_time, last_time)  # Time spent high
        last_time = current_time

def pwm_in_init(pin = 10): 
    # Setup GPIO pin
    pwm_pin = machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP)  # Replace 16 with your GPIO pin number
    pwm_pin.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=pwm_callback)

# Function to perform every 1 second
def pwm_handle_task(timer):    
    global pwm_high_time, pwm_low_time, pwm_frequency, pwm_duty_cycle
    period = pwm_high_time + pwm_low_time
    pwm_frequency = 1_000_000 / period if period > 0 else 0
    pwm_duty_cycle = (pwm_high_time / period) * 100 if period > 0 else 0
    #print(f"Frequency: {pwm_frequency:.2f} Hz, Duty Cycle: {pwm_duty_cycle:.2f}%")
    # reset high time and low time after calculation
    pwm_high_time = 0
    pwm_low_time = 0

def i2c_getInstance(inst,freq=100000):
    i2c=None
    if inst in i2cinstances:
        i2c=i2cinstances[inst]
        print("Reusing existing I2C instance",inst)
    else:
        if inst==0:
            i2c=machine.I2C(0, scl=Pin(5), sda=Pin(4), freq=freq)
        elif inst==1:
            i2c=machine.I2C(1, scl=Pin(19), sda=Pin(18), freq=freq)
    i2cinstances[inst]=i2c
    return i2c

printHelp()
machine.Pin("LED",machine.Pin.OUT).value(1)

pwm_in_init(PWM_IN_PIN)
# Create a Timer object
timer = Timer()
# Configure the timer to call the callback every 1 second (1000 ms)
timer.init(period=1000, mode=Timer.PERIODIC, callback=pwm_handle_task)


while True:

    try:
        line=input()
        cmdline=line.split()
        if cmdline[0]=="exit":
            break
        elif cmdline[0]=="version":
            print(f"version: {version}")
        # GPIO commands
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
            print(f"gpiow:{pin}", value)
        elif cmdline[0]=="gpior":
            if (len(cmdline)!=2):
                print("Usage: gpior <pin>")
                continue
            pin=int(cmdline[1])
            print("Reading pin",pin)
            p=machine.Pin(pin,machine.Pin.IN)
            print(f"gpior:{pin}", p.value())

        # ADC command    
        elif cmdline[0]=="adc":
            if (len(cmdline)!=2):
                print("Usage: adc <pin>")
                continue
            value = 0
            pin=int(cmdline[1])
            print("Reading ADC pin",pin)
            if(pin == 0):
                value = adc0.read_u16()
            elif (pin == 1):
                value = adc1.read_u16()
            elif (pin == 2):
                value = adc2.read_u16()
            print(f"adc:{pin}", value)

        # PWM commands
        elif cmdline[0]=="pwm":
            if (len(cmdline)!=4):
                print("Usage: pwm <pin> <freq> <duty percent>")
                continue
            pin=int(cmdline[1])
            freq=int(cmdline[2])
            duty=int(cmdline[3])
            if (duty<0 or duty>100):
                print("pwm:err Duty cycle must be between 0 and 100")
                continue
            if(pin == PWM_IN_PIN):
                print(f"GPIO {PWM_IN_PIN} is used to read FAN Speed")
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
        elif cmdline[0]=="pwmr":
            if (len(cmdline)!=2):
                print("Usage: pwmr <pin>")
                continue
            pin=int(cmdline[1])

            if (pin != PWM_IN_PIN):
                print(f"Supported: pwmr {PWM_IN_PIN} only")
                continue
            print("pwmr:",pin, pwm_frequency)

        # I2C commands
        elif cmdline[0]=="i2cselect":
            if (len(cmdline)!=2):
                print("Usage: i2cselect <instance> # Select I2C instance 0 or 1 for following commands")
                continue
            inst=int(cmdline[1],0)
            print("Selecting I2C instance",inst)
            i2c=i2c_getInstance(inst)
        elif cmdline[0]=="i2c":
            if (len(cmdline)<2):
                print("Usage: i2c <freq> [instance] # Instantiate i2c interface")
                continue
            freq=int(cmdline[1])
            if len(cmdline)>2:
                inst=int(cmdline[2],0)
                print("Selecting I2C instance",inst)
                i2c=i2c_getInstance(inst)
            else:
                print("Selecting I2C instance",0)
                i2c=i2c_getInstance(0,freq)
            print("i2c: OK")
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
            if (len(cmdline)!=3):
                print("Usage: i2cread <address> <length>")
                continue
            address=int(cmdline[1],0)
            length=int(cmdline[2])
            print("Reading",length,"bytes from",hex(address))
            data=i2c.readfrom(address, length)
            print("i2cread:",data.hex())
        elif cmdline[0]=="i2cwrite":
            if (len(cmdline)<3):
                print("Usage: i2cwrite <address> <data>")
                continue
            address=int(cmdline[1],0)
            data=bytearray(bytes.fromhex("".join(cmdline[2:])))
            print("Writing",len(data),"bytes to",hex(address))
            try:
                acks=i2c.writeto(address,data)
                print("i2cwrite:ACK:",acks)
            except OSError as e:
                print("i2cwrite:Error:",e)
        elif cmdline[0]=="i2c_readreg":
            if (len(cmdline)!=4):
                print("Usage: i2c_readreg <address> <register> <length>")
                continue
            address=int(cmdline[1],0)
            register=int(cmdline[2],0)
            length=int(cmdline[3])
            print("Reading",length,"bytes from register",hex(register),"of",hex(address))
            data=i2c.readfrom_mem(address,register,length)
            print(data)
            print("i2c_readreg:",data.hex())
        elif cmdline[0]=="i2c_writereg":
            if (len(cmdline)<4):
                print("Usage: i2c_writereg <address> <register> <data>")
                continue
            address=int(cmdline[1],16)
            register=int(cmdline[2],16)
            data=bytearray([int(x,16) for x in cmdline[3:]])
            print("Writing",len(data),"bytes to register",hex(register),"of",hex(address))
            i2c.writeto_mem(address,register,data)

        # SPI commands
        elif cmdline[0]=="spi_init":
            if (len(cmdline)<3):
                print("Usage: spi <instance> <freq> # Instantiate spi interface")
                continue
            inst=int(cmdline[1], 0)
            freq=int(cmdline[2], 0)
            print("Selecting SPI instance",inst)
            spi_dev = spi_io(inst, freq)
            spi_instances[inst] = spi_dev
            print("spi_init: OK")
        elif cmdline[0]=="spi_write":
            if (len(cmdline)<3):
                print("Usage: spi_write <instance> <data>")
                print("e.g.: spi_write 1 AA 00 00")
                continue
            instance=int(cmdline[1],0)
            data=bytearray(bytes.fromhex("".join(cmdline[2:])))
            print("Writing",len(data),"bytes to spi device",hex(instance))
            spi_dev= spi_instances[instance]
            try:
                acks=spi_dev.spi_write(data)
                print("spi_write:ACK:",acks)
            except OSError as e:
                print("spi_write:Error:",e)
        elif cmdline[0]=="spi_read":
            if (len(cmdline)< 3):
                print("Usage: spi_read <instance> <command>")
                print("e.g.: spi_read 1 F0 00 00 00")
                continue
            instance=int(cmdline[1],0)
            data=bytearray(bytes.fromhex("".join(cmdline[2:])))
            spi_dev= spi_instances[instance]
            print("Writing",len(data),"bytes to spi device",hex(instance))
            spi_dev= spi_instances[instance]
            try:
                rx_data = bytearray(len(data))   # Buffer for response
                acks=spi_dev.spi_write_read(data, rx_data)
                print("spi_read:",rx_data.hex())
            except OSError as e:
                print("spi_read:Error:",e)

        else:
            print("Unknown command")
            printHelp()

    except EOFError:
        break
    except Exception as e:
        print("Error:",e)
        continue


