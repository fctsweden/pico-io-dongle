
import machine
from machine import UART, Pin
import time
import sys

from spi_io import spi_io
from pwm_in import pwm_in
from shortcircuit import ShortCircuit

# pi pico IO version
version = "2.6.1"


#----- HW configuration for test bench -------------

pwm_instances={}
pwm_in_instances={}
i2c_instances={}
spi_instances={}

# Define ADC pin
# adc init 
adc0=machine.ADC(Pin(26))
adc1=machine.ADC(Pin(27))
adc2=machine.ADC(Pin(28))


# Initialize UART0
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1, Pin.IN, Pin.PULL_UP))

# Value init to HIGH, to make sure valve is OFF from startup / reset
valve_ctrl_pin = Pin(20, Pin.OUT, Pin.PULL_UP)
valve_ctrl_pin.value(1)

#Cbus interface
cbus_tx_enable_pin=machine.Pin(2,machine.Pin.OUT)
cbus_tx_enable_pin.value(1)

# short circuit
short_circuit_obj = ShortCircuit()

print(f"Pico IO bridge verion {version}")

def printHelp():
    print("Supported commands:")
    print("version  # Read pi IO software version")
    print("gpiow <pin> <value> # Set GPIO pin to value")
    print("gpior <pin> # Read GPIO pin")
    print("adc <pin> # Read ADC pin. Use either gpio numbers or the adc channel numbers (0-4). Channel 4 is the internal temp sensor")
    print("pwm <pin> <frequency in Hz> <duty percent>")
    print("pwm_in <pin> # Init pin as pwm input"  )
    print("pwm_read <pin> # Read PWM input pin")    
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
    print("short <cell> <time>")

def i2c_getInstance(inst,freq=100000):
    i2c=None
    if inst in i2c_instances:
        i2c=i2c_instances[inst]
        #print("Reusing existing I2C instance",inst)
    else:
        if inst==0:
            i2c=machine.I2C(0, scl=Pin(5), sda=Pin(4), freq=freq)
        elif inst==1:
            i2c=machine.I2C(1, scl=Pin(19), sda=Pin(18), freq=freq)
    i2c_instances[inst]=i2c
    return i2c

printHelp()
machine.Pin("LED",machine.Pin.OUT).value(1)

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
            p=machine.Pin(pin,machine.Pin.OUT)
            p.value(value)
            print(f"gpiow:{pin}", value)
        elif cmdline[0]=="gpior":
            if (len(cmdline)!=2):
                print("Usage: gpior <pin>")
                continue
            pin=int(cmdline[1])
            p=machine.Pin(pin,machine.Pin.IN)
            print(f"gpior:{pin}", p.value())

        # ADC command    
        elif cmdline[0]=="adc":
            if (len(cmdline)!=2):
                print("Usage: adc <pin>")
                continue
            value = 0
            pin=int(cmdline[1])
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
                print("pwm: Duty Err")
                continue
            period_us=1000000/freq
            duty_ns = duty*period_us*1000/100
            if pin in pwm_instances:
                pwm=pwm_instances[pin]
            else:
                pwm=machine.PWM(machine.Pin(pin, Pin.PULL_UP))
            pwm_instances[pin]=pwm
            pwm.freq(freq)
            pwm.duty_ns(int(duty_ns))
            print("pwm:",pin,freq,duty)
        elif cmdline[0]=="pwm_in":
            if (len(cmdline)!=2):
                print("Usage: pwm_in <pin>")
                continue
            pin=int(cmdline[1])
            pwm_in_obj = pwm_in(pin)
            pwm_in_instances[pin] = pwm_in_obj
            print("pwm_in:", pin, "OK")
        elif cmdline[0]=="pwm_read":
            if (len(cmdline)!=2):
                print("Usage: pwm_read <pin>")
                continue
            pin=int(cmdline[1])
            if(pin in pwm_in_instances):
                print(f"pwm_read:{pin}", pwm_in_instances[pin].pwm_read())
            else:
                print(f"No Supported. Init by pwm_in {pin}")

        # I2C commands
        elif cmdline[0]=="i2cselect":
            if (len(cmdline)!=2):
                print("Usage: i2cselect <instance> # Select I2C instance 0 or 1 for following commands")
                continue
            inst=int(cmdline[1],0)
            #print("Selecting I2C instance",inst)
            i2c=i2c_getInstance(inst)
        elif cmdline[0]=="i2c":
            if (len(cmdline)<2):
                print("Usage: i2c <freq> [instance] # Instantiate i2c interface")
                continue
            freq=int(cmdline[1])
            if len(cmdline)>2:
                inst=int(cmdline[2],0)
                i2c=i2c_getInstance(inst)
            else:
                i2c=i2c_getInstance(0,freq)
            print("i2c: OK")
        elif cmdline[0]=="i2cscan":
            if not 'i2c' in locals():
                continue
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
            data=i2c.readfrom(address, length)
            print("i2cread:",data.hex())
        elif cmdline[0]=="i2cwrite":
            if (len(cmdline)<3):
                print("Usage: i2cwrite <address> <data>")
                continue
            address=int(cmdline[1],0)
            data=bytearray(bytes.fromhex("".join(cmdline[2:])))
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
            i2c.writeto_mem(address,register,data)

        # SPI commands
        elif cmdline[0]=="spi_init":
            if (len(cmdline)<3):
                print("Usage: spi <instance> <freq> # Instantiate spi interface")
                continue
            inst=int(cmdline[1], 0)
            freq=int(cmdline[2], 0)
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
            spi_dev= spi_instances[instance]
            try:
                rx_data = bytearray(len(data))   # Buffer for response
                acks=spi_dev.spi_write_read(data, rx_data)
                print("spi_read:",rx_data.hex())
            except OSError as e:
                print("spi_read:Error:",e)

        # Cbus command:
        elif cmdline[0]=="cbus":
            if (len(cmdline)<2):
                print("Usage: cbus <cmd>")
                print("e.g. ReadFCACData: cbus  0x00 0x0C 0x11 0x00 0x00")
                continue

            cmd=bytearray(bytes.fromhex("".join(cmdline[1:])))
            try:
                # enable cbus write 
                cbus_tx_enable_pin.value(0)
                uart0.write(cmd)
                while not uart0.txdone():
                    pass   # wait until hardware finished sending
                time.sleep_us(100)  # 100 µs
                # disable cbus tx                
                cbus_tx_enable_pin.value(1)

                #drop echoed bytes on rx buffer
                echo = uart0.read(len(cmd))
                # wait resp with 5ms timeout
                cnt = 5
                while cnt:
                    if uart0.any():   # check if there is something in buffer
                        data = uart0.read()   # read one line (until \n or timeout)
                        if data:
                            #print(data.hex())
                            sys.stdout.buffer.write(data)
                            break
                    cnt -= 1
                    time.sleep_ms(1)  # sleep 1 millisecond

            except OSError as e:
                print("cbus:Error:",e)

        # Short circuite:
        elif cmdline[0]=="short":
            if (len(cmdline)!=3):
                print("Usage: short <cell> <time>")
                continue

            cell=int(cmdline[1])
            short_time =int(cmdline[2])
            short_circuit_obj.short(cell, short_time)
            print(f"short:{cell}", short_time)
        else:
            print("Unknown command")
            printHelp()

    except EOFError:
        break
    except Exception as e:
        print("Error:",e)
        continue


