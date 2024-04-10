"""
This is a simple client for the pico_io running on a raspberry pi pico.
Run this script on your computer to control the pico_io running on the raspberry pi pico.
"""
import serial
import re
class pico_io:
    def __init__(self,serialport:str):
        self.serialport=serial.Serial(serialport, 115200, timeout=1)
        self.pwms={}
    def pwm(self,pin:int,freq:int, duty:int):
        resp=self.command(f"pwm {pin} {freq} {duty}")
        self.pwms[pin]=(freq,duty)
        return resp
    
    def command(self,cmd:str):
        self.serialport.write(f"{cmd}\r\n".encode("utf-8"))
        self.serialport.flush()
        line=self.readLine()
        timeout=5
        cmd1=cmd.split(" ")[0]
        while not line.startswith(cmd1+":"):
            timeout-=1
            if timeout==0:
                raise Exception(f"timeout waiting for {cmd} response")
            
            #print(f"waiting for {cmd} response, got",line)
            line=self.readLine()
        return line
    def adc(self,pin:int):
        resp=self.command(f"adc {pin}\n")
        g=re.match(r"adc:(\d+) (\d+)",resp)
        if g:
            return int(g.group(2))
        else :
            return None
    def gpiowrite(self,pin:str,value:int):
        return self.command(f"gpiow {pin} {value}")
    def gpioread(self,pin:str):
        return self.command(f"gpior {pin}")
    def readLine(self)->str:
        return self.serialport.readline().decode("utf-8").strip()
    
#test the class
if __name__ == '__main__':
    p=pico_io("/dev/ttyACM0")
    print(p.gpioread("0"))
    print(p.gpiowrite("0",1))
    print(p.gpioread("0"))
    print(p.adc(0))
    print(p.pwm(25,1000,50))