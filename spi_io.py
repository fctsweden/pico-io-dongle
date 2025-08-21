from machine import Pin, SPI
import time

class spi_io:

    def __init__(self, inst = 1, speed = 100000):
        # Initialize SPI
        # SPI1, baudrate=100KHz, polarity=0, phase=0
        self.spi = SPI(1, baudrate = 100000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
                       sck=Pin(10), mosi=Pin(11), miso=Pin(12))
        # Test station support 2 SPI sensors in SPI1, nna 20/Aug/2025
        # Initialize Chip Select (CS) pin, active low
        if(inst == 1):
            self.cs = Pin(13, Pin.OUT, value=1)
        else:
            self.cs = Pin(9, Pin.OUT, value=1)

        print("spi1 interface init done")
    def spi_write_read(self, data, rx_data):
        """Write data to SPI and read response."""
        self.cs.value(0)  # Select the peripheral
        time.sleep_us(50)  # Small delay for stability
        self.spi.write_readinto(data, rx_data)  # Write data and read response
        time.sleep_us(50)  # Small delay for stability
        self.cs.value(1)  # Deselect the peripheral
        return rx_data

    def spi_write(self, data):
        """Write data to SPI device."""
        self.cs.value(0)  # Select the peripheral
        time.sleep_us(50)  # Small delay for stability
        self.spi.write(data)  # Send data
        time.sleep_us(50)  # Small delay for stability
        self.cs.value(1)  # Deselect the peripheral
