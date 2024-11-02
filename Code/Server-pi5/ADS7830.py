import smbus
import logging
from typing import List

logger = logging.getLogger('ADS7830')


class ADS7830:
    def __init__(self):
        # Get I2C bus
        self.bus = smbus.SMBus(1)
        # I2C address of the device
        self.ADS7830_DEFAULT_ADDRESS = 0x48
        # ADS7830 Command Set
        self.ADS7830_CMD = 0x84  # Single-Ended Inputs

    def read_adc(self, channel):
        """Select the Command data from the given provided value above"""
        COMMAND_SET = self.ADS7830_CMD | (
            (((channel << 2) | (channel >> 1)) & 0x07) << 4
        )
        self.bus.write_byte(self.ADS7830_DEFAULT_ADDRESS, COMMAND_SET)
        data = self.bus.read_byte(self.ADS7830_DEFAULT_ADDRESS)
        return data

    def power(self, channel):
        readings: List[int] = []
        for i in range(9):
            adc_value = self.read_adc(channel)
            readings.append(adc_value)
            logger.debug(f"Reading {i+1}/9 from channel {channel}: {adc_value}")
        readings.sort()
        battery_voltage = readings[4] / 255.0 * 5.0 * 2
        return battery_voltage


if __name__ == '__main__':
    import time

    logging.basicConfig(level=logging.INFO)
    adc = ADS7830()
    try:
        while True:
            voltage = adc.power(0)
            print(f"Battery Voltage: {voltage} V")
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Program terminated by user")
