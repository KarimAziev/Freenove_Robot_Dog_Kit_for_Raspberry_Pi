import smbus
import logging
from typing import List

logger = logging.getLogger('ADS7830')


class ADS7830:
    def __init__(self, bus_number: int = 1, address: int = 0x4B):
        """Initialize the ADS7830 ADC.

        :param bus_number: I2C bus number
        :param address: I2C address of the ADS7830 device
        """
        self.bus = smbus.SMBus(bus_number)
        self.address = address
        # ADS7830 Command Set for Single-Ended Inputs
        self.ADS7830_CMD = 0x84
        logger.info(
            f"ADS7830 initialized on I2C bus {bus_number}, address {hex(address)}"
        )

    def read_adc(self, channel: int) -> int:
        """Read ADC value from the specified channel.

        :param channel: Channel number (0-7)
        :return: ADC value (0-255)
        """
        if channel < 0 or channel > 7:
            raise ValueError("Channel must be between 0 and 7")

        # Build the command for the ADC
        command_set = self.ADS7830_CMD | (
            (((channel << 2) | (channel >> 1)) & 0x07) << 4
        )
        try:
            # Write command to the ADS7830
            self.bus.write_byte(self.address, command_set)
            # Read data back from the ADS7830
            data = self.bus.read_byte(self.address)
            logger.debug(f"Read ADC channel {channel}: {data}")
            return data
        except Exception as e:
            logger.error(f"Error reading ADC channel {channel}: {e}")
            return 0

    def power(self, channel: int) -> float:
        """Calculate the battery voltage from ADC readings.

        :param channel: ADC channel number (0-7)
        :return: Calculated battery voltage
        """
        readings: List[int] = []
        for i in range(9):
            adc_value = self.read_adc(channel)
            readings.append(adc_value)
            logger.debug(f"Reading {i+1}/9 from channel {channel}: {adc_value}")
        readings.sort()
        median_adc = readings[4]

        MAX_ADC_VALUE = 255.0
        REF_VOLTAGE = 5.0
        VOLTAGE_DIVIDER_RATIO = 2
        battery_voltage = (
            (median_adc / MAX_ADC_VALUE) * REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO
        )
        battery_voltage = round(battery_voltage, 2)
        logger.info(
            f"Calculated battery voltage from channel {channel}: {battery_voltage} V"
        )
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
