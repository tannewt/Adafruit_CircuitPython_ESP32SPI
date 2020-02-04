"""Prints out the current version of ESP firmware."""

import board
import busio
from digitalio import DigitalInOut
from adafruit_esp32spi import adafruit_esp32spi

# If you are using a board with pre-defined ESP32 Pins:
# esp32_cs = DigitalInOut(board.ESP_CS)
# esp32_ready = DigitalInOut(board.ESP_BUSY)
# esp32_reset = DigitalInOut(board.ESP_RESET)

# If you have an ItsyBitsy Airlift:
esp32_cs = DigitalInOut(board.D13)
esp32_ready = DigitalInOut(board.D11)
esp32_reset = DigitalInOut(board.D12)

# If you have an externally connected ESP32:
# esp32_cs = DigitalInOut(board.D9)
# esp32_ready = DigitalInOut(board.D10)
# esp32_reset = DigitalInOut(board.D5)

spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

if esp.status == adafruit_esp32spi.WL_IDLE_STATUS:
    print("ESP32 found and in idle mode")
print("Firmware vers.", esp.firmware_version)
