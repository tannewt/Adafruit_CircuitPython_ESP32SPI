# SPDX-FileCopyrightText: 2019 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

from secrets import secrets
import board
import busio
import ipaddress
from digitalio import DigitalInOut
import time
import memorymonitor

mm = memorymonitor.AllocationSize()

import gc
start = gc.mem_free()
from adafruit_esp32spi import socketpool
gc.collect()
after_socket = gc.mem_free()
print("socket", start - after_socket)
from adafruit_esp32spi import esp32
gc.collect()
after_esp32spi = gc.mem_free()
print("esp32spi", after_socket - after_esp32spi)
import adafruit_requests as requests
gc.collect()
after_requests = gc.mem_free()
print("requests", after_esp32spi - after_requests)

# socket 2800
# esp32spi 17584
# requests 1584

print("ESP32 SPI webclient test")

TEXT_URL = "http://wifitest.adafruit.com/testwifi/index.html"
JSON_URL = "http://api.coindesk.com/v1/bpi/currentprice/USD.json"

# If you are using a board with pre-defined ESP32 Pins:
esp32_cs = DigitalInOut(board.ESP_CS)
esp32_ready = DigitalInOut(board.ESP_BUSY)
esp32_reset = DigitalInOut(board.ESP_RESET)
esp32_gpio0 = DigitalInOut(board.ESP_GPIO0)

debug_out = busio.UART(rx=board.ESP_RX, baudrate=115200, receiver_buffer_size=512)

# If you have an AirLift Shield:
# esp32_cs = DigitalInOut(board.D10)
# esp32_ready = DigitalInOut(board.D7)
# esp32_reset = DigitalInOut(board.D5)

# If you have an AirLift Featherwing or ItsyBitsy Airlift:
# esp32_cs = DigitalInOut(board.D13)
# esp32_ready = DigitalInOut(board.D11)
# esp32_reset = DigitalInOut(board.D12)

# If you have an externally connected ESP32:
# NOTE: You may need to change the pins to reflect your wiring
# esp32_cs = DigitalInOut(board.D9)
# esp32_ready = DigitalInOut(board.D10)
# esp32_reset = DigitalInOut(board.D5)

spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
esp = esp32.ESP32(spi, chip_select=esp32_cs, ready=esp32_ready, reset=esp32_reset, gpio0=esp32_gpio0)

#requests.set_socket(socket, esp)

print(dir(debug_out))
while debug_out.in_waiting:
    print(debug_out.readline())

print("Firmware vers.", esp.firmware_version)

while debug_out.in_waiting:
    print(debug_out.readline())
if esp.status == esp32.WL_IDLE_STATUS:
    print("ESP32 found and in idle mode")
print("Firmware vers.", esp.firmware_version)
print("MAC addr:", [hex(i) for i in esp.mac_address])

start_scan = time.monotonic()
for ap in esp.start_scanning_networks():
    print("\t%s\t\tRSSI: %d" % (str(ap.ssid, "utf-8"), ap.rssi))
esp.stop_scanning_networks()
print(time.monotonic() - start_scan, "scan time")

print("Connecting to AP...")
while not esp.ssid:
    try:
        esp.connect(secrets["ssid"], secrets["password"])
    except RuntimeError as e:
        print("could not connect to AP, retrying: ", e)
        continue
print("Connected to", str(esp.ssid, "utf-8"), "\tRSSI:", esp.rssi)
print("My IP address is", ipaddress.ip_address(esp.ip_address))
print("Ping google.com: %d ms" % esp.ping("google.com"))


socket_pool = socketpool.SocketPool(esp)
print(
    "IP lookup wifitest.adafruit.com: %s" % ipaddress.ip_address(socket_pool.gethostbyname("wifitest.adafruit.com"))
)
http = requests.Session(socket_pool)
# # esp._debug = True
print("Fetching text from", TEXT_URL)

while debug_out.in_waiting:
    print(debug_out.readline())

with mm:
    r = http.get(TEXT_URL)
    t = r.text

for bucket, count in enumerate(mm):
    if not count:
        continue
    print("<=", mm.bytes_per_block * (2 ** bucket), count)

print("-" * 40)
print(t)
print("-" * 40)

print()
print("Fetching json from", JSON_URL)

with mm:
    r = http.get(JSON_URL)

print(mm.bytes_per_block)

for bucket, count in enumerate(mm):
    if not count:
        continue
    print("<=", mm.bytes_per_block * (2 ** bucket), count)

print("parsing")
with mm:
    j = r.json()

for bucket, count in enumerate(mm):
    if not count:
        continue
    print("<=", mm.bytes_per_block * (2 ** bucket), count)

print("-" * 40)
print(j)
print("-" * 40)

print("Done!")
