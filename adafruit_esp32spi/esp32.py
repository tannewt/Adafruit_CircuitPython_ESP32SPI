# SPDX-FileCopyrightText: Copyright (c) 2019 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_esp32spi`
================================================================================

CircuitPython driver library for using ESP32 as WiFi  co-processor using SPI


* Author(s): ladyada

Implementation Notes
--------------------

**Hardware:**

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

"""

import struct
import time
from micropython import const
from digitalio import Direction
from adafruit_bus_device.spi_device import SPIDevice

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_ESP32SPI.git"

# pylint: disable=bad-whitespace
_SET_NET_CMD = const(0x10)
_SET_PASSPHRASE_CMD = const(0x11)
_SET_AP_NET_CMD = const(0x18)
_SET_AP_PASSPHRASE_CMD = const(0x19)
_SET_DEBUG_CMD = const(0x1A)

_GET_CONN_STATUS_CMD = const(0x20)
_GET_IPADDR_CMD = const(0x21)
_GET_MACADDR_CMD = const(0x22)
_GET_CURR_SSID_CMD = const(0x23)
_GET_CURR_BSSID_CMD = const(0x24)
_GET_CURR_RSSI_CMD = const(0x25)
_GET_CURR_ENCT_CMD = const(0x26)

_SCAN_NETWORKS = const(0x27)
_START_SERVER_TCP_CMD = const(0x28)
_GET_SOCKET_CMD = const(0x3F)
_GET_STATE_TCP_CMD = const(0x29)
_DATA_SENT_TCP_CMD = const(0x2A)
_AVAIL_DATA_TCP_CMD = const(0x2B)
_GET_DATA_TCP_CMD = const(0x2C)
_START_CLIENT_TCP_CMD = const(0x2D)
_STOP_CLIENT_TCP_CMD = const(0x2E)
_GET_CLIENT_STATE_TCP_CMD = const(0x2F)
_DISCONNECT_CMD = const(0x30)
_GET_IDX_RSSI_CMD = const(0x32)
_GET_IDX_ENCT_CMD = const(0x33)
_REQ_HOST_BY_NAME_CMD = const(0x34)
_GET_HOST_BY_NAME_CMD = const(0x35)
_START_SCAN_NETWORKS = const(0x36)
_GET_FW_VERSION_CMD = const(0x37)
_SEND_UDP_DATA_CMD = const(0x39)
_GET_TIME = const(0x3B)
_GET_IDX_BSSID_CMD = const(0x3C)
_GET_IDX_CHAN_CMD = const(0x3D)
_PING_CMD = const(0x3E)

_SEND_DATA_TCP_CMD = const(0x44)
_GET_DATABUF_TCP_CMD = const(0x45)
_INSERT_DATABUF_TCP_CMD = const(0x46)
_SET_ENT_IDENT_CMD = const(0x4A)
_SET_ENT_UNAME_CMD = const(0x4B)
_SET_ENT_PASSWD_CMD = const(0x4C)
_SET_ENT_ENABLE_CMD = const(0x4F)
_SET_CLI_CERT = const(0x40)
_SET_PK = const(0x41)

_SET_PIN_MODE_CMD = const(0x50)
_SET_DIGITAL_WRITE_CMD = const(0x51)
_SET_ANALOG_WRITE_CMD = const(0x52)
_SET_DIGITAL_READ_CMD = const(0x53)
_SET_ANALOG_READ_CMD = const(0x54)

_START_CMD = const(0xE0)
_END_CMD = const(0xEE)
_ERR_CMD = const(0xEF)
_REPLY_FLAG = const(1 << 7)
_CMD_FLAG = const(0)

SOCKET_CLOSED = const(0)
SOCKET_LISTEN = const(1)
SOCKET_SYN_SENT = const(2)
SOCKET_SYN_RCVD = const(3)
SOCKET_ESTABLISHED = const(4)
SOCKET_FIN_WAIT_1 = const(5)
SOCKET_FIN_WAIT_2 = const(6)
SOCKET_CLOSE_WAIT = const(7)
SOCKET_CLOSING = const(8)
SOCKET_LAST_ACK = const(9)
SOCKET_TIME_WAIT = const(10)

WL_NO_SHIELD = const(0xFF)
WL_NO_MODULE = const(0xFF)
WL_IDLE_STATUS = const(0)
WL_NO_SSID_AVAIL = const(1)
WL_SCAN_COMPLETED = const(2)
WL_CONNECTED = const(3)
WL_CONNECT_FAILED = const(4)
WL_CONNECTION_LOST = const(5)
WL_DISCONNECTED = const(6)
WL_AP_LISTENING = const(7)
WL_AP_CONNECTED = const(8)
WL_AP_FAILED = const(9)

ADC_ATTEN_DB_0 = const(0)
ADC_ATTEN_DB_2_5 = const(1)
ADC_ATTEN_DB_6 = const(2)
ADC_ATTEN_DB_11 = const(3)
# pylint: enable=bad-whitespace

class Network:
    def __init__(self, esp, index, ssid):
        self._esp = esp
        self._index = index
        self._ssid = ssid
        self._rssi = None
        self._encryption = None
        self._bssid = None
        self._channel = None

    @property
    def ssid(self):
        return self._ssid

    @property
    def rssi(self):
        if self._rssi is None:
            rssi = self._esp._send_command_get_response(_GET_IDX_RSSI_CMD, ((self._index,),))
            self._rssi = struct.unpack("<i", rssi)[0]
        return self._rssi

    @property
    def encryption(self):
        if self._encryption is None:
            encr = self._esp._send_command_get_response(_GET_IDX_ENCT_CMD, ((self._index,),))
            self._encryption = encr[0]
        return self._encryption

    @property
    def bssid(self):
        if self._bssid is None:
            bssid = self._esp._send_command_get_response(_GET_IDX_BSSID_CMD, ((self._index,),))
            self._bssid = bssid
        return self._bssid

    @property
    def channel(self):
        if self._channel is None:
            chan = self._esp._send_command_get_response(_GET_IDX_CHAN_CMD, ((self._index,),))
            self._channel = chan[0]
        return self._channel

class ESP32:
    """A class that will talk to an ESP32 module programmed with special firmware
    that lets it act as a fast an efficient WiFi co-processor"""
    TCP_MODE = const(0)
    UDP_MODE = const(1)
    TLS_MODE = const(2)

    # pylint: disable=too-many-arguments
    def __init__(
        self, spi, *, chip_select, ready, reset, gpio0=None, debug=False
    ):
        self._debug = debug
        self._buffer = bytearray(10)
        self._pbuf = bytearray(1)  # buffer for param read
        self._sendbuf = bytearray(256)  # buffer for command sending

        self._spi_device = SPIDevice(spi, chip_select, baudrate=8000000)
        self._cs = chip_select
        self._ready = ready
        self._reset = reset
        self._gpio0 = gpio0
        self._cs.direction = Direction.OUTPUT
        self._ready.direction = Direction.INPUT
        self._reset.direction = Direction.OUTPUT
        if self._gpio0:
            self._gpio0.direction = Direction.INPUT
        self.reset()

    # pylint: enable=too-many-arguments

    def reset(self):
        """Hard reset the ESP32 using the reset pin"""
        print("Reset ESP32")
        if self._gpio0:
            self._gpio0.direction = Direction.OUTPUT
            self._gpio0.value = True  # not bootload mode
        self._cs.value = True
        self._reset.value = False
        time.sleep(0.01)  # reset
        self._reset.value = True
        time.sleep(0.75)  # wait for it to boot up
        if self._gpio0:
            self._gpio0.direction = Direction.INPUT

    def _wait_for_ready(self, *, timeout=0, invert=False):
        """Wait until the ready pin goes low"""
        start_time = time.monotonic()
        while timeout == 0 or (time.monotonic() - start_time) < timeout:
            if self._ready.value == invert: # we're ready!
                return True
        return False

    # pylint: disable=too-many-branches
    def _send_command(self, cmd, params=None, *, param_len_16=False, timeout=0):
        """Send over a command with a list of parameters"""
        if not params:
            params = ()

        packet_len = 4  # header + end byte
        for i in range(len(params)): # do not do enumerate here because it allocates a temporary tuple
            param = params[i]
            packet_len += len(param)  # parameter
            packet_len += 1  # size byte
            if param_len_16:
                packet_len += 1  # 2 of em here!
        while packet_len % 4 != 0:
            packet_len += 1
        # we may need more space
        if packet_len > len(self._sendbuf):
            self._sendbuf = bytearray(packet_len)

        self._sendbuf[0] = _START_CMD
        self._sendbuf[1] = cmd & ~_REPLY_FLAG
        self._sendbuf[2] = len(params)

        # handle parameters here
        ptr = 3
        for i in range(len(params)): # do not do enumerate here because it allocates a temporary tuple
            param = params[i]
            # print("\tSending param #%d `%s` is %d bytes long" % (i, str(param), len(param)))
            if param_len_16:
                self._sendbuf[ptr] = (len(param) >> 8) & 0xFF
                ptr += 1
            self._sendbuf[ptr] = len(param) & 0xFF
            ptr += 1
            for j in range(len(param)):
                par = param[j]
                self._sendbuf[ptr + j] = par
            ptr += len(param)
        self._sendbuf[ptr] = _END_CMD

        self._wait_for_ready(timeout=timeout)
        with self._spi_device as spi:
            # Wait until not ready.
            if not self._wait_for_ready(timeout=timeout, invert=True):
                raise RuntimeError("ESP32 timed out on SPI select")
            spi.write(
                self._sendbuf, start=0, end=packet_len
            )  # pylint: disable=no-member
            # print("Wrote: ", [hex(b) for b in self._sendbuf[0:packet_len]])

    # pylint: disable=too-many-branches

    def _read_byte(self, spi):
        """Read one byte from SPI"""
        spi.readinto(self._pbuf)
        # print("\t\tRead_byte:", hex(self._pbuf[0]))
        return self._pbuf[0]

    def _read_bytes(self, spi, buf, start=0, end=None):
        """Read many bytes from SPI"""
        if not end:
            end = len(buf)
        spi.readinto(buf, start=start, end=end)
        # print("\t\tRead bytes:", [hex(i) for i in buf])

    def _wait_spi_char(self, spi, desired):
        """Read a byte with a time-out, and if we get it, check that its what we expect"""
        times = time.monotonic()
        while (time.monotonic() - times) < 0.1:
            r = self._read_byte(spi)
            if r == _ERR_CMD:
                print("command error: 0x{:x} 0x{:x}".format(self._read_byte(spi), self._read_byte(spi)))
                raise RuntimeError("Error response to command")
            if r == desired:
                return True
        raise RuntimeError("Timed out waiting for SPI char")

    def _check_data(self, spi, desired):
        """Read a byte and verify its the value we want"""
        r = self._read_byte(spi)
        if r != desired:
            raise RuntimeError("Expected %02X but got %02X" % (desired, r))

    def _wait_response_cmd(self, cmd, num_responses=None, *, param_len_16=False, timeout=0):
        """Wait for ready, then parse the response.

           If the command returns more than one value then a list will be returned. If a value is a
           single byte, then it will be returned as an int. Otherwise the value will be a bytearray."""
        self._wait_for_ready()

        responses = None
        with self._spi_device as spi:
            # Wait until not ready.
            if not self._wait_for_ready(timeout=timeout, invert=True):
                raise RuntimeError("ESP32 timed out on SPI select")

            self._wait_spi_char(spi, _START_CMD)
            self._check_data(spi, cmd | _REPLY_FLAG)
            if num_responses is not None:
                self._check_data(spi, num_responses)
            else:
                num_responses = self._read_byte(spi)
                if num_responses == 1:
                    raise RuntimeError()
            if num_responses > 1:
                responses = [None] * num_responses
            for num in range(num_responses):
                param_len = self._read_byte(spi)
                if param_len_16:
                    param_len <<= 8
                    param_len |= self._read_byte(spi)
                if param_len == 0:
                    continue
                # print("\tParameter #%d length is %d" % (num, param_len))
                if param_len > 1 or num_responses > 1:
                    response = bytearray(param_len)
                    self._read_bytes(spi, response)
                else:
                    response = self._read_byte(spi)
                if num_responses > 1:
                    responses[num] = response
                else:
                    responses = response
            self._check_data(spi, _END_CMD)

        # print("Read %d: " % len(responses[0]), responses)
        return responses

    def _read_response_into(self, cmd, buf, *, timeout=0, param_len_16=False):
        """Wait for ready, then read the single response parameter into buf. Return the size read in."""
        self._wait_for_ready()

        param_len = 0
        with self._spi_device as spi:
            # Wait until not ready.
            if not self._wait_for_ready(timeout=timeout, invert=True):
                raise RuntimeError("ESP32 timed out on SPI select")

            self._wait_spi_char(spi, _START_CMD)
            self._check_data(spi, cmd | _REPLY_FLAG)
            # Verify that only one value was returned
            self._check_data(spi, 1)

            param_len = self._read_byte(spi)
            if param_len_16:
                param_len <<= 8
                param_len |= self._read_byte(spi)
            self._read_bytes(spi, buf)
            self._check_data(spi, _END_CMD)

        # print("Read %d: " % len(responses[0]), responses)
        return param_len

    def _send_command_get_response(
        self,
        cmd,
        params=None,
        *,
        reply_params=1,
        sent_param_len_16=False,
        recv_param_len_16=False,
        timeout=0
    ):
        """Send a high level SPI command, wait and return the response"""
        self._send_command(cmd, params, param_len_16=sent_param_len_16, timeout=timeout)
        return self._wait_response_cmd(
            cmd, reply_params, param_len_16=recv_param_len_16, timeout=timeout
        )

    @property
    def status(self):
        """The status of the ESP32 WiFi core. Can be WL_NO_SHIELD or WL_NO_MODULE
        (not found), WL_IDLE_STATUS, WL_NO_SSID_AVAIL, WL_SCAN_COMPLETED,
        WL_CONNECTED, WL_CONNECT_FAILED, WL_CONNECTION_LOST, WL_DISCONNECTED,
        WL_AP_LISTENING, WL_AP_CONNECTED, WL_AP_FAILED"""
        resp = self._send_command_get_response(_GET_CONN_STATUS_CMD, timeout=10)
        return resp  # one byte response

    @property
    def firmware_version(self):
        """A string of the firmware version on the ESP32"""
        resp = self._send_command_get_response(_GET_FW_VERSION_CMD, timeout=1)
        return str(resp, "utf-8")

    @property
    def mac_address(self):
        """A bytearray containing the MAC address of the ESP32"""
        resp = self._send_command_get_response(_GET_MACADDR_CMD, [b"\xFF"])
        return resp

    def start_scanning_networks(self):
        """Scan for visible access points, yields Network object."""
        resp = self._send_command_get_response(_START_SCAN_NETWORKS)
        if resp != 1:
            raise RuntimeError("Failed to start AP scan")
        done = False
        for i in range(10):  # attempts
            self._send_command(_SCAN_NETWORKS)
            essids = self._wait_response_cmd(_SCAN_NETWORKS)
            for i, essid in enumerate(essids):
                done = True
                yield Network(self, i, essid)
            if done:
                break

    def stop_scanning_networks(self):
        """Stops scanning for networks"""
        # Nina firmware does a fixed time scan so no need to stop it.
        pass

    def _wifi_set_network(self, ssid, *, timeout=10):
        """Tells the ESP32 to set the access point to the given ssid"""
        resp = self._send_command_get_response(_SET_NET_CMD, [ssid], timeout=timeout)
        if resp[0][0] != 1:
            raise RuntimeError("Failed to set network")

    def connect(self, ssid, password=None, *, timeout=10):
        """
        Connect to an access point with given name and password.
        Will wait until specified timeout seconds and return on success
        or raise an exception on failure.

        :param ssid: the SSID to connect to
        :param passphrase: the password of the access point
        :param timeout: number of seconds until we time out and fail to create AP
        """
        if isinstance(ssid, str):
            ssid = bytes(ssid, "utf-8")
        if password:
            if isinstance(password, str):
                password = bytes(password, "utf-8")
            resp = self._send_command_get_response(_SET_PASSPHRASE_CMD, [ssid, password], timeout=timeout)
            if resp != 1:
                raise RuntimeError("Failed to set passphrase")
        else:
            self._wifi_set_network(ssid, timeout=timeout)
        times = time.monotonic()
        while (time.monotonic() - times) < timeout:  # wait up until timeout
            stat = self.status
            if stat == WL_CONNECTED:
                return stat
            time.sleep(0.05)
        if stat in (WL_CONNECT_FAILED, WL_CONNECTION_LOST, WL_DISCONNECTED):
            raise RuntimeError("Failed to connect to ssid", ssid)
        if stat == WL_NO_SSID_AVAIL:
            raise RuntimeError("No such ssid", ssid)
        raise RuntimeError("Unknown error 0x%02X" % stat)

    def connect_to_enterprise(self, ssid, *, username=None, password=None, identity=None, timeout=10):
        self._wifi_set_network(ssid, timeout=timeout)
        if identity:
            resp = self._send_command_get_response(_SET_ENT_IDENT_CMD, [ident], timeout=timeout)
            if resp[0] != 1:
                raise RuntimeError("Failed to set enterprise anonymous identity")

        if username:
            """Sets the desired WPA2 Enterprise username"""
            resp = self._send_command_get_response(_SET_ENT_UNAME_CMD, [username], timeout=timeout)
            if resp[0] != 1:
                raise RuntimeError("Failed to set enterprise username")

        if password:
            resp = self._send_command_get_response(_SET_ENT_PASSWD_CMD, [password], timeout=timeout)
            if resp[0] != 1:
                raise RuntimeError("Failed to set enterprise password")

        resp = self._send_command_get_response(_SET_ENT_ENABLE_CMD, timeout=timeout)
        if resp[0] != 1:
            raise RuntimeError("Failed to enable enterprise mode")

    @property
    def ssid(self):
        """The name of the access point we're connected to"""
        resp = self._send_command_get_response(_GET_CURR_SSID_CMD, [b"\xFF"])
        return resp

    @property
    def bssid(self):
        """The MAC-formatted service set ID of the access point we're connected to"""
        resp = self._send_command_get_response(_GET_CURR_BSSID_CMD, [b"\xFF"])
        return resp

    @property
    def rssi(self):
        """The receiving signal strength indicator for the access point we're
        connected to"""
        resp = self._send_command_get_response(_GET_CURR_RSSI_CMD, [b"\xFF"])
        return struct.unpack("<i", resp)[0]

    @property
    def ip_address(self):
        resp = self._send_command_get_response(
            _GET_IPADDR_CMD, [b"\xFF"], reply_params=3
        )
        """Our local IP address"""
        return resp[0]

    @property
    def access_point_active(self):
        """Returns if the ESP32 is in access point mode and is listening for connections"""
        try:
            return self.status == WL_AP_LISTENING
        except RuntimeError:
            self.reset()
            return False

    def start_access_point(
        self, ssid, password, channel=1, timeout=10
    ):  # pylint: disable=invalid-name
        """
        Create an access point with the given name, password, and channel.
        Will wait until specified timeout seconds and return on success
        or raise an exception on failure.

        :param str ssid: the SSID of the created Access Point. Must be less than 32 chars.
        :param str password: the password of the created Access Point. Must be 8-63 chars.
        :param int channel: channel of created Access Point (1 - 14).
        :param int timeout: number of seconds until we time out and fail to create AP
        """
        if len(ssid) > 32:
            raise RuntimeError("ssid must be no more than 32 characters")
        if password and not (8 <= len(password) < 64):
            raise RuntimeError("password must be 8 - 63 characters")
        if not 1 <= channel <= 14:
            raise RuntimeError("channel must be between 1 and 14")

        if isinstance(channel, int):
            channel = bytes(channel)
        if isinstance(ssid, str):
            ssid = bytes(ssid, "utf-8")
        if password:
            if isinstance(password, str):
                password = bytes(password, "utf-8")
            resp = self._send_command_get_response(
                _SET_AP_PASSPHRASE_CMD, [ssid, passphrase, channel], timeout=timeout
            )
            if resp[0][0] != 1:
                raise RuntimeError("Failed to setup AP password")
        else:
            resp = self._send_command_get_response(_SET_AP_NET_CMD, [ssid, channel], timeout=timeout)
            if resp[0][0] != 1:
                raise RuntimeError("Failed to setup AP network")
            self._wifi_set_ap_network(ssid, channel)

        times = time.monotonic()
        while timeout == 0 or (time.monotonic() - times) < timeout:  # wait up to timeout
            stat = self.status
            if stat == WL_AP_LISTENING:
                return stat
            time.sleep(0.05)
        if stat == WL_AP_FAILED:
            raise RuntimeError("Failed to create AP", ssid)
        raise RuntimeError("Unknown error 0x%02x" % stat)

    def _gethostbyname(self, hostname):
        """Convert a hostname to a packed 4-byte IP address. Returns
        a 4 bytearray"""
        if isinstance(hostname, str):
            hostname = bytes(hostname, "utf-8")
        resp = self._send_command_get_response(_REQ_HOST_BY_NAME_CMD, (hostname,))
        if resp != 1:
            raise RuntimeError("Failed to request hostname")
        resp = self._send_command_get_response(_GET_HOST_BY_NAME_CMD)
        return resp

    def ping(self, dest, ttl=250):
        """Ping a destination IP address or hostname, with a max time-to-live
        (ttl). Returns a millisecond timing value"""
        if isinstance(dest, str):  # convert to IP address
            dest = self._gethostbyname(dest)
        # ttl must be between 0 and 255
        ttl = max(0, min(ttl, 255))
        resp = self._send_command_get_response(_PING_CMD, (dest, (ttl,)))
        return struct.unpack("<H", resp)[0]

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, enabled):
        """Enable/disable debug mode on the ESP32. Debug messages will be
        written to the ESP32's UART."""
        resp = self._send_command_get_response(_SET_DEBUG_CMD, ((bool(enabled),),))
        if resp[0][0] != 1:
            raise RuntimeError("Failed to set debug mode")
