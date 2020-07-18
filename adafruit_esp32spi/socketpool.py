# SPDX-FileCopyrightText: Copyright (c) 2019 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_esp32spi_socket`
================================================================================

A socket compatible interface thru the ESP SPI command set

* Author(s): ladyada
"""

import struct
import time
import gc

_GET_SOCKET_CMD = const(0x3F)


_SOCKET_CLOSED = const(0)
_SOCKET_LISTEN = const(1)
_SOCKET_SYN_SENT = const(2)
_SOCKET_SYN_RCVD = const(3)
_SOCKET_ESTABLISHED = const(4)
_SOCKET_FIN_WAIT_1 = const(5)
_SOCKET_FIN_WAIT_2 = const(6)
_SOCKET_CLOSE_WAIT = const(7)
_SOCKET_CLOSING = const(8)
_SOCKET_LAST_ACK = const(9)
_SOCKET_TIME_WAIT = const(10)

class SocketPool:
    """A pool of network sockets available from the given ESP coprocessor.

       Creating multiple pools will not make more sockets available.

       Instances of this class can be used in place of CPython's `socket` module.
    """
    SOCK_STREAM = 1
    SOCK_DGRAM = 2

    AF_INET = 2

    MAX_PACKET = 4000

    def __init__(self, esp):
        self.esp = esp

    def socket(self, family=AF_INET, type=SOCK_STREAM, proto=0, fileno=None):
        socknum = self.esp._send_command_get_response(_GET_SOCKET_CMD)
        if socknum == 255:
            raise RuntimeError("No sockets available")
        print("Allocated socket #%d" % socknum)

        return _Socket(self, socknum, family=family, type=type, proto=proto, fileno=fileno)

    def gethostbyname(self, hostname):
        """Convert a hostname to a packed 4-byte IP address. Returns
        a 4 bytearray"""
        return self.esp._gethostbyname(hostname)

    def getaddrinfo(self, host, port, family=0, socktype=0, proto=0, flags=0):
        """Given a hostname and a port name, return a 'socket.getaddrinfo'
        compatible list of tuples. Honestly, we ignore anything but host & port"""
        if not isinstance(port, int):
            raise RuntimeError("Port must be an integer")
        ipaddr = self.gethostbyname(host)
        return [(SocketPool.AF_INET, socktype, proto, "", (ipaddr, port))]

# def socket_write(self, socket_num, buffer, conn_mode=TCP_MODE, *, timeout=0):
#     """Write the bytearray buffer to a socket"""
#     if self._debug:
#         print("Writing:", buffer)
#     self._socknum_ll[0][0] = socket_num
#     sent = 0
#     total_chunks = (len(buffer) // 64) + 1
#     send_command = _SEND_DATA_TCP_CMD
#     if conn_mode == self.UDP_MODE:  # UDP requires a different command to write
#         send_command = _INSERT_DATABUF_TCP_CMD
#     for chunk in range(total_chunks):
#         resp = self._send_command_get_response(
#             send_command,
#             (
#                 self._socknum_ll[0],
#                 memoryview(buffer)[(chunk * 64) : ((chunk + 1) * 64)],
#             ),
#             sent_param_len_16=True,
#             timeout=timeout,
#         )
#         sent += resp[0][0]

#     if conn_mode == self.UDP_MODE:
#         # UDP verifies chunks on write, not bytes
#         if sent != total_chunks:
#             raise RuntimeError(
#                 "Failed to write %d chunks (sent %d)" % (total_chunks, sent)
#             )
#         # UDP needs to finalize with this command, does the actual sending
#         resp = self._send_command_get_response(_SEND_UDP_DATA_CMD, self._socknum_ll)
#         if resp[0][0] != 1:
#             raise RuntimeError("Failed to send UDP data")
#         return

#     if sent != len(buffer):
#         print(resp, sent)
#         raise RuntimeError(
#             "Failed to send %d bytes (sent %d)" % (len(buffer), sent)
#         )

#     resp = self._send_command_get_response(_DATA_SENT_TCP_CMD, self._socknum_ll)
#     if resp[0][0] != 1:
#         raise RuntimeError("Failed to verify data sent")

# def socket_available(self, socket_num, *, timeout=0):
#     """Determine how many bytes are waiting to be read on the socket"""
#     self._socknum_ll[0][0] = socket_num
#     resp = self._send_command_get_response(_AVAIL_DATA_TCP_CMD,
#                                            self._socknum_ll,
#                                            timeout=timeout)
#     reply = struct.unpack('<H', resp[0])[0]
#     if self._debug:
#         print("ESPSocket: %d bytes available" % reply)
#     return reply

# def socket_read(self, socket_num, size, *, timeout=0):
#     """Read up to 'size' bytes from the socket number. Returns a bytes object"""
#     if self._debug:
#         print(
#             "Reading %d bytes from ESP socket with status %d"
#             % (size, self.socket_status(socket_num, timeout=timeout))
#         )
#     self._socknum_ll[0][0] = socket_num
#     resp = self._send_command_get_response(
#         _GET_DATABUF_TCP_CMD,
#         (self._socknum_ll[0], (size & 0xFF, (size >> 8) & 0xFF)),
#         sent_param_len_16=True,
#         recv_param_len_16=True,
#                                            timeout=timeout,
#     )
#     return bytes(resp[0])

# def start_server(
#     self, port, socket_num, conn_mode=TCP_MODE, ip=None
# ):  # pylint: disable=invalid-name
#     """Opens a server on the specified port, using the ESP32's internal reference number"""
#     if self._debug:
#         print("*** starting server")
#     self._socknum_ll[0][0] = socket_num
#     params = [struct.pack(">H", port), self._socknum_ll[0], (conn_mode,)]
#     if ip:
#         params.insert(0, ip)
#     resp = self._send_command_get_response(_START_SERVER_TCP_CMD, params)

#     if resp[0][0] != 1:
#         raise RuntimeError("Could not start server")

# def server_state(self, socket_num):
#     """Get the state of the ESP32's internal reference server socket number"""
#     self._socknum_ll[0][0] = socket_num
#     resp = self._send_command_get_response(_GET_STATE_TCP_CMD, self._socknum_ll)
#     return resp[0][0]

TCP_MODE = const(0)
UDP_MODE = const(1)
TLS_MODE = const(2)

_DATA_SENT_TCP_CMD = const(0x2A)
_AVAIL_DATA_TCP_CMD = const(0x2B)
_START_CLIENT_TCP_CMD = const(0x2D)
_GET_CLIENT_STATE_TCP_CMD = const(0x2F)
_SEND_DATA_TCP_CMD = const(0x44)
_GET_DATABUF_TCP_CMD = const(0x45)

class _Socket:
    """A simplified implementation of the Python 'socket' class, for connecting
    through an interface to a remote device"""

    def __init__(
        self, pool, socknum, family=SocketPool.AF_INET, type=SocketPool.SOCK_STREAM, proto=0, fileno=None
    ):
        if family != SocketPool.AF_INET:
            raise RuntimeError("Only AF_INET family supported")
        if type != SocketPool.SOCK_STREAM:
            raise RuntimeError("Only SOCK_STREAM type supported")
        self._socknum = socknum
        self._pool = pool
        self._esp = pool.esp
        self._conntype = TCP_MODE

        # These are allocations we reuse often so make them once.
        self._two_byte_buf = bytearray(2)
        self._socknum_param = ((socknum,),)
        self._get_params = [self._socknum_param[0], bytearray(2)]

        self.settimeout(0)

    @property
    def _status(self):
        """Get the socket connection status, can be SOCKET_CLOSED, SOCKET_LISTEN,
        SOCKET_SYN_SENT, SOCKET_SYN_RCVD, SOCKET_ESTABLISHED, SOCKET_FIN_WAIT_1,
        SOCKET_FIN_WAIT_2, SOCKET_CLOSE_WAIT, SOCKET_CLOSING, SOCKET_LAST_ACK, or
        SOCKET_TIME_WAIT"""
        resp = self._esp._send_command_get_response(
            _GET_CLIENT_STATE_TCP_CMD, self._socknum_param, timeout=self._timeout
        )
        return resp

    def _socket_open(self, dest, port):
        """Open a socket to a destination IP address or hostname
        using the ESP32's internal reference number. By default we use
        'conn_mode' TCP_MODE but can also use UDP_MODE or TLS_MODE
        (dest must be hostname for TLS_MODE!)"""
        print("*** Open socket")
        port_param = struct.pack(">H", port)
        print(dest, port, port_param)
        if isinstance(dest, str):  # use the 5 arg version
            dest = bytes(dest, "utf-8")
            resp = self._esp._send_command_get_response(
                _START_CLIENT_TCP_CMD,
                (
                    dest,
                    b"\x00\x00\x00\x00",
                    port_param,
                    self._socknum_param[0],
                    (self._conntype,)
                ),
                timeout=self._timeout
            )
        else:  # ip address, use 4 arg vesion
            resp = self._esp._send_command_get_response(
                _START_CLIENT_TCP_CMD,
                (dest, port_param, self._socknum_param[0], (self._conntype,)),
                                                    timeout=self._timeout,
            )
        print("*** Open socket response", resp)
        if resp != 1:
            raise RuntimeError("Could not connect to remote server")

    def _connect(self, address, port):
        self._socket_open(address, port)
        start = time.monotonic()
        while (not self._timeout or time.monotonic() - start < self._timeout) and self._status != _SOCKET_ESTABLISHED:
            time.sleep(0.01)

        if self._status != _SOCKET_ESTABLISHED:
            raise RuntimeError("Failed to establish connection")
        print("connect took", time.monotonic() - start, "seconds")
        self._buffer = b""


    def connect(self, address):
        """Connect the socket to the 'address' (which can be 32bit packed IP or
        a hostname string). 'conntype' is an extra that may indicate SSL or not,
        depending on the underlying interface"""
        host, port = address
        addr = self._pool.gethostbyname(host)
        return self._connect(addr, port)

    def sendto(self, data, address):
        sent = 0
        total_chunks = (len(data) // 64) + 1
        data = memoryview(data)
        for chunk in range(total_chunks):
            resp = self._send_command_get_response(
                _INSERT_DATABUF_TCP_CMD,
                (
                    self._socknum_param[0],
                    data[(chunk * 64) : ((chunk + 1) * 64)],
                ),
                sent_param_len_16=True,
                timeout=self._timeout,
            )
            sent += resp[0][0]

        # UDP verifies chunks on write, not bytes
        if sent != total_chunks:
            raise RuntimeError(
                "Failed to write %d chunks (sent %d)" % (total_chunks, sent)
            )
        # UDP needs to finalize with this command, does the actual sending
        resp = self._send_command_get_response(_SEND_UDP_DATA_CMD, ((self._socknum,),))
        if resp[0] != 1:
            raise RuntimeError("Failed to send UDP data")


    def send(self, data):
        """Send some data to the socket"""
        sent = 0
        total_chunks = (len(data) // 64) + 1
        data = memoryview(data)

        for chunk in range(total_chunks):
            resp = self._esp._send_command_get_response(
                _SEND_DATA_TCP_CMD,
                (
                    self._socknum_param[0],
                    data[(chunk * 64) : ((chunk + 1) * 64)],
                ),
                sent_param_len_16=True,
                timeout=self._timeout,
            )
            sent += resp[0]

        if sent != len(data):
            print(resp, sent)
            raise RuntimeError(
                "Failed to send %d bytes (sent %d)" % (len(data), sent)
            )

        resp = self._esp._send_command_get_response(_DATA_SENT_TCP_CMD, self._socknum_param)
        if resp != 1:
            raise RuntimeError("Failed to verify data sent")

    def readline(self):
        """Attempt to return as many bytes as we can up to but not including '\r\n'"""
        # print("Socket readline")
        stamp = time.monotonic()
        while b"\r\n" not in self._buffer:
            # there's no line already in there, read some more
            avail = self.available()
            if avail:
                # TODO: Remove this because it causes a longer byte string allocation.
                self._buffer += _socket_provider.socket_read(self._socknum, avail,
                                                             timeout=self._timeout)
            elif self._timeout > 0 and time.monotonic() - stamp > self._timeout:
                return None

        duration = time.monotonic() - stamp
        if duration > 0.1:
            print("readline finished", duration)
        firstline, self._buffer = self._buffer.split(b'\r\n', 1)
        gc.collect()
        return firstline

    def recv_into(self, buf, nbytes=-1):
        if nbytes < 0:
            nbytes = len(buf)

        offset = 0
        view = None
        start = time.monotonic()
        while offset < nbytes:
            # Always check for a timeout.
            duration = time.monotonic() - start
            if self._timeout:
                if duration > self._timeout:
                    break
                else:
                    remaining_time = self._timeout - duration

            # Only read the available data
            avail = self._available
            if avail:
                size = min(nbytes - offset, avail)
            else:
                continue

            b = buf
            if offset > 0:
                if not view:
                    view = memoryview(buf)
                b = view[offset:]

            struct.pack_into("<H", self._get_params[1], 0, size)
            self._esp._send_command(
                        _GET_DATABUF_TCP_CMD,
                        self._get_params,
                        param_len_16=True,
                        timeout=remaining_time,
                    )
            read = self._esp._read_response_into(
                _GET_DATABUF_TCP_CMD, b, timeout=remaining_time, param_len_16=True
            )
            offset += read
        return offset


    def recv(self, bufsize=0):
        """Reads some bytes from the connected remote address.
        :param int bufsize: maximum number of bytes to receive
        """
        buf = bytearray(min(bufsize, self._available))
        self.recv_into(buf)
        return bytes(buf)

    def settimeout(self, value):
        """Set the read timeout for sockets, if value is 0 it will block"""
        self._timeout = value

    @property
    def _available(self):
        """How many bytes of data are available to be read from the ESP"""
        self._esp._send_command(_AVAIL_DATA_TCP_CMD, self._socknum_param,
                                 timeout=self._timeout,
                )
        self._esp._read_response_into(
            _AVAIL_DATA_TCP_CMD, self._two_byte_buf, timeout=self._timeout
        )
        # Don't use struct to avoid the tuple allocation. Instead compute the little endian uint16_t
        # ourselves.
        return self._two_byte_buf[0] | (self._two_byte_buf[1] << 8)

    def connected(self):
        """Whether or not we are connected to the socket"""
        if self.socknum == NO_SOCKET_AVAIL:
            return False
        if self.available():
            return True
        status = _socket_provider.socket_status(self.socknum, timeout=self._timeout)
        result = status not in (
            adafruit_esp32spi.SOCKET_LISTEN,
            adafruit_esp32spi.SOCKET_CLOSED,
            adafruit_esp32spi.SOCKET_FIN_WAIT_1,
            adafruit_esp32spi.SOCKET_FIN_WAIT_2,
            adafruit_esp32spi.SOCKET_TIME_WAIT,
            adafruit_esp32spi.SOCKET_SYN_SENT,
            adafruit_esp32spi.SOCKET_SYN_RCVD,
            adafruit_esp32spi.SOCKET_CLOSE_WAIT
        )
        if not result:
            self.close()
            self._socknum = NO_SOCKET_AVAIL
        return result

    @property
    def socknum(self):
        """The socket number"""
        return self._socknum

    def close(self):
        """Close the socket, after reading whatever remains"""
        self._socknum_ll[0][0] = self._socknum
        resp = self._send_command_get_response(_STOP_CLIENT_TCP_CMD, self._socknum_ll, timeout=self._timeout)
        if resp[0][0] != 1:
            raise RuntimeError("Failed to close socket")
