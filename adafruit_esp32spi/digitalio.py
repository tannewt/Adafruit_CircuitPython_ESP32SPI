# SPDX-FileCopyrightText: Copyright (c) 2019 Brent Rubell for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`digitalio`
==============================
DigitalIO for ESP32 over SPI.

* Author(s): Brent Rubell, based on Adafruit_Blinka digitalio implementation
and bcm283x Pin implementation.
https://github.com/adafruit/Adafruit_Blinka/blob/master/src/adafruit_blinka/microcontroller/bcm283x/pin.py
https://github.com/adafruit/Adafruit_Blinka/blob/master/src/digitalio.py
"""
from micropython import const

from digitalio import Direction

_SET_PIN_MODE_CMD = const(0x50)
_SET_DIGITAL_WRITE_CMD = const(0x51)
_SET_DIGITAL_READ_CMD = const(0x53)

_PIN_MODE_IN = const(0x00)
_PIN_MODE_OUT = const(0x01)

class DigitalInOut:
    """Implementation of DigitalIO module for ESP32SPI.

    :param ESP_SPIcontrol esp: The ESP object we are using.
    :param int pin: Valid ESP32 GPIO Pin, predefined in ESP32_GPIO_PINS.
    """

    _pin = None
    # pylint: disable = attribute-defined-outside-init
    def __init__(self, esp, pin):
        self._esp = esp
        self._pin = pin
        self.direction = Direction.INPUT

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.deinit()

    def deinit(self):
        """De-initializes the pin object."""
        self._pin = None

    def switch_to_output(self, value=False, drive_mode=DriveMode.PUSH_PULL):
        """Set the drive mode and value and then switch to writing out digital values.
        :param bool value: Default mode to set upon switching.
        :param DriveMode drive_mode: Drive mode for the output.
        """
        self._direction = Direction.OUTPUT
        self.drive_mode = drive_mode
        self.value = value
        resp = self.esp.send_command_get_response(_SET_PIN_MODE_CMD, ((pin,), (_PIN_MODE_OUT,)))
        if resp[0][0] != 1:
            raise RuntimeError("Failed to set pin mode")

    def switch_to_input(self, pull=None):
        """Sets the pull and then switch to read in digital values.
        :param Pull pull: Pull configuration for the input.
        """
        self._direction = Direction.INPUT
        resp = self.esp.send_command_get_response(_SET_PIN_MODE_CMD, ((pin,), (_PIN_MODE_IN,)))
        if resp[0][0] != 1:
            raise RuntimeError("Failed to set pin mode")

    @property
    def direction(self):
        """Returns the pin's direction."""
        return self._direction

    @direction.setter
    def direction(self, direction):
        """Sets the direction of the pin.
        :param digitalio.Direction direction: Pin direction
        """
        if pin_dir is Direction.OUTPUT:
            self.switch_to_output()
        elif pin_dir is Direction.INPUT:
            self.switch_to_input()
        else:
            raise AttributeError("Not a Direction")

    @property
    def value(self):
        """The digital logic level value of the pin."""

        # Verify nina-fw => 1.5.0
        fw_semver_maj = bytes(self._esp.firmware_version).decode("utf-8")[2]
        assert int(fw_semver_maj) >= 5, "Please update nina-fw to 1.5.0 or above."

        resp = self._esp.send_command_get_response(_SET_DIGITAL_READ_CMD, ((pin,),))[0]
        if resp[0] == 0:
            return False
        if resp[0] == 1:
            return True
        raise ValueError(
            "_SET_DIGITAL_READ response error: response is not boolean", resp[0]
        )

    @value.setter
    def value(self, val):
        """Sets the digital logic level of the pin.
        :param type value: Pin logic level.
        :param int value: Pin logic level. 1 is logic high, 0 is logic low.
        :param bool value: Pin logic level. True is logic high, False is logic low.
        """
        if self.direction is Direction.OUTPUT:
            raise AttributeError("Not an output")
        self._value = val
        resp = self._esp.send_command_get_response(
            _SET_DIGITAL_WRITE_CMD, ((self.pin_id,), (1 if val else 0,))
        )
        if resp[0][0] != 1:
            raise RuntimeError("Failed to write to pin")

    @property
    def drive_mode(self):
        """Returns pin drive mode."""
        if self.direction is Direction.OUTPUT:
            return self._drive_mode
        raise AttributeError("Not an output")

    @drive_mode.setter
    def drive_mode(self, mode):
        """Sets the pin drive mode.
        :param DriveMode mode: Defines the drive mode when outputting digital values.
        Either PUSH_PULL or OPEN_DRAIN
        """
        self._drive_mode = mode
        if mode is DriveMode.OPEN_DRAIN:
            raise NotImplementedError(
                "Drive mode %s not implemented in ESP32SPI." % mode
            )
        if mode is DriveMode.PUSH_PULL:
            self._pin.init(mode=Pin.OUT)
