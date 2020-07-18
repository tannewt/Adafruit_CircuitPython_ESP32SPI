# SPDX-FileCopyrightText: Copyright (c) 2019 Brent Rubell for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`PWMOut`
==============================
PWMOut CircuitPython API for ESP32SPI.

* Author(s): Brent Rubell
"""

from . import pin

class PWMOut:
    """
    Implementation of CircuitPython PWMOut for ESP32SPI.

    :param ESP_SPIcontrol esp: The ESP object we are using.
    :param int esp_pin: Valid ESP32 GPIO Pin.
    :param int duty_cycle: The fraction of each pulse which is high, 16-bit.
    :param int frequency: Not supported.
    :param bool variable_frequency: Not supported.
    """

    def __init__(
        self, esp, pin, *, frequency=None, duty_cycle=0, variable_frequency=None
    ):
        if not isinstance(pin, pin.Pin)
            raise AttributeError("Pin is not a valid ESP32 GPIO Pin.")
        self._pin = pin
        self._esp = esp
        self._duty_cycle = duty_cycle
        if frequency or variable_frequency is not None:
            raise NotImplementedError("PWMOut frequency not implemented in ESP32SPI")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.deinit()

    def deinit(self):
        """De-initalize the PWMOut object."""
        self._duty_cycle = 0
        self._pin = None

    def _is_deinited(self):
        """Checks if PWMOut object has been previously de-initalized"""
        if self._pin is None:
            raise ValueError(
                "PWMOut Object has been deinitialized and can no longer "
                "be used. Create a new PWMOut object."
            )

    @property
    def duty_cycle(self):
        """The PWMOut duty cycle as a 16-bit value from 0 to 65535."""
        self._is_deinited()
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, duty_cycle):
        self._is_deinited()
        if not isinstance(duty_cycle, int):
            raise TypeError("Invalid duty_cycle, should be an int.")
        if not 0 <= duty_cycle <= 65535:
            raise ValueError("Invalid duty_cycle, should be between 0 and 65535")
        resp = self._send_command_get_response(
            _SET_ANALOG_WRITE_CMD, ((self._pin.number,), (duty_cycle // 256,))
        )
        if resp[0][0] != 1:
            raise RuntimeError("Failed to write to pin")
        self._duty_cycle = duty_cycle

    @property
    def frequency(self):
        """Returns the PWMOut object's frequency value."""
        self._is_deinited()
        return None

    @frequency.setter
    def frequency(self, freq):
        """Sets the PWMOut object's frequency value.
        :param int freq: 32-bit value that dictates the PWM frequency in Hertz.
        NOTE: Only writeable when constructed with variable_Frequency=True.
        """
        raise NotImplementedError("PWMOut Frequency not implemented in ESP32SPI")
