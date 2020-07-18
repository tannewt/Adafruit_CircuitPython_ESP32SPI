    # def set_analog_read(self, pin, atten=ADC_ATTEN_DB_11):
    #     """
    #     Get the analog input value of pin. Returns an int between 0 and 65536.

    #     :param int pin: ESP32 GPIO pin to read from.
    #     :param int atten: attenuation constant
    #     """
    #     # Verify nina-fw => 1.5.0
    #     fw_semver_maj = bytes(self.firmware_version).decode("utf-8")[2]
    #     assert int(fw_semver_maj) >= 5, "Please update nina-fw to 1.5.0 or above."

    #     resp = self._send_command_get_response(_SET_ANALOG_READ_CMD, ((pin,), (atten,)))
    #     resp_analog = struct.unpack("<i", resp[0])
    #     if resp_analog[0] < 0:
    #         raise ValueError(
    #             "_SET_ANALOG_READ parameter error: invalid pin", resp_analog[0]
    #         )
    #     if self._debug:
    #         print(resp, resp_analog, resp_analog[0], 16 * resp_analog[0])
    #     return 16 * resp_analog[0]
