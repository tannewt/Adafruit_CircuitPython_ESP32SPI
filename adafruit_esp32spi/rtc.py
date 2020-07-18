
    # def get_time(self):
    #     """The current unix timestamp"""
    #     if self.status == WL_CONNECTED:
    #         resp = self._send_command_get_response(_GET_TIME)
    #         resp_time = struct.unpack("<i", resp[0])
    #         if resp_time == (0,):
    #             raise ValueError("_GET_TIME returned 0")
    #         return resp_time
    #     if self.status in (WL_AP_LISTENING, WL_AP_CONNECTED):
    #         raise RuntimeError(
    #             "Cannot obtain NTP while in AP mode, must be connected to internet"
    #         )
    #     raise RuntimeError("Must be connected to WiFi before obtaining NTP.")
