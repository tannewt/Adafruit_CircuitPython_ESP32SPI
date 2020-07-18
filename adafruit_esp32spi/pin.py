
__init_done = False

class Pin:
    """
    An ESP32 Pin
    """

    def __init__(self, number):
        if __init_done:
            raise RuntimeError("Cannot intialize pins at runtime")
        self.number = number

    def __repr__(self):
        return str(self)

    def __str__(self):
        return "IO" + str(self.number)

IO0 = Pin(0)
IO1 = Pin(1)
IO2 = Pin(2)
IO4 = Pin(4)
IO5 = Pin(5)
IO12 = Pin(12)
IO13 = Pin(13)
IO14 = Pin(14)
IO15 = Pin(15)
IO16 = Pin(16)
IO17 = Pin(17)
IO18 = Pin(18)
IO19 = Pin(19)
IO21 = Pin(21)
IO22 = Pin(22)
IO23 = Pin(23)
IO25 = Pin(25)
IO26 = Pin(26)
IO27 = Pin(27)
IO32 = Pin(32)
IO33 = Pin(33)

__init_done = True
