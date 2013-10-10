import time

import uinput
from Xlib import display

def mousepos():
    """mousepos() --> (x, y) get the mouse coordinates on the screen (linux, Xlib)."""
    data2 = display.Display().screen().root.query_pointer()._data
    return data2["root_x"], data2["root_y"]

def main():
    events = (
        uinput.REL_X,
        uinput.REL_Y,
        uinput.BTN_LEFT,
        uinput.BTN_RIGHT,
        )

    device = uinput.Device(events)
    p = mousepos()
    print p
    x = 3000 - p[0]
    y = 3000 - p[1]
    print x
    print y

    device.emit(uinput.REL_X, x, syn=False)
    device.emit(uinput.REL_Y, y)

    time.sleep(0.01)

if __name__ == "__main__":
    main()