import time

import uinput

def main():
    events = (
        uinput.REL_X,
        uinput.REL_Y,
        uinput.ABS_X + (0,255,0,0),
        uinput.ABS_Y + (0,255,0,0),
        uinput.BTN_LEFT,
        uinput.BTN_RIGHT,
        )

    device = uinput.Device(events)

    for i in range(20):
        # syn=False to emit an "atomic" (5, 5) event.
        device.emit(uinput.ABS_X, 100, syn=False)
        device.emit(uinput.ABS_Y, 100)

        # Just for demonstration purposes: shows the motion. In real
        # application, this is of course unnecessary.
        time.sleep(0.01)

if __name__ == "__main__":
    main()