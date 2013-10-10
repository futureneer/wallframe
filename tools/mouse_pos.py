# mousepos.py (linux only)
"""module mousepos
"""
# uses the package python-xlib
# from http://snipplr.com/view/19188/mouseposition-on-linux-via-xlib/

from Xlib import display
import time

def mousepos():
    """mousepos() --> (x, y) get the mouse coordinates on the screen (linux, Xlib)."""
    data2 = display.Display().screen().root.query_pointer()._data
    return data2["root_x"], data2["root_y"]

if __name__ == "__main__":

    for i in range(1, 50):

        pos=mousepos()
        print pos[0]
        print pos[1]
        time.sleep (1)