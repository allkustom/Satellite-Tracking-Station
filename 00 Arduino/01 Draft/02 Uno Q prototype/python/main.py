import time

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI

arduino_ready = False


print("[Python] Init]")



def callSign():
    print("Called by Arduino")
    
Bridge.provide("Call", callSign)


def receive_ready(ready):
    global arduino_ready
    arduino_ready = ready
    # if ready:
    #     print("Arduino Ready, send coordinate")
    #     callCoord()

Bridge.provide("callReady", receive_ready)

def callCoord():
    coordX = 45.0
    coordY = 45.0
    coordZ = 45.0

    print(f"[Python] Target Coord: ({coordX}, {coordY}, {coordZ})")
    Bridge.call("pythonInput", coordX, coordY, coordZ)


def loop():
    global arduino_ready
    # pass
    if arduino_ready:
        callCoord()
        arduino_ready = False
    else:
        time.sleep(0.05)

# web_ui = WebUI()


App.run(user_loop=loop)
