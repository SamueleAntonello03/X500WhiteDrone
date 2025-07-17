from time import sleep
import sys
import os
import signal

current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK


def test():

    
    
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
   
    if not cam.connect():
        exit(1)
    
    
    cam.setGimbalRotation(45.0,-45.0)
    sleep(5)
    cam.setGimbalRotation(0.0,0.0)
    sleep(5)

    val = cam.requestZoomIn()
    sleep(1)
    val = cam.requestZoomHold()
    sleep(1)
    print("Zoom level: ", cam.getZoomLevel())

    val = cam.requestZoomOut()
    sleep(1)
    val = cam.requestZoomHold()
    sleep(1)
    print("Zoom level: ", cam.getZoomLevel())
 
    cam.setGimbalRotation(-45.0,25.0) 
    sleep(5)
    cam.setGimbalRotation(0.0,0.0)
    sleep(5)

    cam.disconnect()
    os.kill(os.getpid(),signal.SIGKILL)

if __name__ == "__main__":
    test()
    
