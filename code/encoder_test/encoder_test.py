#!/usr/bin/python
import os
import time

while 1==1:
    os.system("cat /sys/devices/ocp.3/48302000.epwmss/48302180.eqep/position && cat /sys/devices/ocp.3/48304000.epwmss/48304180.eqep/position")
    time.sleep(3)
    print "-----"
