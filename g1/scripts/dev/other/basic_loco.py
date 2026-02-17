import time
import sys
import os
sys.path.append("/home/ag/ef_ws/g1/scripts/dev")
from ef_client import Robot

g1 = Robot("enp1s0")

g1.stop()
g1.headlight(color="red", duration=10)
g1.say("Testing. Please be careful!")
g1.say("running forward")
g1.walk_for(1)
#time.sleep(0.33)
g1.stop()
time.sleep(0.5)
g1.say("turning")
g1.turn_for(90)
#time.sleep(3.14)
g1.stop()
time.sleep(0.5)
g1.say("walking")
g1.walk_for(1)
#time.sleep(0.33)
g1.stop()
time.sleep(5)
"""g1.say("testing arm motion, please step away")
g1.rotate_joint("elbow",160)
g1.rotate_joint("elbow",160,arm="left")
g1.rotate_joint("shoulder_pitch",20)
g1.rotate_joint("shoulder_pitch",20,arm="left")
g1.rotate_joint("elbow", -160)
g1.rotate_joint("elbow", -160, arm="left")
g1.rotate_joint("shoulder_yaw", 30)
g1.rotate_joint("shoulder_roll", 30, arm="left")
g1.say("please put me on the support frame and hold my hands then press enter to go to damping mode")
enterkey=input("press enter")
#if enterkey=="":
	#g1.damp()
"""