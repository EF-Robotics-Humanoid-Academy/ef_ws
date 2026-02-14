import time
import sys
import os

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..")))
from safety.hanger_boot_sequence import hanger_boot_sequence

client = hanger_boot_sequence(iface=sys.argv[1] if len(sys.argv) > 1 else "eth0")

# Walk forward ~1m (0.3 m/s x 3.3s)
client.Move(0.3, 0, 0)
time.sleep(3.3)
client.StopMove()
time.sleep(0.5)

# Turn ~90 deg (0.5 rad/s x 3.14s = pi/2 rad)
client.Move(0, 0, 0.5)
time.sleep(3.14)
client.StopMove()
time.sleep(0.5)

# Walk forward ~1m
client.Move(0.3, 0, 0)
time.sleep(3.3)
client.StopMove()
