from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
import time

ChannelFactoryInitialize(0, "enp1s0")  # change iface if needed

def cb(msg):
  print("slam_info:", str(msg.data)[:200])

sub = ChannelSubscriber("rt/slam_info", String_)
sub.Init(cb, 10)

time.sleep(5)
