#!/usr/bin/env python3

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float32

def fixName(input):
  out = ''
  for i in input:
    if i in ('.', ':', '-'):
      out += '_'
    else:
      out += i
  return out

class RateTracker:
  def __init__(self):
    self.last_value = None
    self.last_value_time = None

  def addValue(self, time, value):
    ret = None
    if value != self.last_value and self.last_value_time is not None:
      dv = value - self.last_value
      dt = time - self.last_value_time
      d = rospy.Duration()
      ret = dv/dt.to_sec()
    self.last_value = value
    self.last_value_time = time
    return ret

trackers = {}
publishers = {}

def diagCallback(msg):
  for s in msg.status:
    for kv in s.values:
      if 'HCInOctets' in kv.key or 'HCOutOctets' in kv.key:
        #print (s.name, kv.key, kv.value)
        if not s.name in trackers:
          trackers[s.name] = {}
        if not kv.key in trackers[s.name]:
          trackers[s.name][kv.key] = RateTracker()
        rate = trackers[s.name][kv.key].addValue(msg.header.stamp, float(kv.value))
        if rate is not None:
          if not s.name in publishers:
            publishers[s.name] = {}
          if not kv.key in publishers[s.name]:
            publishers[s.name][kv.key] = rospy.Publisher(fixName("wifi_rates/"+s.name+"/"+kv.key), Float32, queue_size=10)
          publishers[s.name][kv.key].publish(rate)


if __name__ == '__main__':
  rospy.init_node("wifi_rates")
  rospy.Subscriber("/diagnostics", DiagnosticArray, diagCallback)
  rospy.spin()