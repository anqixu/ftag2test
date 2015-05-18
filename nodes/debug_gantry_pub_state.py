#!/usr/bin/env python  
import roslib
roslib.load_manifest('ftag2test')
import rospy
import sys
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

if __name__ == "__main__":
  nargin = len(sys.argv)
  if nargin != 7:
    print 'Usage: %s <tx> <ty> <tz> <wrist_roll> <wrist_pivot> <flange_roll>'
    sys.exit(0)

  rospy.init_node('test')
  pub = rospy.Publisher('/state', Float64MultiArray, queue_size=10)
  dim = MultiArrayDimension()
  dim.label = 'width'
  dim.size = 6
  dim.stride = 6
  msg = Float64MultiArray()
  msg.layout.data_offset = 0
  msg.layout.dim.append(dim)
  msg.data = [float(v) for v in sys.argv[1:]]
  rospy.sleep(0.1)
  pub.publish(msg)
  rospy.sleep(0.1)
  pub.publish(msg)
