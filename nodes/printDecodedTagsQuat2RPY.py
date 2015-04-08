#! /usr/bin/env python

import rospy
from ftag2_core.msg import TagDetections
import tf
import math


class Quat2RPYNode:
  def __init__(self):
    rospy.init_node('quat_2_rpy_node')
    self.sub_tags = rospy.Subscriber('/ftag2/detected_tags', TagDetections, self.processTagsMsg)
    rospy.loginfo('%s initialized' % rospy.get_name())
    
   
  def processTagsMsg(self, msg):
    rospy.loginfo('Detected %d tags: (roll_deg, pitch_deg, yaw_deg, x_m, y_m, z_m, ID)' % len(msg.tags))
    count = 0
    for tag in msg.tags:
      count += 1
      rpy = tf.transformations.euler_from_quaternion([tag.pose.orientation.x, tag.pose.orientation.y, tag.pose.orientation.z, tag.pose.orientation.w])
      rospy.loginfo('  %d: %.2f, %.2f, %.2f   (%.2f, %.2f, %.2f)   %s' % (count, math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2]), tag.pose.position.x, tag.pose.position.y, tag.pose.position.z, tag.decodedPayloadStr))
    rospy.loginfo('')

    
  def spin(self):
    rospy.spin()


if __name__ == "__main__":
  try:
    node = Quat2RPYNode()
    node.spin()
  except rospy.ROSInterruptException:
    pass
