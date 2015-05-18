#! /usr/bin/env python

import rospy
from ftag2_core.msg import TagDetections
import tf
import math
import numpy


class FTag2Quat2RotNode:
  def __init__(self):
    rospy.init_node('ftag2_quat_2_rxyz_node')
    self.sub_tags = rospy.Subscriber('/ftag2/detected_tags', TagDetections, self.processTagsMsg)
    rospy.loginfo('%s initialized' % rospy.get_name())
    
   
  def processTagsMsg(self, msg):
    rospy.loginfo('Detected %d tags: world(rx_deg, ry_deg, rz_deg)   world[x_m, y_m, z_m]  tag{pitch, yaw, roll}, ID' % len(msg.tags))
    count = 0
    for tag in msg.tags:
      count += 1
      rxyz = tf.transformations.euler_from_quaternion([tag.pose.orientation.x, tag.pose.orientation.y, tag.pose.orientation.z, tag.pose.orientation.w])
      tag_pyr = tf.transformations.euler_from_quaternion([tag.pose.orientation.x, tag.pose.orientation.y, tag.pose.orientation.z, tag.pose.orientation.w], 'rxyz') # pitch/yaw/roll in tag coordinate frame
      rospy.loginfo(' %d: (%.2f, %.2f, %.2f)   [%.2f, %.2f, %.2f]   {%.2f, %.2f, %.2f}   %s' % (count, math.degrees(rxyz[0]), math.degrees(rxyz[1]), math.degrees(rxyz[2]), tag.pose.position.x, tag.pose.position.y, tag.pose.position.z, math.degrees(tag_pyr[0]), math.degrees(tag_pyr[1]), math.degrees(tag_pyr[2]), tag.decodedPayloadStr))
      
    rospy.loginfo('')

    
  def spin(self):
    rospy.spin()


if __name__ == "__main__":
  try:
    node = FTag2Quat2RotNode()
    node.spin()
  except rospy.ROSInterruptException:
    pass
