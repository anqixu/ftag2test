#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf
import math


class AprilTagQuat2RotNode:
  def __init__(self):
    rospy.init_node('apriltag_quat_2_rxyz_node')
    self.sub_tags = rospy.Subscriber('/apriltag/pose_cam', PoseStamped, self.processPoseMsg)
    rospy.loginfo('%s initialized' % rospy.get_name())
    
   
  def processPoseMsg(self, msg):
    IDs = eval(msg.header.frame_id)
    rospy.loginfo('Detected %d tags: (rx_deg, ry_deg, rz_deg, x_m, y_m, z_m, ID)' % len(IDs))
    count = 0
    for ID in IDs:
      count += 1
      rxyz = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
      rx = math.degrees(rxyz[0])+180.0
      if rx < -180.0:
        rx += 360.0
      elif rx > 180.0:
        rx -= 360.0
      rospy.loginfo('  %d: %.2f, %.2f, %.2f   (%.2f, %.2f, %.2f)   %s' % (count, rx, math.degrees(rxyz[1]), math.degrees(rxyz[2]), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, ID))
    rospy.loginfo('')


  def spin(self):
    rospy.spin()


if __name__ == "__main__":
  try:
    node = AprilTagQuat2RotNode()
    node.spin()
  except rospy.ROSInterruptException:
    pass
