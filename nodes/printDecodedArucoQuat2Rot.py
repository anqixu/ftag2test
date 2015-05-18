#! /usr/bin/env python

import rospy
from aruco_ros.msg import MarkerArray
import tf
import math


class ArucoQuat2RotNode:
  def __init__(self):
    rospy.init_node('aruco_quat_2_rot_node')
    self.sub_tags = rospy.Subscriber('/aruco/markers', MarkerArray, self.processTagsMsg)
    rospy.loginfo('%s initialized' % rospy.get_name())
    
   
  def processTagsMsg(self, msg):
    rospy.loginfo('Detected %d tags: (rx_deg, ry_deg, rz_deg, x_m, y_m, z_m, ID)' % len(msg.markers))
    count = 0
    for marker in msg.markers:
      count += 1
      rxyz = tf.transformations.euler_from_quaternion([marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w])
      rospy.loginfo('  %d: %.2f, %.2f, %.2f   (%.2f, %.2f, %.2f)   %s' % (count, math.degrees(rxyz[0]), math.degrees(rxyz[1]), math.degrees(rxyz[2]), marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, marker.id))
    rospy.loginfo('')


  def spin(self):
    rospy.spin()


if __name__ == "__main__":
  try:
    node = ArucoQuat2RotNode()
    node.spin()
  except rospy.ROSInterruptException:
    pass
