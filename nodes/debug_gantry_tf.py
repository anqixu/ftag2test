#!/usr/bin/env python  
import roslib
roslib.load_manifest('ftag2test')
import rospy
import tf
import gantry_tf as gtf
import math
import threading
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped



RADIANS = math.pi/180.0


class GantryTF():
  def __init__(self):
    rospy.init_node('gantry_tf')
    self.state_idx = -float('inf')
    self.state = [0, 0, 0, 0, 0, 0]
    self.state_mutex = threading.Lock()
    self.tf_broadcaster = tf.TransformBroadcaster()
    self.tf_listener = tf.TransformListener()
    self.state_sub = rospy.Subscriber('/gantry_state', Float64MultiArray, self.handleState)
    self.pose_pub = rospy.Publisher('/gantry_pose', PoseStamped, queue_size=10)
    self.pose2_pub = rospy.Publisher('/gantry_pose_manual', PoseStamped, queue_size=10)
    rospy.loginfo('gantry tf initialized')


  def handleState(self, msg):
    if len(msg.data) != 7:
      rospy.logerr('Expecting 7 entries in state, received %d' % len(msg.data))
      return
    rospy.loginfo('Received state: %s' % str(msg.data))
    with self.state_mutex:
      self.state_idx = int(msg.data[0])
      self.state = msg.data[1:]
    
  
  def publishTF(self, state_idx, state):
    self.tf_broadcaster.sendTransform((-1.15, -1.15, 0.3),
      tf.transformations.quaternion_from_euler(0, 0, 0),
      rospy.Time.now(),
      "gantry", "world")
    self.tf_broadcaster.sendTransform((state[0], state[1], state[2]),
      tf.transformations.quaternion_from_euler(0, 0, 0),
      rospy.Time.now(),
      "wrist", "gantry")
    self.tf_broadcaster.sendTransform((0, 0, 0),
      tf.transformations.quaternion_from_euler(0, 0, -state[3]*RADIANS),
      rospy.Time.now(),
      "wrist_twisted", "wrist")
    self.tf_broadcaster.sendTransform((0, 0, 0),
      tf.transformations.quaternion_from_euler(0, state[4]*RADIANS, 0),
      rospy.Time.now(),
      "hand", "wrist_twisted")
    self.tf_broadcaster.sendTransform((0, 0, -0.07672),
      tf.transformations.quaternion_from_euler(0, 0, -state[5]*RADIANS),
      rospy.Time.now(),
      "flange", "hand")
    self.tf_broadcaster.sendTransform((0, 0, -0.035),
      tf.transformations.quaternion_from_euler(0, 0, 135*RADIANS),
      rospy.Time.now(),
      "tag", "flange")
      
    try:
      msg = PoseStamped()
      msg.header.seq = state_idx
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = 'tag'
      (trans, rot) = self.tf_listener.lookupTransform("gantry", "tag", rospy.Time(0))
      msg.pose.position.x = trans[0]
      msg.pose.position.y = trans[1]
      msg.pose.position.z = trans[2]
      msg.pose.orientation.x = rot[0]
      msg.pose.orientation.y = rot[1]
      msg.pose.orientation.z = rot[2]
      msg.pose.orientation.w = rot[3]
      self.pose_pub.publish(msg)
      
      (trans2, rot2) = gtf.pose_from_state(state)
      msg.pose.position.x = trans2[0]
      msg.pose.position.y = trans2[1]
      msg.pose.position.z = trans2[2]
      msg.pose.orientation.x = rot2[0]
      msg.pose.orientation.y = rot2[1]
      msg.pose.orientation.z = rot2[2]
      msg.pose.orientation.w = rot2[3]
      self.pose2_pub.publish(msg)
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.logerr('TF transformation exception')
      pass


  def spin(self):
    hz = rospy.Rate(100)
    while not rospy.is_shutdown():
      with self.state_mutex:
        state_idx = self.state_idx
        state = self.state
      self.publishTF(state_idx, state)
      hz.sleep()


if __name__ == '__main__':
  gantry = GantryTF()
  gantry.spin()
