#!/usr/bin/env python  
import roslib
roslib.load_manifest('ftag2test')
import rospy
import tf
import math
from std_msgs.msg import Float64MultiArray


radians = math.pi/180.0


class GantryTF():
  def __init__(self):
    rospy.init_node('gantry_tf')
    self.state = [0, 0, 0, 0, 0, 0]
    self.tf_broadcaster = tf.TransformBroadcaster()
    self.state_sub = rospy.Subscriber('/state', Float64MultiArray, self.handleState)
    rospy.loginfo('gantry tf initialized')


  def handleState(self, msg):
    if len(msg.data) != 6:
      rospy.logerr('Expecting 6 entries in state, received %d' % len(msg.data))
      return
    rospy.loginfo('Received state: %s' % str(msg.data))
    #self.publishTF(msg.data)
    self.state = msg.data
    
  
  def publishTF(self, state):
    self.tf_broadcaster.sendTransform((-1.15, -1.15, 0.3),
      tf.transformations.quaternion_from_euler(0, 0, 0),
      rospy.Time.now(),
      "gantry", "world")
    self.tf_broadcaster.sendTransform((state[0], state[1], state[2]),
      tf.transformations.quaternion_from_euler(0, 0, 0),
      rospy.Time.now(),
      "wrist", "gantry")
    self.tf_broadcaster.sendTransform((0, 0, 0),
      tf.transformations.quaternion_from_euler(0, 0, -state[3]*radians),
      rospy.Time.now(),
      "wrist_twisted", "wrist")
    self.tf_broadcaster.sendTransform((0, 0, 0),
      tf.transformations.quaternion_from_euler(0, state[4]*radians, 0),
      rospy.Time.now(),
      "hand", "wrist_twisted")
    self.tf_broadcaster.sendTransform((0, 0, -0.07672),
      tf.transformations.quaternion_from_euler(0, 0, -state[5]*radians),
      rospy.Time.now(),
      "flange", "hand")
    self.tf_broadcaster.sendTransform((0, 0, -0.035),
      tf.transformations.quaternion_from_euler(0, 0, 135*radians),
      rospy.Time.now(),
      "tag", "flange")

    
  def spin(self):
    hz = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.publishTF(self.state)
      hz.sleep()


if __name__ == '__main__':
  gantry = GantryTF()
  gantry.spin()
