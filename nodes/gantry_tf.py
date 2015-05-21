#!/usr/bin/env python
import sys
import math
import numpy as np
import tf.transformations as tft


RADIANS = math.pi/180.0
TZ_FLANGE_FROM_HAND_M = -0.07672 # 3 inches
TZ_TAG_FROM_FLANGE_M = -0.035 # distance between tip of flange and tag (on Ebook screen)
RZ_TAG_FROM_FLANGE_DEG = 135.0


# Return 4x4 homogeneous transformation matrix M such that: pt_in_gantry_frame = M.dot(pt_in_tag_frame)
def M_from_state(state):
  M_wrist_from_gantry = tft.translation_matrix((state[0], state[1], state[2]))
  M_wrist_twisted_from_wrist = tft.quaternion_matrix(tft.quaternion_from_euler(0, 0, -state[3]*RADIANS))
  M_hand_from_wrist_twisted = tft.quaternion_matrix(tft.quaternion_from_euler(0, state[4]*RADIANS, 0))
  M_flange_from_hand = tft.translation_matrix((0, 0, TZ_FLANGE_FROM_HAND_M)).dot(tft.quaternion_matrix(tft.quaternion_from_euler(0, 0, -state[5]*RADIANS)))
  M_tag_from_flange = tft.translation_matrix((0, 0, TZ_TAG_FROM_FLANGE_M)).dot(tft.quaternion_matrix(tft.quaternion_from_euler(0, 0, RZ_TAG_FROM_FLANGE_DEG*RADIANS)))
    
  M_tag_from_gantry = M_wrist_from_gantry
  M_tag_from_gantry = M_tag_from_gantry.dot(M_wrist_twisted_from_wrist)
  M_tag_from_gantry = M_tag_from_gantry.dot(M_hand_from_wrist_twisted)
  M_tag_from_gantry = M_tag_from_gantry.dot(M_flange_from_hand)
  M_tag_from_gantry = M_tag_from_gantry.dot(M_tag_from_flange)
  
  return M_tag_from_gantry


# Returns 1x3 vector T = (tx, ty, tz), corresponding to position of tag in gantry's frame
def position_from_state(state):
  M_tag_from_gantry = M_from_state(state)
  return tft.translation_from_matrix(M_tag_from_gantry)

# Returns tuple (T, Q), where T = (tx, ty, tz) and Q = (qx, qy, qz, qw), corresponding to
# the translation vector and the quaternion rotation vector of the tag's pose in the
# gantry's frame
def pose_from_state(state):
  M_tag_from_gantry = M_from_state(state)
  Txyz = tft.translation_from_matrix(M_tag_from_gantry)
  Qxyzw = tft.quaternion_from_matrix(M_tag_from_gantry)
  return (Txyz, Qxyzw)


# Returns 4-tuple (T_top_left, T_bottom_left, T_bottom_right, T_top_right) of
# 1x3 vectors T_i = (tx_i, ty_i, tz_i), corresponding to the positions of
# the 4 tag corners, in gantry's frame
def tag_corner_poses_from_state(state, tag_width_m):
  tag_half_width_m = tag_width_m/2.0
  
  M_tag_from_gantry = M_from_state(state)
  
  # NOTE: tag frame's conventions are as follows:
  # +x = right of image
  # +y = bottom of image
  # +z = into image (following right-handed coordinate frame)
  T = M_tag_from_gantry.dot([-tag_half_width_m, -tag_half_width_m, 0, 1])
  T_top_left = (T[:3]/T[3])
  
  T = M_tag_from_gantry.dot([-tag_half_width_m, tag_half_width_m, 0, 1])
  T_bottom_left = (T[:3]/T[3])
  
  T = M_tag_from_gantry.dot([tag_half_width_m, tag_half_width_m, 0, 1])
  T_bottom_right = (T[:3]/T[3])
  
  T = M_tag_from_gantry.dot([tag_half_width_m, -tag_half_width_m, 0, 1])
  T_top_right = (T[:3]/T[3])
  
  return (T_top_left, T_bottom_left, T_bottom_right, T_top_right)
  

# Test logic
if __name__ == '__main__':
  if len(sys.argv) != 7:
    print 'Usage: %s <tx> <ty> <tz> <wrist_roll> <wrist_pivot> <flange_roll>'
    sys.exit(0)
  
  state = [float(num) for num in sys.argv[1:]]
  #print M_from_state(state)
  #print position_from_state(state)
  #print pose_from_state(state)
  print tag_corner_poses_from_state(state, 0.1)
