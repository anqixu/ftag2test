function [xyzw] = tf_quaternion_from_euler(roll_rad, pitch_rad, yaw_rad, axes)
% Replicating tf.transformations.py's interface

if nargin < 4,
  axes = 'xyz'; % implicitly assuming static frame, a.k.a. 'sxyz'
end

quat = quaternion.eulerangles(axes, [roll_rad, pitch_rad, yaw_rad]);
xyzw = [quat.e(2:4); quat.e(1)]';

end
