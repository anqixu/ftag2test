function [rpy_rad] = tf_euler_from_quaternion(xyzw, axes)
% Replicating tf.transformations.py's interface

if nargin < 4,
  axes = 'xyz'; % implicitly assuming static frame, a.k.a. 'sxyz'
end

xyzw = xyzw(:);
quat = quaternion([xyzw(4); xyzw(1:3)]);
rpy_rad = quat.EulerAngles(axes)';

end
