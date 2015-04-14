function [rx_inv_rad, ry_inv_rad, rz_inv_rad] = tf_euler_inverse(rx_rad, ry_rad, rz_rad, axes, axes_inv)
%TF_EULER_INVERSE Compute Euler angles after inverting static/moving frames

if nargin < 4,
  axes = 'xyz';
end
if nargin < 5,
  axes_inv = 'xyz';
end

quat = quaternion.eulerangles(axes, [rx_rad, ry_rad, rz_rad]);
quat_inv = quat.inverse();
rxyz_inv_rad = quat_inv.EulerAngles(axes_inv);
rx_inv_rad = rxyz_inv_rad(1);
ry_inv_rad = rxyz_inv_rad(2);
rz_inv_rad = rxyz_inv_rad(3);

end
