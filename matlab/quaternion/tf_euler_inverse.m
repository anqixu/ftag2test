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

% Validation that quaternion inverse is same as taking matrix inverse of
% rotation matrix
%{
near_I = quat.RotationMatrix * quat_inv.RotationMatrix;
I = eye(3);
err_magn = sum((near_I(:) - I(:)).^2);
assert(err_magn < 1e-10);
%}

rxyz_inv_rad = quat_inv.EulerAngles(axes_inv);
rx_inv_rad = rxyz_inv_rad(1);
ry_inv_rad = rxyz_inv_rad(2);
rz_inv_rad = rxyz_inv_rad(3);

end
