function [angle_deg] = tf_angle_between_euler_poses( ...
  rx1_deg, ry1_deg, rz1_deg, rx2_deg, ry2_deg, rz2_deg, axes1, axes2, nvec)
% Compute angle between normal vectors of 2 different Euler-angle poses

if nargin < 7,
  axes1 = 'xyz';
end
if nargin < 8,
  axes2 = 'xyz';
end
if nargin < 9,
  nvec = [0;0;1];
end

rad = pi/180;

if length(rx1_deg) > 1,
  angle_deg = zeros(size(rx1_deg));
  for i = 1:length(rx1_deg),
    q1 = quaternion.eulerangles(axes1, [rx1_deg(i)*rad, ry1_deg(i)*rad, rz1_deg(i)*rad]);
    q2 = quaternion.eulerangles(axes2, [rx2_deg(i)*rad, ry2_deg(i)*rad, rz2_deg(i)*rad]);
    v1 = RotationMatrix(q1)*nvec;
    v2 = RotationMatrix(q2)*nvec;
    angle_deg(i) = real(acosd(v1'*v2));
  end
else
  q1 = quaternion.eulerangles(axes1, [rx1_deg*rad, ry1_deg*rad, rz1_deg*rad]);
  q2 = quaternion.eulerangles(axes2, [rx2_deg*rad, ry2_deg*rad, rz2_deg*rad]);
  v1 = RotationMatrix(q1)*nvec;
  v2 = RotationMatrix(q2)*nvec;
  angle_deg = real(acosd(v1'*v2));
end

end
