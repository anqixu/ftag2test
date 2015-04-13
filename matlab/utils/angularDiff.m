function d = angularDiff(a, b, range)
% Angular difference of b from a

if nargin < 3,
  range = 360.0;
end

d = b - a + range/2;
pos_idx = (d>0);
d(pos_idx) = d(pos_idx) - floor(d(pos_idx)/range)*range - range/2;
d(~pos_idx) = d(~pos_idx) - (floor(d(~pos_idx)/range) + 1)*range + range/2;

end
