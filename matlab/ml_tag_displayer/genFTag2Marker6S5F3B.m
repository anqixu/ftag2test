function [tag, phases] = genFTag2Marker6S5F3B(phase_bits, tagWidthPx)
% F = 3: 54 bits payload, 36 bits extra, 5 freqs total; uses CRC-12 (0x8F8)
%
% DEMO USAGE:
%
% tag = genFTag2Marker6S5F3B([0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0]', 1200);
% tag = genFTag2Marker6S5F3B([1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1]', 1200);
% tag = genFTag2Marker6S5F3B([0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 1; 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 1 1; 0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 1 0 1]', 1200);
% tag = genFTag2Marker6S5F3B([0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 1; 0 1 0 1 0 1 0 1 0; 1 0 1 0 1 0 1 0 1; 1 1 1 1 0 1 1 1 1; 1 1 1 1 1 1 1 1 1]', 1200);
% tag = genFTag2Marker6S5F3B([1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 0; 1 1 1 1 1 1 1 0 1; 1 1 1 1 1 1 1 0 0; 1 1 1 1 1 1 0 1 1; 1 1 1 1 1 1 0 1 0]', 1200);
% tag = genFTag2Marker6S5F3B([1 1 1 1 1 1 1 1 1; 1 1 1 1 0 1 1 1 1; 1 0 1 0 1 0 1 0 1; 0 1 0 1 0 1 0 1 0; 0 0 0 0 0 0 0 0 1; 0 0 0 0 0 0 0 0 0]', 1200);
% tag = genFTag2Marker6S5F3B([1 0 1 0 1 0 1 0 1; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0]', 1200);
% tag = genFTag2Marker6S5F3B([0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0; 1 0 1 0 1 0 1 0 1]', 1200);
%
% imshow(tag, 'border', 'tight');

% Convert to phases
phase_ids = phase_bits(:, :, 1)*4 + phase_bits(:, :, 2)*2 + phase_bits(:, :, 3);
phases = phase_ids*360/8;

N = tagWidthPx;
x = linspace(0, 360, N);
rays = zeros(size(phases, 1), N);
for i = 1:size(phases, 1),
  rays(i, :) = zeros(1, N);
  for f = 1:size(phases, 2),
    rays(i, :) = rays(i, :) + cosd(f*x + phases(i, f));
  end;
  mm = minmax(rays(i, :));
  rays(i, :) = (rays(i, :) - mm(1))/(mm(2) - mm(1));
end;

W = round(N*8/6);
tag = zeros(W, W);
range = (W/8+1):(W/8+N);
for r = 1:W,
  i = ceil(r/W*8);
  if i > 1 && i < 8,
    tag(r, range) = tag(r, range) + rays(i-1, :);
  end;
end;

end
