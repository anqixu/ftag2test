function [tag, phase_ids] = genFTag2Marker6S5F3BV2(payload_bits, tagWidthPx)
% V2 configuration:
% - 6 slices
% - 5 frequencies
% - @ 1Hz: sig bit, xor_2_3Hz bit, xor_4_5Hz bit
% - @ 2Hz: 3 bits payload
% - @ 3Hz: 3 bits payload
% - @ 4Hz: 2 bits payload
% - @ 5Hz: 2 bits payload
% - total payload: 60 bits
%
% - gray coding:
%   If payload phase bleeds over to next bin, its gray-coded variant
%   will have exactly 1 bit flipped. Therefore, the checksum is stored
%   as allxor(graycode(payload_chunk)), and is used to validate the
%   decoded phases.
%
% DEMO USAGE:
%
% tag = genFTag2MarkerV2([0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0]', 1200);
% tag = genFTag2MarkerV2([1 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1 1; 1 1 1 1 1 1 1 1 1 1]', 1200);
% tag = genFTag2MarkerV2([0 0 0 0 0 0 0 0 0 0; 1 1 1 1 1 1 1 1 1 1; 0 1 0 1 0 1 0 1 0 1; 1 1 0 0 1 1 0 0 1 1; 1 1 1 0 0 0 1 1 0 0; 0 0 0 0 0 0 0 0 0 1]', 1200);
%
% imshow(tag, 'border', 'tight');

payload_bits = (payload_bits ~= 0);
if size(payload_bits, 1) > 1 && size(payload_bits, 2) > 1,
  payload_bits = reshape(payload_bits, 60, 1);
end

phase_bits = zeros(6, 5, 3);

% phase signature pattern
phase_bits(1, 1, 1) = 1;
phase_bits(2, 1, 1) = 0;
phase_bits(3, 1, 1) = 0;
phase_bits(4, 1, 1) = 0;
phase_bits(5, 1, 1) = 1;
phase_bits(6, 1, 1) = 1;

% XOR and grey code
sliced_payload = zeros(6, 10);
for s = 1:6,
  sliced_payload(s, :) = payload_bits(s*10 - 9:s*10)';
  
  phase_bits(s, 1, 2) = allxor(greycode(sliced_payload(s, 1:6), false));
  phase_bits(s, 1, 3) = allxor(greycode(sliced_payload(s, 7:10), false));
  
  phase_bits(s, 2, 1) = sliced_payload(s, 1);
  phase_bits(s, 2, 2) = sliced_payload(s, 2);
  phase_bits(s, 2, 3) = sliced_payload(s, 3);

  phase_bits(s, 3, 1) = sliced_payload(s, 4);
  phase_bits(s, 3, 2) = sliced_payload(s, 5);
  phase_bits(s, 3, 3) = sliced_payload(s, 6);

  phase_bits(s, 4, 1) = sliced_payload(s, 7);
  phase_bits(s, 4, 2) = sliced_payload(s, 8);
  phase_bits(s, 4, 3) = 0;

  phase_bits(s, 5, 1) = sliced_payload(s, 9);
  phase_bits(s, 5, 2) = sliced_payload(s, 10);
  phase_bits(s, 5, 3) = 0;
end;

% Convert to phases
phase_ids = phase_bits(:, :, 1)*4 + phase_bits(:, :, 2)*2 + phase_bits(:, :, 3);
phases = phase_ids*360/8;

% Synthesize tag image
tag = genFTag2MarkerFromPhases(phases, tagWidthPx);

end
