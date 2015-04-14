function [tag, phase_ids] = genFTag2Marker6S5F3B(payload, tagWidthPx)
% [OUTDATED; SEE genFTag2Marker6S5F3BV2.m]
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

payload = (payload(:) ~= 0);
if length(payload) > 54,
  payload = payload(1:54);
end;

num = 0;
for i = 1:54,
  num = num*2;
  if payload(i) == 1,
    num = num + 1;
  end;
end;
fprintf('ID = %lu\n', num);

phase_bits = zeros(6, 5, 3);

% phase signature pattern
phase_bits(1, 1, 1) = 1;
phase_bits(2, 1, 1) = 0;
phase_bits(3, 1, 1) = 1;
phase_bits(4, 1, 1) = 0;
phase_bits(5, 1, 1) = 1;
phase_bits(6, 1, 1) = 0;

% CRC-12
crc12_h = crc.generator([1   1 0 0 0  1 1 1 1   1 0 0 0]);
crc_checksum = generate(crc12_h, payload);
crc_checksum = crc_checksum(length(payload)+1:end);
phase_bits(1, 1, 2) = crc_checksum(1);
phase_bits(1, 1, 3) = crc_checksum(2);
phase_bits(2, 1, 2) = crc_checksum(3);
phase_bits(2, 1, 3) = crc_checksum(4);
phase_bits(3, 1, 2) = crc_checksum(5);
phase_bits(3, 1, 3) = crc_checksum(6);
phase_bits(4, 1, 2) = crc_checksum(7);
phase_bits(4, 1, 3) = crc_checksum(8);
phase_bits(5, 1, 2) = crc_checksum(9);
phase_bits(5, 1, 3) = crc_checksum(10);
phase_bits(6, 1, 2) = crc_checksum(11);
phase_bits(6, 1, 3) = crc_checksum(12);

% XOR and grey code
sliced_payload = zeros(6, 9);
for s = 1:6,
  sliced_payload(s, :) = payload(s*9 - 8:s*9);
  
  chunk = sliced_payload(s, 1:3);
  phase_bits(s, 2, 1) = allxor(chunk);
  encoded_chunk = greycode(chunk, false);
  phase_bits(s, 3, 1) = encoded_chunk(1);
  phase_bits(s, 3, 2) = encoded_chunk(2);
  phase_bits(s, 3, 3) = encoded_chunk(3);
  
  chunk = sliced_payload(s, 4:6);
  phase_bits(s, 2, 2) = allxor(chunk);
  encoded_chunk = greycode(chunk, false);
  phase_bits(s, 4, 1) = encoded_chunk(1);
  phase_bits(s, 4, 2) = encoded_chunk(2);
  phase_bits(s, 4, 3) = encoded_chunk(3);
  
  chunk = sliced_payload(s, 7:9);
  phase_bits(s, 2, 3) = allxor(chunk);
  encoded_chunk = greycode(chunk, false);
  phase_bits(s, 5, 1) = encoded_chunk(1);
  phase_bits(s, 5, 2) = encoded_chunk(2);
  phase_bits(s, 5, 3) = encoded_chunk(3);
end;

% Convert to phases
phase_ids = phase_bits(:, :, 1)*4 + phase_bits(:, :, 2)*2 + phase_bits(:, :, 3);
phases = phase_ids*360/8;

% Synthesize tag image
tag = genFTag2MarkerFromPhases(phases, tagWidthPx);

end
