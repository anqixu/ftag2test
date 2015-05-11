function [tag, phase_ids] = genFTag2Marker6S2F22B(payload_bits, tagWidthPx)
% per slice: 2 bits for 1Hz (LSB used for sig.), 2 bits for 2Hz, 3*6 bits total payload
%
% DEMO USAGE:
%{
tag = genFTag2Marker6S2F22B([0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0]', 800);
tag = genFTag2Marker6S2F22B([0 0 1; 0 1 0; 0 1 1; 1 0 0; 1 0 1; 1 1 0]', 800);
imshow(tag, 'border', 'tight');
%}

% Validate payload argument
payload_unique = unique(payload_bits);
payload_unique(payload_unique == 0) = [];
payload_unique(payload_unique == 1) = [];
if any(payload_unique),
  fprintf('Found non-binary contents in specified payload_bits argument:');
  disp(payload_unique);
  error('Invalid payload_bits argument');
end

% Basic information about tag structure
num_slices = 6;
bits_per_freq = [2, 2];
num_freqs = length(bits_per_freq);
num_sig_bits_per_slice = 1;
num_raw_bits_per_slice = sum(bits_per_freq);
num_payload_bits_per_slice = num_raw_bits_per_slice - num_sig_bits_per_slice;
num_payload_bits = num_payload_bits_per_slice * num_slices;

% Format payload bits into column vector
payload_bits = (payload_bits ~= 0);
if size(payload_bits, 1) == 6 && size(payload_bits, 2) == 3,
  payload_bits = payload_bits';
end
if size(payload_bits, 1) > 1 && size(payload_bits, 2) > 1,
  payload_bits = reshape(payload_bits, num_payload_bits, 1);
end

phase_bits = zeros(num_slices, num_freqs, max(bits_per_freq));

% phase signature pattern
phase_bits(1, 1, 2) = 1;
phase_bits(2, 1, 2) = 0;
phase_bits(3, 1, 2) = 0;
phase_bits(4, 1, 2) = 0;
phase_bits(5, 1, 2) = 1;
phase_bits(6, 1, 2) = 1;

% Store actual payload
for s = 1:6,
  base_idx = (s-1)*num_payload_bits_per_slice;
  phase_bits(s, 1, 1) = payload_bits(base_idx + 1);
  phase_bits(s, 2, 1) = payload_bits(base_idx + 2);
  phase_bits(s, 2, 2) = payload_bits(base_idx + 3);
end;

% Convert to phases
phase_ids = zeros(num_slices, num_freqs);
phases = zeros(num_slices, num_freqs);
for f = 1:num_freqs,
  for b = 1:bits_per_freq(f),
    phase_ids(:, f) = 2*phase_ids(:, f) + phase_bits(:, f, b);
  end
  phases(:, f) = phase_ids(:, f)*360/(2^bits_per_freq(f));
end

% Synthesize tag image
tag = genFTag2MarkerFromPhases(phases, tagWidthPx);

end
