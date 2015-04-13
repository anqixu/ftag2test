function [tag_type_id, num_slices, num_freqs, bit_pattern, gt_phases] = parseTagTypeAndPhases(tag_type, payload_str)
  % Parse tag type
  if nargin < 2, % argument is a tag filename
    payload_str = tag_type;
    tag_type = regexpi(tag_type, '[0-9]s[0-9]f[0-9]+b', 'match');
    if isempty(tag_type),
      error('parseGroundTruth:TagType', 'Unrecognized tag_type in filename: %s', tag_type);
    elseif iscell(tag_type) && length(tag_type) ~= 1,
      error('parseGroundTruth:TagType', 'Multiple tag_type found in filename: %s', tag_type);
    else
      tag_type = tag_type{1};
    end
  end
      
  if strcmpi(tag_type, '6S2F21B'),
    tag_type_id = 6221;
  elseif strcmpi(tag_type, '6S2F22B'),
    tag_type_id = 6222;
  elseif strcmpi(tag_type, '6S3F211B'),
    tag_type_id = 63211;
  elseif strcmpi(tag_type, '6S5F3B'),
    tag_type_id = 653;
  elseif strcmpi(tag_type, '6S5F33322B'),
    tag_type_id = 6533322;
  else
    error('parseGroundTruth:TagType', 'Unrecognized tag_type, expecting format #S#F#<...>B: %s', tag_type);
  end
  num_slices = floor(tag_type_id/10^floor(log10(tag_type_id)));
  num_freqs = mod(floor(tag_type_id/10^floor(log10(tag_type_id)-1)), 10);
  bit_pattern_id = mod(tag_type_id, 10^(floor(log10(tag_type_id))-1));
  if bit_pattern_id < 10 && num_freqs > 1,
    bit_pattern = ones(1, num_freqs)*bit_pattern_id;
  else
    bit_pattern = arrayfun(@(c) str2double(c), mat2str(bit_pattern_id));
  end
      
  % Parse payload string
  payload_expr = '';
  for s_i = 1:num_slices,
    for f_j = 1:num_freqs,
      payload_expr = strcat(payload_expr, '[0-9]');
    end
    if s_i < num_slices,
      payload_expr = strcat(payload_expr, '_');
    end
  end
  gt_payload_str = regexpi(payload_str, payload_expr, 'match');
  if isempty(gt_payload_str),
    error('parseGroundTruth:PayloadStr', 'Unrecognized payload_str, expecting format ##_##_...: %s', payload_str);
  elseif iscell(gt_payload_str) && length(gt_payload_str) ~= 1,
    error('parseGroundTruth:PayloadStr', 'Multiple payload_str found: %s', payload_str);
  else
    gt_payload_str = gt_payload_str{1};
  end
  bit_chunks_str = strrep(gt_payload_str, '_', '');
  bit_chunks_vec = arrayfun(@(c) str2double(c), bit_chunks_str);
  bit_chunks = reshape(bit_chunks_vec, num_freqs, num_slices)';
  gt_phases = bit_chunks .* repmat(360./(2.^bit_pattern), num_slices, 1);
end
