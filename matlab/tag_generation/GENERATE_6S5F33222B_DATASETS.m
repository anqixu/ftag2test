%% Reset workspace

clear all;
INIT_WORKSPACE;

%% Specify configuration and initialize constants
dataset_dir = '../ftag2_datasets/';
tag_width_px = 500;
tag_with_border_width_px = 500;
num_random_tags = 1000;
rng_seed = 1730;

rng(rng_seed);

genTagFilename = @(destDir, fileHeader, phase_ids) ...
  sprintf('%s/%s_%s_%s_%s_%s_%s_%s.png', destDir, fileHeader, ...
    sprintf('%d', phase_ids(1, :)), ...
    sprintf('%d', phase_ids(2, :)), ...
    sprintf('%d', phase_ids(3, :)), ...
    sprintf('%d', phase_ids(4, :)), ...
    sprintf('%d', phase_ids(5, :)), ...
    sprintf('%d', phase_ids(6, :)));

if exist(dataset_dir, 'dir') ~= 7,
  mkdir(dataset_dir);
  fprintf('Created folder: %s\n', dataset_dir);
end

%% Specify payloads for manually-sampled tags for 6S5F33222B family
% Since 2^11 bits is too large, even when divided by 6S, we must resort
% to a few randomly-sampled tags for sweep
manual_set_payload_bits = { ...
  zeros(6, 11), ...
  [ones(3, 11); zeros(3, 11)], ...
  round(rand(6, 11)), ...
  round(rand(6, 11)), ...
  round(rand(6, 11)), ...
  };

%% Specify payloads for random-sampled tags for 6S5F33222B family
random_set_payload_bits_dict = containers.Map('KeyType', 'char', 'ValueType', 'logical');
random_set_payload_bits = cell(1, num_random_tags);
curr_random_tag_idx = 1;
while curr_random_tag_idx <= num_random_tags,
  payload_bits = uint8(round(rand(6, 11)));
  payload_str = sprintf('%d', payload_bits(:)');
  if ~random_set_payload_bits_dict.isKey(payload_str),
    random_set_payload_bits_dict(payload_str) = true;
    random_set_payload_bits{curr_random_tag_idx} = payload_bits;
    curr_random_tag_idx = curr_random_tag_idx + 1;
  end
end

%% Generate manually-sampled tag images
manual_dataset_dir = sprintf('%s/6S5F33222B_manual_set/', dataset_dir);

if exist(manual_dataset_dir, 'dir') ~= 7,
  mkdir(manual_dataset_dir);
  fprintf('Created folder: %s\n', manual_dataset_dir);
end

fprintf('Generating %d manually-sampled tags...\n', length(manual_set_payload_bits));

for idx = 1:length(manual_set_payload_bits),
  [tag, phase_ids] = genFTag2Marker6S5F33222B(manual_set_payload_bits{idx}, tag_width_px);
  tagPadded = padTagBorder(tag, tag_with_border_width_px);
  imwrite(tagPadded, genTagFilename(manual_dataset_dir, 'ftag2_6s5f33222b', phase_ids));
end

%% Generate randomly-sampled tag images
random_dataset_dir = sprintf('%s/6S5F33222B_random_set/', dataset_dir);

if exist(random_dataset_dir, 'dir') ~= 7,
  mkdir(random_dataset_dir);
  fprintf('Created folder: %s\n', random_dataset_dir);
end

fprintf('Generating %d randomly-sampled tags...\n', length(random_set_payload_bits));

for idx = 1:length(random_set_payload_bits),
  [tag, phase_ids] = genFTag2Marker6S5F33222B(random_set_payload_bits{idx}, tag_width_px);
  tagPadded = padTagBorder(tag, tag_with_border_width_px);
  imwrite(tagPadded, genTagFilename(random_dataset_dir, 'ftag2_6s5f33222b', phase_ids));
end

fprintf('ALL DONE!\n');
