%% Reset workspace

clear all;
INIT_WORKSPACE;

%% Specify configuration and initialize constants
dataset_dir = '../ftag2_datasets/';
tag_width_px = 400;
tag_with_border_width_px = 500;
num_random_tags = 1000;
rng_seed = 1729;

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

%% Specify payloads for manually-sampled tags for 6S2F22B family
% NOTE: assuming that slice index / position does not significantly affect
%       error in encoded phase, we only need to test a few tag patterns
manual_set_payload_bits = { ...
  [0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0], ...
  [1 1 1; 1 1 1; 1 1 1; 0 0 0; 0 0 0; 0 0 0], ...
  [1 1 1; 0 0 0; 0 0 1; 0 1 0; 0 1 1; 0 0 0], ...
  [0 0 0; 1 0 0; 1 0 1; 1 1 0; 1 1 1; 1 1 1], ...
  };

%% Specify payloads for random-sampled tags for 6S2F22B family
all_IDs_perm = randperm(2^18); % WARNING: enumeration will be slow depending on number of bits being encoded!
random_set_payload_bits = cell(1, num_random_tags);
for random_tag_idx = 1:num_random_tags,
  ID = all_IDs_perm(random_tag_idx);
  random_set_payload_bits{random_tag_idx} = uint8(reshape(dec2bin(ID, 18), 3, 6)')-uint8('0');
end

%% Generate manually-sampled tag images
manual_dataset_dir = sprintf('%s/6S2F22B_manual_set/', dataset_dir);

if exist(manual_dataset_dir, 'dir') ~= 7,
  mkdir(manual_dataset_dir);
  fprintf('Created folder: %s\n', manual_dataset_dir);
end

fprintf('Generating %d manually-sampled tags...\n', length(manual_set_payload_bits));

for idx = 1:length(manual_set_payload_bits),
  [tag, phase_ids] = genFTag2Marker6S2F22B(manual_set_payload_bits{idx}, tag_width_px);
  tagPadded = padTagBorder(tag, tag_with_border_width_px);
  imwrite(tagPadded, genTagFilename(manual_dataset_dir, 'ftag2_6s2f22b', phase_ids));
end

%% Generate randomly-sampled tag images
random_dataset_dir = sprintf('%s/6S2F22B_random_set/', dataset_dir);

if exist(random_dataset_dir, 'dir') ~= 7,
  mkdir(random_dataset_dir);
  fprintf('Created folder: %s\n', random_dataset_dir);
end

fprintf('Generating %d randomly-sampled tags...\n', length(random_set_payload_bits));

for idx = 1:length(random_set_payload_bits),
  [tag, phase_ids] = genFTag2Marker6S2F22B(random_set_payload_bits{idx}, tag_width_px);
  tagPadded = padTagBorder(tag, tag_with_border_width_px);
  imwrite(tagPadded, genTagFilename(random_dataset_dir, 'ftag2_6s2f22b', phase_ids));
end

fprintf('ALL DONE!\n');
