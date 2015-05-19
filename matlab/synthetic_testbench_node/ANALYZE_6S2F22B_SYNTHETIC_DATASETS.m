%% Reset workspace, then create or load datasets

clear all;
INIT_WORKSPACE;
close all;

trial_ids = {'sweep_tx', 'sweep_ty', 'sweep_tz', 'sweep_pitch', 'sweep_yaw', 'sweep_roll', 'random'};
sweep_feature_vars = {'tag_tx_m', 'tag_ty_m', 'tag_tz_m', 'tag_rx_deg', 'tag_ry_deg', 'tag_rz_deg'};
fig_i = 1;

dataset_dir = '../ftag2_datasets';
all_trials_file = fullfile(dataset_dir, 'trials', '6s2f22b_all_trials.processed.mat');

generate_tag_images = false;
synthesize_and_decode_scenes = false;
process_raw_trials = false;

if generate_tag_images,
  GENERATE_6S2F22B_DATASETS; %#ok<*UNRCH>
  synthesize_and_decode_scenes = true;
end

if synthesize_and_decode_scenes,
  input('Start "roslaunch ftag2test synthetic_testbench.launch", then press ENTER to continue');
  SYNTHESIZE_6S2F22B_MANUAL_DATASET;
  SYNTHESIZE_6S2F22B_RANDOM_DATASET;
  process_raw_trials = true;
end

if process_raw_trials,
  clear trials;
  for i = 1:length(trial_ids),
    ind_trial_file = sprintf('%s/trials/6s2f22b_%s.mat', dataset_dir, trial_ids{i});
    fprintf('\nPROCESSING %s...\n\n', ind_trial_file);
    load(ind_trial_file);
    trials.(trial_ids{i}) = processProgressSeq(progress_seq, trial_ids{i});
  end
  save(all_trials_file, 'trials');
  fprintf('\nFINISHED processing all raw files.\n');
else
  load(all_trials_file);
end

%% Apply filters to remove innaccurately detected poses
%
% * when tag is pitched sufficiently away (OOP > 70'), mag+phase signature
%      is mistakenly detected in rotated tag image;
%      simple fix is to threshold these border cases; cleaner fix might
%      require comparing magnitude spectrums for rotated and non-rotated
%      tag and determining better filtering / selection criteria
%      (maybe including phase error as well);
%      *payload phases are affected by this error!*
% * for tag-in-image with parallel edges, there are multiple 3D poses
%      that project down into these same tag-in-image pose (e.g. +15' pitch
%      vs -15' pitch);
%      possible resolutions: A - seed solvePnP with previous tracked pose,
%      B - reject excessively large pose jumps (in OOP angle) in tracker

APPLY_FILTER = true;

SWEEP_ANGLE_MIN_DEG = 2;
RXY_MAX_DEG = 9;
QUAD_CORNER_TO_IMAGE_BORDER_MIN_PX = 2;
PAYLOAD_STR_MAX_DIFF = 7;

% Initialize exclude_idx vectors
for trial_i = 1:length(trial_ids),
  trial_id = trial_ids{trial_i};
  trials.(trial_id).seqds_exclude_idx = false(size(trials.(trial_id).seqds, 1), 1);
  trials.(trial_id).ds_exclude_idx = false(size(trials.(trial_id).ds, 1), 1);
end

% Filter rx & ry sweep trials to remove multi-3D-pose ambiguities ...
% ... based on differences in sweep var
for trial_i = 1:(length(trial_ids) - 1), % skip random
  feature_var = sweep_feature_vars{trial_i};
  if isempty(strfind(feature_var, 'rx')) && ...
      isempty(strfind(feature_var, 'ry')),
    continue; % only filter rx and ry
  end
  if ~APPLY_FILTER,
    continue;
  end

  trial_id = trial_ids{trial_i};
  trial = trials.(trial_id);

  parallel_edged_quad_seqds_idx = (abs(trial.seqds.(feature_var)) > 2*SWEEP_ANGLE_MIN_DEG) & ...
  (abs(trial.seqds.(feature_var) - -trial.seqds.(strcat('ftag2_', feature_var(5:end)))) <= SWEEP_ANGLE_MIN_DEG);
  parallel_edged_quad_ds_idx = (abs(trial.ds.(feature_var)) > 2*SWEEP_ANGLE_MIN_DEG) & ...
  (abs(trial.ds.(feature_var) - -trial.ds.(strcat('ftag2_', feature_var(5:end)))) <= SWEEP_ANGLE_MIN_DEG);
  trials.(trial_id).seqds_exclude_idx = ...
    trials.(trial_id).seqds_exclude_idx | parallel_edged_quad_seqds_idx;
  trials.(trial_id).ds_exclude_idx = ...
    trials.(trial_id).ds_exclude_idx | parallel_edged_quad_ds_idx;
end

% Filter all trials to remove multi-3D-pose ambiguities ...
% based on rxy differences
if APPLY_FILTER,
  for trial_i = 1:length(trial_ids),
    trial_id = trial_ids{trial_i};
    trial = trials.(trial_id);
    parallel_edged_quad_seqds_idx = (trial.seqds.diff_rxy_deg >= RXY_MAX_DEG);
    parallel_edged_quad_ds_idx = (trial.ds.diff_rxy_deg >= RXY_MAX_DEG);
    trials.(trial_id).seqds_exclude_idx = ...
      trials.(trial_id).seqds_exclude_idx | parallel_edged_quad_seqds_idx;
    trials.(trial_id).ds_exclude_idx = ...
      trials.(trial_id).ds_exclude_idx | parallel_edged_quad_ds_idx;
  end
end

% Filter all trials to remove quads near image border, and payloads
% that greatly differ from expected
if APPLY_FILTER,
  for trial_i = 1:length(trial_ids),
    trial_id = trial_ids{trial_i};
    trial = trials.(trial_id);
    near_border_quad_seqds_idx = ...
      (trial.seqds.quad_border_mindist_px <= QUAD_CORNER_TO_IMAGE_BORDER_MIN_PX);
    poor_payload_seqds_idx = ...
      (trial.seqds.diff_payload_str_n >= PAYLOAD_STR_MAX_DIFF);
    near_border_quad_ds_idx = ...
      (trial.ds.quad_border_mindist_px <= QUAD_CORNER_TO_IMAGE_BORDER_MIN_PX);
    poor_payload_ds_idx = ...
      (trial.ds.diff_payload_str_n >= PAYLOAD_STR_MAX_DIFF);
    trials.(trial_id).seqds_exclude_idx = ...
      trials.(trial_id).seqds_exclude_idx | near_border_quad_seqds_idx | poor_payload_seqds_idx;
    trials.(trial_id).ds_exclude_idx = ...
      trials.(trial_id).ds_exclude_idx | near_border_quad_ds_idx | poor_payload_ds_idx;
  end
end

% Display mis-detection and multi-detections
fprintf('%11s:\t%9s\t%9s\t%9s\t%6s\n', 'TRIAL', '0-DET', 'N-DET', 'FILTERED', 'OBS');
for trial_i = 1:length(trial_ids),
  trial_id = trial_ids{trial_i};
  trial = trials.(trial_id);
  num_zero_det = sum(trial.seqds.ftag2_num_tags_detected == 0);
  num_multi_det = sum(trial.seqds.ftag2_num_tags_detected > 1);
  num_total_det = size(trial.seqds, 1);
  num_filtered_det = sum((trial.seqds.ftag2_num_tags_detected == 1) & trial.seqds_exclude_idx);
  num_obs = sum(~trial.ds_exclude_idx);
  fprintf('%11s:\t%4d (%2.0f%%)\t%4d (%2.0f%%)\t%4d (%2.0f%%)\t%6d\n', trial_id, ...
    num_zero_det, num_zero_det/num_total_det*100, ...
    num_multi_det, num_multi_det/num_total_det*100, ...
    num_filtered_det, num_filtered_det/num_total_det*100, ...
    num_obs);
end
fprintf('\n');

%% Plot 0-detections vs sweep trials
%
% *RESULTS:*
%
% * tx & ty: decoder fails when tag is moved out of viewport, as expected
% * tz: decoder fails when tag is too close to keep all in viewport;
%       expect black-border check to fail more and more as tag moves away
% * rx & ry (tag pitch / yaw, a.k.a. out-of-plane rotations):
%       progressively increased failure as O-O-P angle approaches +/- 90'
% * rz (tag roll, a.k.a. in-plane rotation): no failures for planar tag,
%       as expected

for trial_i = 1:(length(trial_ids)-1), % skip random
  trial_id = trial_ids{trial_i};
  seqds = trials.(trial_id).seqds;
  feature_var = sweep_feature_vars{trial_i};
  
  num_detect_per_feature_seq = [seqds.(feature_var), seqds.ftag2_num_tags_detected];
  
  zero_det_idx = (num_detect_per_feature_seq(:, 2) == 0);
  one_det_idx = (num_detect_per_feature_seq(:, 2) == 1);
  multi_det_idx = (num_detect_per_feature_seq(:, 2) > 1);
  zero_det_feature_vals = num_detect_per_feature_seq(num_detect_per_feature_seq(:, 2) == 0, 1);
  zero_det_unique_vals = unique(zero_det_feature_vals);
  zero_det_unique_val_count = zeros(size(zero_det_unique_vals));
  for j = 1:length(zero_det_unique_vals),
    zero_det_unique_val_count(j) = sum(zero_det_feature_vals == zero_det_unique_vals(j));
  end
  
  % Plot detection counts over (sweep var vs id_seq)
  seq_range = 1:size(seqds, 1);
  figure(fig_i); fig_i = fig_i+1; clf;
  hold on;
  plot(-1, 0, 'rx', 'MarkerSize', 6);
  plot(-1, 0, 'gv', 'MarkerSize', 6);
  plot(-1, 0, 'bo', 'MarkerSize', 6);
  plot(seq_range(one_det_idx), num_detect_per_feature_seq(one_det_idx, 1), ...
    'gv', 'MarkerSize', 6);
  plot(seq_range(zero_det_idx), num_detect_per_feature_seq(zero_det_idx, 1), ...
    'rx', 'MarkerSize', 6);
  plot(seq_range(multi_det_idx), num_detect_per_feature_seq(multi_det_idx, 1), ...
    'bo', 'MarkerSize', 6);
  hold off;
  ax = axis;
  ax(1) = 0;
  axis(ax);
  xlabel('raw sequence index');
  ylabel(feature_var, 'interpreter', 'none');
  title(sprintf('#-Detections for %s', trial_id), 'interpreter', 'none');
  legend('0-det', '1-det', 'N-det', 'Location', 'SouthEast');

  % Plot 0-detection counts vs sweep var
  figure(fig_i); fig_i = fig_i+1; clf;
  stem(zero_det_unique_vals, zero_det_unique_val_count, '-xb');
  xlabel(feature_var, 'interpreter', 'none');
  ylabel(sprintf('Num 0-detections [%d obs]', length(zero_det_feature_vals)));
  title('Unique values');

  % Plot 0-detection histograms vs sweep var
  figure(fig_i); fig_i = fig_i+1; clf;
  if ~isempty(zero_det_feature_vals),
    hist(zero_det_feature_vals, min(length(zero_det_unique_vals), 20));
  else
    stem(zero_det_unique_vals, zero_det_unique_val_count, '-xb'); % stub
  end
  xlabel(feature_var, 'interpreter', 'none');
  ylabel(sprintf('Num 0-detections [%d obs]', length(zero_det_feature_vals)));
  title('Histogram');
end

%% Compute and visualize 0-detections in random set
%
% *RESULTS:*
%
% * tx & ty: few 0-det; increases slightly when tag is moved away
%       from center of image
% * tz: concentration of 0-dets at near range, presumably due to tag
%       partially out of view frustum
% * rx & ry (tag pitch / yaw, a.k.a. out-of-plane rotations):
%       mostly uniform; 0-det increases slightly when tag is rotated
%       away from planar pose (at OOP angles > 55')
% * rz (tag roll, a.k.a. in-plane rotation): mostly uniform 0-det dist

zero_idx = (trials.random.seqds.ftag2_num_tags_detected == 0);
NUM_TAGS_PER_RANDOM_POSE = 10;
sds = trials.random.seqds(zero_idx, :);
group_sids = floor((sds.id_seq-1)/NUM_TAGS_PER_RANDOM_POSE);
unique_group_sids = unique(group_sids);
random_zero_det_poses = zeros(length(unique_group_sids), 7);
for gsid_idx = 1:length(unique_group_sids),
  target_idcs = find(group_sids == unique_group_sids(gsid_idx));
  sampleds = sds(target_idcs(1), :);
  random_zero_det_poses(gsid_idx, :) = [...
    length(target_idcs), ...
    sampleds.tag_tx_m, ...
    sampleds.tag_ty_m, ...
    sampleds.tag_tz_m, ...
    sampleds.tag_rx_deg, ...
    sampleds.tag_ry_deg, ...
    sampleds.tag_rz_deg, ...
    ];
end

rds = mat2dataset(random_zero_det_poses, 'VarNames', { ...
  'num_zero_detects', ...
  'tag_tx_m', ...
  'tag_ty_m', ...
  'tag_tz_m', ...
  'tag_rx_deg', ...
  'tag_ry_deg', ...
  'tag_rz_deg', ...
  });
%}

num_hist_bins = 40;
for pose_var_cell = rds.Properties.VarNames(2:end),
  pose_var = pose_var_cell{1};
  
  figure(fig_i); fig_i = fig_i+1; clf;
  hist(rds.(pose_var), num_hist_bins);
  xlabel(pose_var, 'interpreter', 'none');
  ylabel('Histogram counts');
end

%% Plot tag pose accuracies for sweep sets
%
% *RESULTS:*
%
% * tx & ty: small, zero-mean errors; magnitude & sign strongly correlated
%       to sweep feature's value
% * tz: error magnitude generally grows as tag moves further, with
%       high-rate sawtooth pattern hypothesized due to pixel-level jitter
%       during sweep
% * rx & ry (tag pitch & yaw, a.k.a. out-of-plane rotations):
%       small, zero-mean errors; low-rate linear dependency on sweep var &
%       high-rate sawtooth pattern hypothesized due to pixel-level jitter
% * rz (tag roll, a.k.a. in-plane rotation): small, zero-mean errors;
%      no visual dependence on sweep var

num_hist_bins = 20;
for trial_i = 1:(length(trial_ids)-1), % skip random
  trial_id = trial_ids{trial_i};
  feature_var = sweep_feature_vars{trial_i};
  
  % Isolate one obs per tag detection
  tds = trials.(trial_id).ds;
  selected_idx = (tds.tag_freq == 1) & (tds.tag_slice == 1) & ~(trials.(trial_id).ds_exclude_idx);
  ftds = tds(selected_idx, :);

  diff_pose_var = sprintf('diff_%s', feature_var(5:end));

  diff_vals = ftds.(diff_pose_var);
  
  figure(fig_i); fig_i = fig_i+1; clf;
  histfit(diff_vals, num_hist_bins);
  xlabel(diff_pose_var, 'interpreter', 'none');
  ylabel('Histogram counts');
  
  figure(fig_i); fig_i = fig_i+1; clf;
  boxplot(diff_vals, 'orientation', 'horizontal', 'extrememode', 'compress');
  xlabel(diff_pose_var, 'interpreter', 'none');
  ylabel('Boxplot (full)');
  
%   q1q3 = quantile(diff_vals, [0.25, 0.75]);
%   w = 1.75;
%   figure(fig_i); fig_i = fig_i+1; clf;
%   boxplot(diff_vals, 'orientation', 'horizontal', 'extrememode', 'compress');
%   xlabel(diff_pose_var, 'interpreter', 'none');
%   ylabel('Boxplot (CROPPED)');
%   ax = axis;
%   ax(1) = q1q3(1) - w*(q1q3(2)-q1q3(1));
%   ax(2) = q1q3(2) + w*(q1q3(2)-q1q3(1));
%   axis(ax);
  
  % Find obs index corresponding to first large negative change
  % (i.e. switch to second tag)
  f_vals = tds.(feature_var);
  f_diffs = f_vals(2:end)-f_vals(1:end-1);
  f_dsweep = unique(f_diffs);
  second_tag_start_id_seq = tds.id_seq(find(f_diffs == min(f_dsweep), 1, 'first')+1);
  selected_idx_single = (tds.id_seq < second_tag_start_id_seq) & ...
    (tds.tag_freq == 1) & (tds.tag_slice == 1) & ~(trials.(trial_id).ds_exclude_idx);
  ftds_single = tds(selected_idx_single, :);

  figure(fig_i); fig_i = fig_i+1; clf;
  plot(ftds_single.(feature_var), ftds_single.(diff_pose_var), '.', 'MarkerSize', 6);
  xlabel(feature_var, 'interpreter', 'none');
  ylabel(diff_pose_var, 'interpreter', 'none');
end

%% Plot tag pose accuracies for random set
%
% *RESULTS:*
%
% * tx, ty, tz: normal-distributed small errors
% * rx, ry (tag pitch / yaw, a.k.a. out-of-plane rotations):
%   normal-distributed small errors; certain amount of large angular errors
%   filtered due to quad-to-3D ambiguities with solvePnP
% * rz (tag roll, a.k.a. in-plane rotation):
%       normal-distributed small errors

% Isolate one obs per tag detection
tds = trials.random.ds;
selected_idx = (tds.tag_freq == 1) & (tds.tag_slice == 1) & ~(trials.random.ds_exclude_idx);
ftds = tds(selected_idx, :);

% Plot error bars for each of the pose variables
pose_vars = {'tx_m', 'ty_m', 'tz_m', ...
  'rx_deg', 'ry_deg', 'rz_deg', ...
  'pitch_deg', 'yaw_deg', 'roll_deg', 'txy_m', 'txyz_m', 'rxy_deg'};
num_hist_bins = 20;
for pose_var_cell = pose_vars,
  diff_pose_var = sprintf('diff_%s', pose_var_cell{1});

  diff_vals = ftds.(diff_pose_var);
  
  figure(fig_i); fig_i = fig_i+1; clf;
  histfit(diff_vals, num_hist_bins);
  xlabel(diff_pose_var, 'interpreter', 'none');
  ylabel('Histogram counts');
  
  figure(fig_i); fig_i = fig_i+1; clf;
  boxplot(diff_vals, 'orientation', 'horizontal', 'extrememode', 'compress');
  xlabel(diff_pose_var, 'interpreter', 'none');
  ylabel('Boxplot (full)');
  
%   q1q3 = quantile(diff_vals, [0.25, 0.75]);
%   w = 1.75;
%   figure(fig_i); fig_i = fig_i+1; clf;
%   boxplot(diff_vals, 'orientation', 'horizontal', 'extrememode', 'compress');
%   xlabel(diff_pose_var, 'interpreter', 'none');
%   ylabel('Boxplot (CROPPED)');
%   ax = axis;
%   ax(1) = q1q3(1) - w*(q1q3(2)-q1q3(1));
%   ax(2) = q1q3(2) + w*(q1q3(2)-q1q3(1));
%   axis(ax);
end

%% Plot phase payload errors vs sweep trials
%
% *RESULTS:*
%
% * freq: clear *linear* scaling of error *variance* observed by comparing
%         freq=1 and freq=2 plots
% * tx: *constant error bias* additionally with *some variance*;
%       hypothesize due to sub-pixel sampling of tag texture causing sine
%       signal irregularities as tx sweeps, leading to sawtooth error
%       pattern
% * ty: *constant error bias* with near-zero variance per sine signal;
%       hypothesize due to vertical translational invariance in FTag2
%       quad detector
% * tz: near-0 error bias; *linear variance growth* as tag moves away;
%       sawtooth error pattern hypothesized due to sub-pixel sampling
%       (same as tx)
% * rx (tag pitch): *positive error bias* with parabolic shape vs
%       angle; when viewed as tag_height_oop_multiplier = cos(tag_pitch),
%       see clear *linear bias scaling* (increased error when multiplier
%       approaches 0); sign flip in phase error at large out-of-plane
%       angles (>70') hypothesized due to severely poor ray sampling of
%       limited-sized tag in image (thus propose not to model)
% * ry (tag yaw): manual follow-up inspections show that some slices have
%       *feature-signed* parabolic-S shape while others have
%       *feature-unsigned* parabolic shape (hypothesize possibly related to
%       phase payload + caused by over-sampling black border on either
%       end of ray);
%       different error bias in [-20', 20'] range suspect due to other
%       variates overwhelming effects of ry
% * rz (tag roll): significant *error biases*, where sign of bias
%      consistently differs for [45', 215'] range and for [215', -45']
%      range; error variance remains relatively constant w.r.t. rz
%
% Most visible outliers in signed phase error are caused by ray
% undersampling, due to rounding & bilinear-interpolation inaccuracies

for trial_i = 1:(length(trial_ids)-1), % skip random
  trial_id = trial_ids{trial_i};
  feature_var = sweep_feature_vars{trial_i};
  
  tds = trials.(trial_id).ds;
  for filter_freq = 0:trials.(trial_id).tag_num_freqs, % 0 == only show first tag, s=1, f=1
    % Filter observations
    if filter_freq == 0, % Isolate first seq_id, s=1, f=1
      % Find obs index corresponding to first large negative change
      % (i.e. switch to second tag)
      f_vals = tds.(feature_var);
      f_diffs = f_vals(2:end)-f_vals(1:end-1);
      f_dsweep = unique(f_diffs);
      second_tag_start_id_seq = tds.id_seq(find(f_diffs == min(f_dsweep), 1, 'first')+1);
      selected_idx = (tds.id_seq < second_tag_start_id_seq) & ...
        (tds.tag_slice == 1) & (tds.tag_freq == 1) & ~(trials.(trial_id).ds_exclude_idx);
      title_str = 'Single tag, slice=1, freq=1';
    else
      selected_idx = (tds.tag_freq == filter_freq) & ~(trials.(trial_id).ds_exclude_idx);
      title_str = sprintf('All tags, all slices, freq=%d', filter_freq);
    end
    ftds = tds(selected_idx, :);
    
    % Plot signed phase error vs sweep var
    figure(fig_i); fig_i = fig_i+1; clf;
    plot(ftds.(feature_var), ftds.diff_phase_deg, ...
      '.b', 'LineWidth', 1, 'MarkerSize', 6);
    rho = corr(ftds.(feature_var), ftds.diff_phase_deg);
    xlabel(sprintf('%s [rho: %.4f]', feature_var, rho), ...
      'interpreter', 'none');
    ylabel(sprintf('Signed phase error (deg) [%d obs]', size(ftds, 1)));
    title(title_str);
    
%     % Plot abs phase error vs sweep var
%     figure(fig_i); fig_i = fig_i+1; clf;
%     plot(ftds.(feature_var), abs(ftds.diff_phase_deg), ...
%       '.b', 'LineWidth', 1, 'MarkerSize', 6);
%     rho = corr(ftds.(feature_var), abs(ftds.diff_phase_deg));
%     xlabel(sprintf('%s [rho: %.4f]', feature_var, rho), ...
%       'interpreter', 'none');
%     ylabel(sprintf('Abs phase error (deg) [%d obs]', size(ftds, 1)));
%     title(title_str);
    
    % Show additional plots against cos(feature)
    if strcmp(trial_id, 'sweep_pitch') || strcmp(trial_id, 'sweep_yaw'),
      figure(fig_i); fig_i = fig_i+1; clf;
      plot(cosd(ftds.(feature_var)), ftds.diff_phase_deg, ...
        '.b', 'LineWidth', 1, 'MarkerSize', 6);
      rho = corr(cosd(ftds.(feature_var)), ftds.diff_phase_deg);
      xlabel(sprintf('cos(%s) [rho: %.4f]', feature_var, rho), ...
        'interpreter', 'none');
      ylabel(sprintf('Signed phase error (deg) [%d obs]', size(ftds, 1)));
      title(title_str);

%       figure(fig_i); fig_i = fig_i+1; clf;
%       plot(1./cosd(ftds.(feature_var)), ftds.diff_phase_deg, ...
%         '.b', 'LineWidth', 1, 'MarkerSize', 6);
%       rho = corr(1./cosd(ftds.(feature_var)), ftds.diff_phase_deg);
%       xlabel(sprintf('1/cos(%s) [rho: %.4f]', feature_var, rho), ...
%         'interpreter', 'none');
%       ylabel(sprintf('Signed phase error (deg) [%d obs]', size(ftds, 1)));
%       title(title_str);
    end
  end
end

%% Plot phase payload errors vs features in random set
%
% *RESULTS:*
%
% * freq: clear *linear* scaling of error *variance* observed by comparing
%       freq=1 and freq=2 plots
% * tx & ty: no distinguishable relationship to constant error variance
% * tz: magnitude of error grows as tag moves away from camera, as expected
% * rx & ry (tag pitch and yaw): no distinguishable patterns to constant
%       error variance
% * rz (tag roll) & imgRot (tag 90-rotation ID):
%       piecewise biases similar to those from sweep rz plots
%
% Plot of ftag2_phase_deg vs signed phase error is not surprising, but
% does at least confirm that *very few errors* bleed over to next bit range

trial_i = length(trial_ids); % only random
trial_id = trial_ids{trial_i};
tds = trials.(trial_id).ds;
for filter_freq = 1:trials.(trial_id).tag_num_freqs,
  % Filter observations
  selected_idx = (tds.tag_freq == filter_freq) & ~(trials.(trial_id).ds_exclude_idx);
  title_str = sprintf('All tags, all slices, freq=%d', filter_freq);
  
  ftds = tds(selected_idx, :);

  for feature_var_ = { ...
      'ftag2_tx_m', 'ftag2_ty_m', 'ftag2_tz_m', ...
      'ftag2_rx_deg', 'ftag2_ry_deg', 'ftag2_rz_deg', ...
      'ftag2_pitch_deg', 'ftag2_yaw_deg', 'ftag2_roll_deg', ...
      'ftag2_oop_deg', 'ftag2_tag_img_rot', 'ftag2_phase_deg', ...
      },
    feature_var = feature_var_{1};

    % Plot signed phase error vs sweep var
    figure(fig_i); fig_i = fig_i+1; clf;
    plot(ftds.(feature_var), ftds.diff_phase_deg, ...
      '.b', 'LineWidth', 1, 'MarkerSize', 6);
    rho = corr(ftds.(feature_var), ftds.diff_phase_deg);
    xlabel(sprintf('%s [rho: %.4f]', feature_var, rho), ...
      'interpreter', 'none');
    ylabel(sprintf('Signed phase error (deg) [%d obs]', size(ftds, 1)));
    title(title_str);

  %   % Plot abs phase error vs sweep var
  %   figure(fig_i); fig_i = fig_i+1; clf;
  %   plot(ftds.(feature_var), abs(ftds.diff_phase_deg), ...
  %     '.b', 'LineWidth', 1, 'MarkerSize', 6);
  %   rho = corr(ftds.(feature_var), abs(ftds.diff_phase_deg));
  %   xlabel(sprintf('%s [rho: %.4f]', feature_var, rho), ...
  %     'interpreter', 'none');
  %   ylabel(sprintf('Abs phase error (deg) [%d obs]', size(ftds, 1)));
  %   title(title_str);
  end
end

%% Show relationship between tag roll and rot90 value
%
% Observe that rot90 value is highly correlated to roll, but not perfectly;
% i.e. for some values of roll, there have been multiple rot90 values

for trial_id = {'sweep_roll', 'random'},
  tds = trials.(trial_id{1}).seqds;
  selected_idx = ~trials.(trial_id{1}).seqds_exclude_idx;
  ftds = tds(selected_idx, :);

  figure(fig_i); fig_i = fig_i+1; clf;
  plot(ftds.tag_roll_deg, ftds.ftag2_tag_img_rot*90, ...
        '.b', 'LineWidth', 1, 'MarkerSize', 6);
  xlabel('tag roll (deg)');
  ylabel('tag image rot90 (deg)');
  title(sprintf('tag roll vs quad rot90 for %s set', trial_id{1}), ...
    'interpreter', 'none');
end

%% Clean up workspace
close all;
