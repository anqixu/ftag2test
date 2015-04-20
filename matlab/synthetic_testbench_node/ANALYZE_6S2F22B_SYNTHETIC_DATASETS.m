%% Reset workspace

clear all;
INIT_WORKSPACE;
close all;

trial_ids = {'sweep_tx', 'sweep_ty', 'sweep_tz', 'sweep_pitch', 'sweep_yaw', 'sweep_roll', 'random'};
sweep_feature_vars = {'tag_tx_m', 'tag_ty_m', 'tag_tz_m', 'tag_rx_deg', 'tag_ry_deg', 'tag_rz_deg'};
fig_i = 1;

%% Create or load datasets

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
else
  load(all_trials_file);
end

%% Display mis-detection and multi-detections
fprintf('%11s:\t%9s\t%9s\t%6s\n', 'TRIAL', '0-DET', 'N-DET', 'OBS');
for trial_i = 1:length(trial_ids),
  trial_id = trial_ids{trial_i};
  trial = trials.(trial_id);
  num_zero_det = size(trial.zero_detection_raw_IDs, 1);
  num_multi_det = size(trial.multi_detection_raw_IDs, 1);
  num_obs = size(trial.ds, 1);
  num_one_det = num_obs/trial.tag_num_slices/trial.tag_num_freqs;
  num_total_det = num_zero_det + num_one_det + num_multi_det;
  fprintf('%11s:\t%4d (%2.0f%%)\t%4d (%2.0f%%)\t%6d\n', trial_id, ...
    num_zero_det, num_zero_det/num_total_det*100, ...
    num_multi_det, num_multi_det/num_total_det*100, ...
    num_obs);
end
fprintf('\n');

%% Plot 0-detections vs sweep trials

for trial_i = 1:(length(trial_ids)-1), % skip random
  trial_id = trial_ids{trial_i};
  feature_var = sweep_feature_vars{trial_i};
  ind_trial_file = sprintf('%s/trials/6s2f22b_%s.mat', dataset_dir, trial_id);
  
  clear progress_seq;
  load(ind_trial_file);
  
  num_seq = size(progress_seq, 2);
  num_detect_per_feature_seq = zeros(num_seq, 2);
  for seq_i = 1:num_seq,
    s = progress_seq{seq_i};
    num_detect_per_feature_seq(seq_i, :) = [s.(feature_var), s.ftag2_num_tags_detected];
  end
  
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
  seq_range = 1:num_seq;
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

%%
% *RESULTS:*
%
% * tx & ty: decoder fails when tag is moved out of viewport, as expected
% * tz: decoder fails when tag is too close to keep all in viewport;
%       expect black-border check to fail more and more as tag moves away
% * rx & ry (tag pitch / yaw, a.k.a. out-of-plane rotations):
%       progressively increased failure as O-O-P angle approaches +/- 90'
% * rz (tag roll, a.k.a. in-plane rotation): no failures for planar tag,
%       as expected

%% Compute 0-detections in random set

% Load raw data for random trial
rand_trial_file = sprintf('%s/trials/6s2f22b_random.mat', dataset_dir);
load(rand_trial_file);
random_zero_det_poses = [];
NUM_TAGS_PER_RANDOM_POSE = 10;
for i = 1:length(progress_seq)/NUM_TAGS_PER_RANDOM_POSE,
  num_zero_detects = 0;
  pose_vecs = [];
  
  for j = 1:NUM_TAGS_PER_RANDOM_POSE,
    s = progress_seq{NUM_TAGS_PER_RANDOM_POSE*(i-1)+j};
    if isfield(s, 'ftag2_num_tags_detected') && s.ftag2_num_tags_detected == 0,
      num_zero_detects = num_zero_detects + 1;
      if isempty(pose_vecs),
        pose_vecs = [s.tag_tx_m, s.tag_ty_m, s.tag_tz_m, ...
          s.tag_rx_deg, s.tag_ry_deg, s.tag_rz_deg];
      end
    end
  end
  
  if num_zero_detects > 0,
    random_zero_det_poses = [random_zero_det_poses; ...
      num_zero_detects, pose_vecs]; %#ok<AGROW>
  end
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

%% Visualize 0-detections in random set as fn of tag pose

pose_vars = {'tag_tx_m', 'tag_ty_m', 'tag_tz_m', ...
  'tag_rx_deg', 'tag_ry_deg', 'tag_rz_deg'};
num_hist_bins = 40;
for pose_var_cell = pose_vars,
  pose_var = pose_var_cell{1};
  
  figure(fig_i); fig_i = fig_i+1; clf;
  hist(rds.(pose_var), num_hist_bins);
  xlabel(pose_var, 'interpreter', 'none');
  ylabel('Histogram counts');
end

%%
% *RESULTS:*
%
% * tx & ty: mostly uniform; 0-det increases slightly when tag is moved
%       away from center of image
% * tz: high 0-det count at near range, hypothesized due to tag (partially)
%       out of view frustum; increased 0-det count for large tz, likely
%       due to small tag size in image
% * rx & ry (tag pitch / yaw, a.k.a. out-of-plane rotations):
%       mostly uniform; 0-det increases slightly when tag is rotated
%       away from planar pose
% * rz (tag roll, a.k.a. in-plane rotation): uniform 0-det distribution

%% Plot tag pose accuracies for sweep sets

for trial_i = 1:(length(trial_ids)-1), % skip random
  trial_id = trial_ids{trial_i};
  feature_var = sweep_feature_vars{trial_i};
  
  % Isolate one obs per tag detection
  tds = trials.(trial_id).ds;
  selected_idx = (tds.tag_freq == 1) & (tds.tag_slice == 1);
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
  
  q1q3 = quantile(diff_vals, [0.25, 0.75]);
  w = 1.75;
  figure(fig_i); fig_i = fig_i+1; clf;
  boxplot(diff_vals, 'orientation', 'horizontal', 'extrememode', 'compress');
  xlabel(diff_pose_var, 'interpreter', 'none');
  ylabel('Boxplot (CROPPED)');
  ax = axis;
  ax(1) = q1q3(1) - w*(q1q3(2)-q1q3(1));
  ax(2) = q1q3(2) + w*(q1q3(2)-q1q3(1));
  axis(ax);
  
  % Find obs index corresponding to first large negative change
  % (i.e. switch to second tag)
  f_vals = tds.(feature_var);
  f_diffs = f_vals(2:end)-f_vals(1:end-1);
  f_dsweep = unique(f_diffs);
  second_tag_start_id_seq = tds.id_seq(find(f_diffs == min(f_dsweep), 1, 'first')+1);
  selected_idx_single = (tds.id_seq < second_tag_start_id_seq) & (tds.tag_slice == 1) & (tds.tag_freq == 1);
  ftds_single = tds(selected_idx_single, :);

  figure(fig_i); fig_i = fig_i+1; clf;
  plot(ftds_single.(feature_var), ftds_single.(diff_pose_var), '.', 'MarkerSize', 6);
  xlabel(feature_var, 'interpreter', 'none');
  ylabel(diff_pose_var, 'interpreter', 'none');
end

%%
% *RESULTS:*
%
% * tx & ty: small, zero-mean errors; magnitude & sign strongly correlated
%       to sweep feature's value
% * tz: clear error bias, hypothesized due to quad detector always
%       underestimating quad & biased camera intrinsics matrix;
%       error magnitude generally grows as tag moves further, with
%       high-rate sawtooth pattern hypothesized due to pixel-level jitter
%       during sweep
% * rx & ry (tag pitch & yaw, a.k.a. out-of-plane rotations):
%       small, zero-mean errors; low-rate linear dependency on sweep var &
%       high-rate sawtooth pattern hypothesized due to pixel-level jitter;
%       *few extremely large outliers (why? maybe due to solvePnP?)*
% * rz (tag roll, a.k.a. in-plane rotation): small, zero-mean errors;
%      no visual dependence on sweep var

%% Plot tag pose accuracies for random set

% Isolate one obs per tag detection
tds = trials.random.ds;
selected_idx = (tds.tag_freq == 1) & (tds.tag_slice == 1);
ftds = tds(selected_idx, :);

% Plot error bars for each of the pose variables
pose_vars = {'tx_m', 'ty_m', 'tz_m', ...
  'rx_deg', 'ry_deg', 'rz_deg', ...
  'pitch_deg', 'yaw_deg', 'roll_deg'};
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
  
  q1q3 = quantile(diff_vals, [0.25, 0.75]);
  w = 1.75;
  figure(fig_i); fig_i = fig_i+1; clf;
  boxplot(diff_vals, 'orientation', 'horizontal', 'extrememode', 'compress');
  xlabel(diff_pose_var, 'interpreter', 'none');
  ylabel('Boxplot (CROPPED)');
  ax = axis;
  ax(1) = q1q3(1) - w*(q1q3(2)-q1q3(1));
  ax(2) = q1q3(2) + w*(q1q3(2)-q1q3(1));
  axis(ax);
end

%%
% *RESULTS:*
%
% * tx, ty, tz: normal-distributed small errors
% * rx, ry (tag pitch / yaw, a.k.a. out-of-plane rotations):
%   normal-distributed small errors; but *unexpectedly high outlier count*:
%   suspect due to solvePnP-related error
% * rz (tag roll, a.k.a. in-plane rotation):
%       normal-distributed small errors; but *few outliers near +/-180'*

%% Plot phase payload errors vs sweep trials

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
      selected_idx = (tds.id_seq < second_tag_start_id_seq) & (tds.tag_slice == 1) & (tds.tag_freq == 1);
      title_str = 'Single tag, slice=1, freq=1';
    else
      selected_idx = (tds.tag_freq == filter_freq);
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

%%
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

%% Plot phase payload errors vs features in random set

% TODO: plot sign and magn errors vs {tx, ty, tz, rx, ry, rz, pitch, yaw,
% roll, rot90}, while filtering by freq=1 and freq=2 (across all tags and
% all slices)

%% Compute naive gen. backwards stepwise linear regression on phase bias

% TODO: glmfit, stepwise, anova
% TODO: analyze residuals
%{
    - always-present features:
      - payload phase / joint_phases_in_slice
      - frequency (only with 'payload phase')
      - slice
        
    - regression features:
      - tag x/tag_width, y/tag_width; or sqrt(x^2+y^2)/tag_width
      - tag_width/z * cos(tag_pitch) [a.k.a. tag_pixel_height]
      - tag_width/z * cos(tag_yaw) [a.k.a. tag_pixel_width]
      - rot90; or tag_roll
      - tag z/tag_width
%}

%% Compute gen. linear regression with hand-crafted features on phase bias

% TODO: glmfit, anova
% TODO: analyze residuals

%% Compute gen. linear regression with hand-crafted features on phase variance

% TODO: glmfit, anova
% TODO: analyze residuals

%% Fit hand-crafted sign+bias+variance model

% TODO: model fitting
% TODO: analyze residuals

%{
  - Q: glmfit + anova: which factors are significant, what's the left-over residual
    - error bias = sgn(weighted features) * magn(weighted features)
    - what's the mean squared residual? how many entries have incorrect sign?
    - instead of using sgn, how about using sigmoid?
    - explicitly look at how the errors are being corrected by model. does it shrink? does it expand some errors? does it change sign of error?
%}

%% Clean up workspace
close all;
