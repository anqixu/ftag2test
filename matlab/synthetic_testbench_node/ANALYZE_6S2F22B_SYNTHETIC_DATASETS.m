%% Reset workspace

clear all;
INIT_WORKSPACE;
close all;

trial_ids = {'sweep_tx', 'sweep_ty', 'sweep_tz', 'sweep_pitch', 'sweep_yaw', 'sweep_roll', 'random'};
sweep_feature_vars = {'tag_tx_m', 'tag_ty_m', 'tag_tz_m', 'tag_rx_deg', 'tag_ry_deg', 'tag_rz_deg'};

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

%% Plot tag pose accuracies for random set

% TODO: analyze results

% Isolate one obs per tag detection
tds = trials.random.ds;
selected_idx = (tds.tag_freq == 1) & (tds.tag_slice == 1);
ftds = tds(selected_idx, :);

% Plot error bars for each of the pose variables
pose_vars = {'tx_m', 'ty_m', 'tz_m', ...
  'rx_deg', 'ry_deg', 'rz_deg', ...
  'pitch_deg', 'yaw_deg', 'roll_deg'};
fig_i = 1;
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
  ylabel('Boxplot (cropped)');
  ax = axis;
  ax(1) = q1q3(1) - w*(q1q3(2)-q1q3(1));
  ax(2) = q1q3(2) + w*(q1q3(2)-q1q3(1));
  axis(ax);
end

%% Display mis-detection and multi-detections
fprintf('%11s:\t%5s\t%5s\t%5s\n', 'TRIAL', '0-DET', 'N-DET', 'OBS');
for trial_i = 1:length(trial_ids),
  trial_id = trial_ids{trial_i};
  fprintf('%11s:\t%5d\t%5d\t%5d\n', trial_id, ...
    size(trials.(trial_id).zero_detection_raw_IDs, 1), ...
    size(trials.(trial_id).multi_detection_raw_IDs, 1), ...
    size(trials.(trial_id).ds, 1));
end
fprintf('\n');

%% Plot 0-detections vs sweep trials
%
% RESULTS:
% * tx & ty: decoder fails when tag is moved out of viewport, as expected
% * tz: decoder begins to fail as tag moves too far; fails when tag too
%       close to keep in viewport; single failure @ ~1.3m believed to be
%       suprious
% * rx & ry (tag pitch / yaw, a.k.a. out-of-plane rotations):
%       progressively increased failure as O-O-P angle approaches +/- 90'
% * rz (tag roll, a.k.a. in-plane rotation):
%       expected uniform count given spurious random failures;
%       *cannot explain increased failures in [45', 135'] range*

fig_i = 28;
for trial_i = 1:(length(trial_ids)-1), % skip random
  trial_id = trial_ids{trial_i};
  feature_var = sweep_feature_vars{trial_i};
  ind_trial_file = sprintf('%s/trials/6s2f22b_%s.mat', dataset_dir, trial_id);
  
  clear progress_seq;
  load(ind_trial_file);
  zero_det_feature_vals = [];
  for seq_i = 1:length(progress_seq),
    s = progress_seq{seq_i};
    if s.ftag2_num_tags_detected == 0,
      zero_det_feature_vals = [zero_det_feature_vals; s.(feature_var)]; %#ok<*AGROW>
    end
  end
  zero_det_unique_vals = unique(zero_det_feature_vals);
  zero_det_unique_val_count = zeros(size(zero_det_unique_vals));
  for j = 1:length(zero_det_unique_vals),
    zero_det_unique_val_count(j) = sum(zero_det_feature_vals == zero_det_unique_vals(j));
  end

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

%% Plot phase payload errors vs sweep trials
%
% RESULTS:
% * freq: clear *linear* scaling of error *variance* observed by comparing
%         freq=1 and freq=2 plots
% * tx: *constant error bias* additionally with *some variance*;
%       hypothesize due to sub-pixel sampling of tag texture causing sine
%       signal irregularities as tx sweeps
% * ty: *constant error bias* with near-zero variance per sine signal;
%       hypothesize due to vertical translational invariance in FTag2
%       quad detector
% * tz: near-0 error bias; *linear variance growth* as tag moves away
% * rx (tag pitch): significant *signed error*; parabolic shape vs
%       angle; once converted as *tag_height* ~ 1/cos(tag_pitch), clearly
%       indicates *linear bias scaling*
% * ry (tag yaw): excluding [-20', 20'] range: signed bias in phase error
%       has *signed parabolic* shape; once converted as
%       *tag_width* ~= 1/cos(tag_yaw), observe somewhat *linear variance*
%       relationship;
%       bias in [-20', 20'] range: suspect due to other variates
%       overwhelming effects of ry
% * rz (tag roll): significant *signed error biases*, where sign of bias
%      consistently differs for [45', 215'] range and for [215', -45']
%      range; error variance remains relatively constant w.r.t. rz

fig_i = 40;
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
      second_tag_start_id_seq = find(f_diffs == min(f_dsweep), 1, 'first')+1;
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
    
    % Plot abs phase error vs sweep var
    figure(fig_i); fig_i = fig_i+1; clf;
    plot(ftds.(feature_var), abs(ftds.diff_phase_deg), ...
      '.b', 'LineWidth', 1, 'MarkerSize', 6);
    rho = corr(ftds.(feature_var), abs(ftds.diff_phase_deg));
    xlabel(sprintf('%s [rho: %.4f]', feature_var, rho), ...
      'interpreter', 'none');
    ylabel(sprintf('Abs phase error (deg) [%d obs]', size(ftds, 1)));
    title(title_str);
    
    % Show additional plots against 1/cos(feature)
    if strcmp(trial_id, 'sweep_pitch') || strcmp(trial_id, 'sweep_yaw'),
      % Plot signed phase error vs sweep var
      figure(fig_i); fig_i = fig_i+1; clf;
      plot(1./cosd(ftds.(feature_var)), ftds.diff_phase_deg, ...
        '.b', 'LineWidth', 1, 'MarkerSize', 6);
      rho = corr(1./cosd(ftds.(feature_var)), ftds.diff_phase_deg);
      xlabel(sprintf('1/cos(%s) [rho: %.4f]', feature_var, rho), ...
        'interpreter', 'none');
      ylabel(sprintf('Signed phase error (deg) [%d obs]', size(ftds, 1)));
      title(title_str);

      % Plot abs phase error vs sweep var
      figure(fig_i); fig_i = fig_i+1; clf;
      plot(1./cosd(ftds.(feature_var)), abs(ftds.diff_phase_deg), ...
        '.b', 'LineWidth', 1, 'MarkerSize', 6);
      rho = corr(1./cosd(ftds.(feature_var)), abs(ftds.diff_phase_deg));
      xlabel(sprintf('1/cos(%s) [rho: %.4f]', feature_var, rho), ...
        'interpreter', 'none');
      ylabel(sprintf('Abs phase error (deg) [%d obs]', size(ftds, 1)));
      title(title_str);      
    end
  end
end

%% Compute 0-detections in random set

% Load raw data for random trial
rand_trial_file = sprintf('%s/trials/6s2f22b_random.mat', dataset_dir);
load(rand_trial_file);
random_zero_det_poses = [];
for i = 1:length(progress_seq)/10,
  num_zero_detects = 0;
  pose_vecs = [];
  
  for j = 1:10,
    s = progress_seq{10*(i-1)+j};
    if s.ftag2_num_tags_detected == 0,
      num_zero_detects = num_zero_detects + 1;
      if isempty(pose_vecs),
        pose_vecs = [s.tag_tx_m, s.tag_ty_m, s.tag_tz_m, ...
          s.tag_rx_deg, s.tag_ry_deg, s.tag_rz_deg];
      end
    end
  end
  
  if num_zero_detects > 0,
    random_zero_det_poses = [random_zero_det_poses; ...
      num_zero_detects, pose_vecs];
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

% TODO: analyze results

pose_vars = {'tag_tx_m', 'tag_ty_m', 'tag_tz_m', ...
  'tag_rx_deg', 'tag_ry_deg', 'tag_rz_deg'};
fig_i = 50; % TODO: figure out count now
num_hist_bins = 20;
for pose_var_cell = pose_vars,
  pose_var = pose_var_cell{1};
  
  figure(fig_i); fig_i = fig_i+1; clf;
  histfit(rds.(pose_var), num_hist_bins);
  xlabel(pose_var, 'interpreter', 'none');
  ylabel('Histogram counts');
end

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
      - z/tag_width * 1/cos(tag_pitch) [a.k.a. tag_pixel_height]
      - z/tag_width * 1/cos(tag_yaw) [a.k.a. tag_pixel_width]
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
