%% Reset workspace, then create or load datasets

clear all;
INIT_WORKSPACE;
close all;

trial_ids = {'sweep_tx', 'sweep_ty', 'sweep_tz', 'sweep_pitch', 'sweep_yaw', 'sweep_roll', 'random'};
sweep_feature_vars = {'tag_tx_m', 'tag_ty_m', 'tag_tz_m', 'tag_rx_deg', 'tag_ry_deg', 'tag_rz_deg'};
fig_i = 1;

dataset_dir = '../ftag2_datasets';
all_trials_file = fullfile(dataset_dir, 'trials', '6s2f22b_all_trials.processed.mat');
load(all_trials_file);

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

%% Choose dataset for bias regression
trial_id = 'random';
tds = trials.(trial_id).ds;
tds_exclude_idx = trials.(trial_id).ds_exclude_idx;

biasModelSimple = strcat('diff_phase_deg ~ ', ...
  ' 1', ...
  ' + ftag2_tag_img_rot*tag_freq', ...
  '');

% WARNING: Using a linear dependency on ftag2_width_px
%  (\prop tag_width_m/tag_tz_m) is equivalent a 1/z harmonic dependency
%  on tag_tz_m. Thus, using such a model will result in an
%  infinity-asymptotical over-compensation of the predicted bias, at small
%  tag_tz_m distances from camera. Additionally, this effect is triply
%  amplified by the interaction term "ftag2_width_px*quad_diag13_px2",
%  since both "ftag2_width_px" and "quad_diag13_px" individually are
%  harmonically-related to tag_tz_m
biasModelQuad = strcat('diff_phase_deg ~ ', ...
  ' 1', ...
  ' + ftag2_tag_img_rot*tag_freq', ...
  ' + ftag2_tag_img_rot*ftag2_width_px', ...
  ' + ftag2_tag_img_rot*quad_diag13_px2', ...
  ' + ftag2_tag_img_rot*quad_diag_sqrd_ratio', ...
  ' + ftag2_tag_img_rot*ftag2_width_px*quad_diag13_px2', ...
  ' + ftag2_tag_img_rot*ftag2_width_px*quad_diag_sqrd_ratio', ...
  ' + ftag2_tag_img_rot*quad_diag13_px2*quad_diag_sqrd_ratio', ...
  '');

biasModelPose = strcat('diff_phase_deg ~ ', ...
  ' 1', ...
  ' + ftag2_tag_img_rot*tag_freq', ...
  ' + ftag2_tag_img_rot*ftag2_txy_norm', ...
  ' + ftag2_tag_img_rot*ftag2_tz_norm', ...
  ' + ftag2_tag_img_rot*ftag2_pitch_height_scale', ...
  ' + ftag2_tag_img_rot*ftag2_yaw_width_scale', ...
  '');

% Inspect informally performance of linear regressions on phase bias

biasFitSimpleAll = fitlm(tds, biasModelSimple, ...
  'CategoricalVars', {'ftag2_tag_img_rot'}, 'Exclude', tds_exclude_idx);
disp('BIAS FIT SIMPLE:');
disp(biasFitSimpleAll);
disp(biasFitSimpleAll.anova);

disp('BIAS FIT QUAD (DAVID):');
biasFitQuadAll = fitlm(tds, biasModelQuad, ...
  'CategoricalVars', {'ftag2_tag_img_rot'}, 'Exclude', tds_exclude_idx);
disp(biasFitQuadAll);
disp(biasFitQuadAll.anova);

disp('BIAS FIT POSE (ANQI):');
biasFitPoseAll = fitlm(tds, biasModelPose, ...
  'CategoricalVars', {'ftag2_tag_img_rot'}, 'Exclude', tds_exclude_idx);
disp(biasFitPoseAll);
disp(biasFitPoseAll.anova);

%% Perform N-fold cross-validation and report test-set results

TEST_SET_RATIO = 0.2;
N_FOLD_XVAL = 4;
biasModelXval = biasModelPose;

total_num_obs = size(tds, 1);
set_type_ids = zeros(total_num_obs, 1); % 0 = test set, 1-N_FOLD_XVAL: xval subsets
xval_subset_size = floor(total_num_obs*(1-TEST_SET_RATIO)/N_FOLD_XVAL);
xval_set_ids = repmat(1:N_FOLD_XVAL, xval_subset_size, 1);
set_type_ids(1:numel(xval_set_ids)) = xval_set_ids(:);

biasFitTrain = cell(N_FOLD_XVAL, 1);
best_fit_idx = 0;
best_fit_RMSE = inf;
worst_fit_idx = 0;
worst_fit_RMSE = 0;
for xval_id = 1:N_FOLD_XVAL,
  biasFitTrain{xval_id} = fitlm(tds, biasModelXval, ...
    'CategoricalVars', {'ftag2_tag_img_rot'}, ...
    'Exclude', (tds_exclude_idx | (set_type_ids == xval_id) | (set_type_ids == 0)));
  dsXval = tds(~tds_exclude_idx & (set_type_ids == xval_id), :);
  pred_diff_phase_deg = biasFitTrain{xval_id}.predict(dsXval);
  RMSE = sqrt(mean(angularDiff(dsXval.diff_phase_deg, pred_diff_phase_deg).^2));
  
  if RMSE < best_fit_RMSE,
    best_fit_RMSE = RMSE;
    best_fit_idx = xval_id;
  end
  if RMSE > worst_fit_RMSE,
    worst_fit_RMSE = RMSE;
    worst_fit_idx = xval_id;
  end
end
biasFit = biasFitTrain{best_fit_idx};

testds = tds(~(tds_exclude_idx | (set_type_ids ~= 0)), :);
pred_diff_phase_deg = biasFit.predict(testds);
test_fit_RMSE = sqrt(mean((pred_diff_phase_deg - testds.diff_phase_deg).^2));

fprintf('===== %d-FOLD XVAL RESULTS =====\n%s\n- best xval RMSE: %.4f (id=%d)\n- worst xval RMSE: %.4f (id=%d)\n- test RMSE: %.4f\n', ...
  N_FOLD_XVAL, biasModelXval, best_fit_RMSE, best_fit_idx, worst_fit_RMSE, worst_fit_idx, test_fit_RMSE);

%% Remove bias from all datasets, and generate variance random-group set

apply_bias_model = false; % if false then load existing bias model
trials_wo_bias_file = fullfile(dataset_dir, 'trials', '6s2f22b_all_trials.bias_removed.mat');

if apply_bias_model,
  for trial_i = 1:length(trial_ids),
    trial_id = trial_ids{trial_i};
    tds = trials.(trial_id).ds;

    fprintf('- Removing bias from %s\n', trial_id);
    if isempty(biasFit),
      pred_diff_phase_deg = zeros(size(tds, 1), 1);
    else
      pred_diff_phase_deg = biasFit.predict(tds);
    end

    tds.diff_phase_wo_bias_deg = angularDiff(tds.tag_phase_deg, tds.ftag2_phase_deg - pred_diff_phase_deg);

    trials.(trial_id).ds = tds;
  end

  % for each freq and each pose in random set
  %
  % IMPORTANT: stdev of phase errors is INDEPENDENT from bias of phase
  %            errors, so it DOES NOT MATTER which bias correction model used

  trial_id = 'random';
  NUM_TAGS_PER_RANDOM_POSE = 6;
  NUM_FREQS = trials.(trial_id).tag_num_freqs;
  tds = trials.(trial_id).ds;
  tds_exclude_idx = trials.(trial_id).ds_exclude_idx;
  group_seqids = unique(floor((tds.id_seq(~tds_exclude_idx)-1)/NUM_TAGS_PER_RANDOM_POSE));
  gds_all_target_idx = cell(length(group_seqids)*NUM_FREQS, 1);
  gds_first_target_idx = zeros(length(group_seqids)*NUM_FREQS, 1);
  gds_i = 0;
  fprintf('- Computing group dataset\n');

  for group_seqid = group_seqids',
    for freq = 1:NUM_FREQS,
      target_idx = find((floor((tds.id_seq-1)/NUM_TAGS_PER_RANDOM_POSE) == group_seqid) & ...
        (tds.tag_freq == freq) & ~tds_exclude_idx);
      gds_i = gds_i + 1;
      gds_all_target_idx{gds_i} = target_idx;
      gds_first_target_idx(gds_i) = target_idx(1);
    end
  end

  gds = tds(gds_first_target_idx, :);
  gds.id_seq = [];
  gds.id_frame = [];
  gds.tag_slice = [];
  gds.ftag2_num_tags_detected = [];
  gds.diff_payload_str_n = [];
  gds.tag_slice_wide_phase_id = [];
  gds.ftag2_mag = [];
  gds.tag_phase_deg = [];
  gds.ftag2_phase_deg = [];
  gds.diff_phase_deg = [];
  gds.diff_phase_wo_bias_deg = [];
  gds.diff_tx_m = [];
  gds.diff_ty_m = [];
  gds.diff_tz_m = [];
  gds.diff_txy_m = [];
  gds.diff_txyz_m = [];
  gds_diff_rx_deg = [];
  gds.diff_ry_deg = [];
  gds.diff_rz_deg = [];
  gds.diff_pitch_deg = [];
  gds.diff_yaw_deg = [];
  gds.diff_oop_deg = [];
  gds.diff_rxy_deg = [];
  gds.quad_border_mindist_px = [];
  gds.quad_area_px2 = [];
  gds.tag_roll_range_id = [];
  gds.ftag2_roll_range_id = [];

  gds.mean_diff_phase_wo_bias_deg = nan(size(gds, 1), 1);
  gds.stdev_diff_phase_wo_bias_deg = nan(size(gds, 1), 1);
  num_groups = size(gds, 1);
  for gds_i = 1:num_groups,
    if mod(gds_i, 1000) == 0,
      fprintf('-- Processing group %4d/%4d\n', gds_i, num_groups);
    end
      
    target_idx = gds_all_target_idx{gds_i};
    if (length(target_idx) >= 3),
      gds.mean_diff_phase_wo_bias_deg(gds_i) = mean(tds.diff_phase_wo_bias_deg(target_idx));
      gds.stdev_diff_phase_wo_bias_deg(gds_i) = std(tds.diff_phase_wo_bias_deg(target_idx));
      gds.ftag2_tx_m(gds_i) = mean(tds.ftag2_tx_m(target_idx));
      gds.ftag2_ty_m(gds_i) = mean(tds.ftag2_ty_m(target_idx));
      gds.ftag2_tz_m(gds_i) = mean(tds.ftag2_tz_m(target_idx));
      gds.ftag2_rx_deg(gds_i) = angularMean(tds.ftag2_tx_m(target_idx));
      gds.ftag2_ry_deg(gds_i) = angularMean(tds.ftag2_tx_m(target_idx));
      gds.ftag2_rz_deg(gds_i) = angularMean(tds.ftag2_tx_m(target_idx));
      gds.ftag2_width_px(gds_i) = mean(tds.ftag2_width_px(target_idx));
      gds.quad_corner1x_px(gds_i) = mean(tds.quad_corner1x_px(target_idx));
      gds.quad_corner1y_px(gds_i) = mean(tds.quad_corner1y_px(target_idx));
      gds.quad_corner2x_px(gds_i) = mean(tds.quad_corner2x_px(target_idx));
      gds.quad_corner2y_px(gds_i) = mean(tds.quad_corner2y_px(target_idx));
      gds.quad_corner3x_px(gds_i) = mean(tds.quad_corner3x_px(target_idx));
      gds.quad_corner3y_px(gds_i) = mean(tds.quad_corner3y_px(target_idx));
      gds.quad_corner4x_px(gds_i) = mean(tds.quad_corner4x_px(target_idx));
      gds.quad_corner4y_px(gds_i) = mean(tds.quad_corner4y_px(target_idx));
      if length(unique(tds.ftag2_tag_img_rot(target_idx))) > 1,
        gds.ftag2_tag_img_rot(gds_i) = nan;
      end
    end
  end

  rad = pi/180;

  [pitch_rad, yaw_rad, roll_rad] = tf_euler_inverse(gds.ftag2_rx_deg*rad, gds.ftag2_ry_deg*rad, gds.ftag2_rz_deg*rad);
  gds.ftag2_pitch_deg = pitch_rad/rad;
  gds.ftag2_yaw_deg = yaw_rad/rad;
  gds.ftag2_roll_deg = roll_rad/rad;
  gds.ftag2_oop_deg = tf_angle_between_euler_poses(gds.ftag2_pitch_deg, gds.ftag2_yaw_deg, gds.ftag2_roll_deg, ...
    zeros(size(gds.ftag2_pitch_deg)), zeros(size(gds.ftag2_pitch_deg)), zeros(size(gds.ftag2_pitch_deg)));
  gds.ftag2_txy_norm = sqrt(gds.ftag2_tx_m.^2+gds.ftag2_ty_m.^2)./gds.tag_width_m;
  gds.ftag2_tz_norm = gds.ftag2_tz_m./gds.tag_width_m;
  gds.ftag2_pitch_height_scale = cosd(gds.ftag2_pitch_deg);
  gds.ftag2_yaw_width_scale = cosd(gds.ftag2_yaw_deg);
  gds.quad_diag13_px_sqrd = (gds.quad_corner1x_px-gds.quad_corner3x_px).^2+(gds.quad_corner1y_px-gds.quad_corner3y_px).^2;
  gds.quad_diag24_px_sqrd = (gds.quad_corner2x_px-gds.quad_corner4x_px).^2+(gds.quad_corner2y_px-gds.quad_corner4y_px).^2;
  gds.quad_diag_sqrd_ratio = gds.quad_diag13_px_sqrd ./ gds.quad_diag24_px_sqrd;

  gds_exclude_idx = isnan(gds.mean_diff_phase_wo_bias_deg);

  trials.random.gds = gds;
  trials.random.gds_exclude_idx = gds_exclude_idx;
  trials.bias_model = biasModelXval;
  trials.bias_fit = biasFit;
  
  save(trials_wo_bias_file, 'trials');
  fprintf('- Finished computing and saving bias-removed datasets\n');
else
  load(trials_wo_bias_file, 'trials'); %#ok<*UNRCH>
  gds = trials.random.gds;
  gds_exclude_idx = trials.random.gds_exclude_idx;
end

%% Plot bias-corrected phase payload errors vs sweep trials

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
    plot(ftds.(feature_var), ftds.diff_phase_wo_bias_deg, ...
      '.b', 'LineWidth', 1, 'MarkerSize', 6);
    rho = corr(ftds.(feature_var), ftds.diff_phase_wo_bias_deg);
    xlabel(sprintf('%s [rho: %.4f]', feature_var, rho), ...
      'interpreter', 'none');
    ylabel(sprintf('Signed phase error w/o bias (deg) [%d obs]', size(ftds, 1)));
    title(title_str);
    
    % Show additional plots against cos(feature)
    if strcmp(trial_id, 'sweep_pitch') || strcmp(trial_id, 'sweep_yaw'),
      figure(fig_i); fig_i = fig_i+1; clf;
      plot(cosd(ftds.(feature_var)), ftds.diff_phase_wo_bias_deg, ...
        '.b', 'LineWidth', 1, 'MarkerSize', 6);
      rho = corr(cosd(ftds.(feature_var)), ftds.diff_phase_wo_bias_deg);
      xlabel(sprintf('cos(%s) [rho: %.4f]', feature_var, rho), ...
        'interpreter', 'none');
      ylabel(sprintf('Signed phase error w/o bias (deg) [%d obs]', size(ftds, 1)));
      title(title_str);
    end
  end
end

%% Plot bias-corrected phase payload errors vs features in random set

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
    plot(ftds.(feature_var), ftds.diff_phase_wo_bias_deg, ...
      '.b', 'LineWidth', 1, 'MarkerSize', 6);
    rho = corr(ftds.(feature_var), ftds.diff_phase_wo_bias_deg);
    xlabel(sprintf('%s [rho: %.4f]', feature_var, rho), ...
      'interpreter', 'none');
    ylabel(sprintf('Signed phase error w/o bias (deg) [%d obs]', size(ftds, 1)));
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

%% Plot variances of bias-corrected phase payload errors vs features in random set

NUM_FREQS = trials.random.tag_num_freqs;
for filter_freq = 1:NUM_FREQS,
  % Filter observations
  selected_idx = (gds.tag_freq == filter_freq) & ~(gds_exclude_idx);
  title_str = sprintf('All tags, all slices, freq=%d', filter_freq);
  
  ftds = gds(selected_idx, :);

  for feature_var_ = { ...
      'ftag2_tx_m', 'ftag2_ty_m', 'ftag2_tz_m', ...
      'ftag2_rx_deg', 'ftag2_ry_deg', 'ftag2_rz_deg', ...
      'ftag2_pitch_deg', 'ftag2_yaw_deg', 'ftag2_roll_deg', ...
      'ftag2_oop_deg', 'ftag2_tag_img_rot', ...
      },
    feature_var = feature_var_{1};

    % Plot standard deviation of bias-corrected phase error vs sweep var
    figure(fig_i); fig_i = fig_i+1; clf;
    plot(ftds.(feature_var), ftds.stdev_diff_phase_wo_bias_deg, ...
      '.b', 'LineWidth', 1, 'MarkerSize', 6);
    rho = corr(ftds.(feature_var), ftds.stdev_diff_phase_wo_bias_deg);
    xlabel(sprintf('%s [rho: %.4f]', feature_var, rho), ...
      'interpreter', 'none');
    ylabel(sprintf('Stdev of phase error w/o bias (deg) [%d obs]', size(ftds, 1)));
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

%% Compute linear regression on phase bias

fitStdevSimple = fitlm(gds, strcat('stdev_diff_phase_wo_bias_deg ~ ', ...
    ' 1', ...
    ' + tag_freq', ...
    ''), ...
    'CategoricalVars', {'ftag2_tag_img_rot'}, 'Exclude', gds_exclude_idx);
disp('STDEV FIT SIMPLE:');
disp(fitStdevSimple);
disp(fitStdevSimple.anova);

fitStdevDavid = fitlm(gds, strcat('stdev_diff_phase_wo_bias_deg ~ ', ...
    ' 1', ...
    ' + tag_freq', ...
    ' + ftag2_width_px', ...
    ' + quad_diag13_px_sqrd', ...
    ' + quad_diag_sqrd_ratio', ...
    ' + ftag2_width_px*quad_diag13_px_sqrd', ...
    ' + ftag2_width_px*quad_diag_sqrd_ratio', ...
    ' + quad_diag13_px_sqrd*quad_diag_sqrd_ratio', ...
    ''), ...
    'CategoricalVars', {'ftag2_tag_img_rot'}, 'Exclude', gds_exclude_idx);
disp('STDEV FIT QUAD (DAVID):');
disp(fitStdevDavid);
disp(fitStdevDavid.anova);

fitStdevAnqi = fitlm(gds, strcat('stdev_diff_phase_wo_bias_deg ~ ', ...
    ' 1', ...
    ' + tag_freq', ...
    ' + ftag2_txy_norm', ...
    ' + ftag2_tz_norm', ...
    ' + ftag2_pitch_height_scale', ...
    ' + ftag2_yaw_width_scale', ...
    ''), ...
    'CategoricalVars', {'ftag2_tag_img_rot'}, 'Exclude', gds_exclude_idx);
disp('STDEV FIT POSE (ANQI):');
disp(fitStdevAnqi);
disp(fitStdevAnqi.anova);

%% Regression test code
%premove = 0.1;
%fitBackward = stepwiselm(truenaivefitds, 'linear', 'ResponseVar', 'diff_phase_deg', ...
%    'Intercept', true, 'Criterion', 'sse', 'Upper', 'linear', ...
%    'Penter', premove/2, 'PRemove', premove);

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

%% Clean up workspace
close all;
