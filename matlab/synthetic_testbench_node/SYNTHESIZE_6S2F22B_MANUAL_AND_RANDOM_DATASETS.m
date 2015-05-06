%% Reset workspace

clear all;
INIT_WORKSPACE;

%% Specify configuration and initialize constants for manual set
manual_set_dir = '../ftag2_datasets/6S2F22B_manual_set/';
random_set_dir = '../ftag2_datasets/6S2F22B_random_set/';
trials_dir = '../ftag2_datasets/trials/';
tag_type = '6s2f22b';
rng_seed = 1730;

sweep_num_samples = 1000;

num_rand_poses = 1000;
num_rand_tags_per_pose = 6;

% NOTE: following are for manual sweep trials only
tag_width_m = 0.1;

tag_tx_m_dft = 0.;
tag_tx_m_min = -0.55;
tag_tx_m_max = 0.55;

tag_ty_m_dft = 0.;
tag_ty_m_min = -0.4;
tag_ty_m_max = 0.4;

tag_tz_m_dft = 1.0;
tag_tz_m_min = 0.1;
tag_tz_m_max = 2.5;

tag_pitch_deg_dft = 0.;
tag_pitch_deg_min = -60.0; % WARNING: beyond 75' sometimes sig+mag mis-detected as rotated angle
tag_pitch_deg_max = 60.0; % also, beyond 65', significant phase errors could result due to inaccurate quad corners

tag_yaw_deg_dft = 0.;
tag_yaw_deg_min = -60.0;
tag_yaw_deg_max = 60.0;

tag_roll_deg_dft = 0.;
tag_roll_deg_min = 0.;
tag_roll_deg_max = 360.0;

rng(rng_seed);

if exist(trials_dir, 'dir') ~= 7,
  mkdir(trials_dir);
  fprintf('Created folder: %s\n', trials_dir);
end

%% Enumerate all images in dataset folder

manual_tag_files = dir(fullfile(pwd, manual_set_dir, '*.png'));
num_manual_tag_files = length(manual_tag_files);

%% Uniformly sweep through tx
targets_template = {};

clear t;
t.tag_width_m = tag_width_m;
t.tag_tx_m = tag_tx_m_dft;
t.tag_ty_m = tag_ty_m_dft;
t.tag_tz_m = tag_tz_m_dft;
t.tag_rx_deg = tag_pitch_deg_dft;
t.tag_ry_deg = tag_yaw_deg_dft;
t.tag_rz_deg = tag_roll_deg_dft;

for tag_tx_m = linspace(tag_tx_m_min, tag_tx_m_max, sweep_num_samples),
  t.tag_tx_m = tag_tx_m;
  targets_template{length(targets_template)+1} = t; %#ok<*SAGROW>
  clear t;
end

targets_sweep_tx = {};
for file_idx = 1:num_manual_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, manual_set_dir, manual_tag_files(file_idx).name);
  targets_sweep_tx = [targets_sweep_tx, targets_template]; %#ok<*AGROW>
end

%% Uniformly sweep through ty
targets_template = {};

clear t;
t.tag_width_m = tag_width_m;
t.tag_tx_m = tag_tx_m_dft;
t.tag_ty_m = tag_ty_m_dft;
t.tag_tz_m = tag_tz_m_dft;
t.tag_rx_deg = tag_pitch_deg_dft;
t.tag_ry_deg = tag_yaw_deg_dft;
t.tag_rz_deg = tag_roll_deg_dft;

for tag_ty_m = linspace(tag_ty_m_min, tag_ty_m_max, sweep_num_samples),
  t.tag_ty_m = tag_ty_m;
  targets_template{length(targets_template)+1} = t;
  clear t;
end

targets_sweep_ty = {};
for file_idx = 1:num_manual_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, manual_set_dir, manual_tag_files(file_idx).name);
  targets_sweep_ty = [targets_sweep_ty, targets_template];
end

%% Uniformly sweep through tz
targets_template = {};

clear t;
t.tag_width_m = tag_width_m;
t.tag_tx_m = tag_tx_m_dft;
t.tag_ty_m = tag_ty_m_dft;
t.tag_tz_m = tag_tz_m_dft;
t.tag_rx_deg = tag_pitch_deg_dft;
t.tag_ry_deg = tag_yaw_deg_dft;
t.tag_rz_deg = tag_roll_deg_dft;

for tag_tz_m = linspace(tag_tz_m_min, tag_tz_m_max, sweep_num_samples),
  t.tag_tz_m = tag_tz_m;
  targets_template{length(targets_template)+1} = t;
  clear t;
end

targets_sweep_tz = {};
for file_idx = 1:num_manual_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, manual_set_dir, manual_tag_files(file_idx).name);
  targets_sweep_tz = [targets_sweep_tz, targets_template];
end

%% Uniformly sweep through pitch
targets_template = {};

clear t;
t.tag_width_m = tag_width_m;
t.tag_tx_m = tag_tx_m_dft;
t.tag_ty_m = tag_ty_m_dft;
t.tag_tz_m = tag_tz_m_dft;
t.tag_rx_deg = tag_pitch_deg_dft;
t.tag_ry_deg = tag_yaw_deg_dft;
t.tag_rz_deg = tag_roll_deg_dft;

for tag_pitch_deg = linspace(tag_pitch_deg_min, tag_pitch_deg_max, sweep_num_samples),
  t.tag_rx_deg = tag_pitch_deg;
  targets_template{length(targets_template)+1} = t;
  clear t;
end

targets_sweep_pitch = {};
for file_idx = 1:num_manual_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, manual_set_dir, manual_tag_files(file_idx).name);
  targets_sweep_pitch = [targets_sweep_pitch, targets_template];
end

%% Uniformly sweep through pitch
targets_template = {};

clear t;
t.tag_width_m = tag_width_m;
t.tag_tx_m = tag_tx_m_dft;
t.tag_ty_m = tag_ty_m_dft;
t.tag_tz_m = tag_tz_m_dft;
t.tag_rx_deg = tag_pitch_deg_dft;
t.tag_ry_deg = tag_yaw_deg_dft;
t.tag_rz_deg = tag_roll_deg_dft;

for tag_yaw_deg = linspace(tag_yaw_deg_min, tag_yaw_deg_max, sweep_num_samples),
  t.tag_ry_deg = tag_yaw_deg;
  targets_template{length(targets_template)+1} = t;
  clear t;
end

targets_sweep_yaw = {};
for file_idx = 1:num_manual_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, manual_set_dir, manual_tag_files(file_idx).name);
  targets_sweep_yaw = [targets_sweep_yaw, targets_template];
end

%% Uniformly sweep through roll
targets_template = {};

clear t;
t.tag_width_m = tag_width_m;
t.tag_tx_m = tag_tx_m_dft;
t.tag_ty_m = tag_ty_m_dft;
t.tag_tz_m = tag_tz_m_dft;
t.tag_rx_deg = tag_pitch_deg_dft;
t.tag_ry_deg = tag_yaw_deg_dft;
t.tag_rz_deg = tag_roll_deg_dft;

for tag_roll_deg = linspace(tag_roll_deg_min, tag_roll_deg_max, sweep_num_samples),
  t.tag_rz_deg = tag_roll_deg;
  targets_template{length(targets_template)+1} = t;
  clear t;
end

targets_sweep_roll = {};
for file_idx = 1:num_manual_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, manual_set_dir, manual_tag_files(file_idx).name);
  targets_sweep_roll = [targets_sweep_roll, targets_template];
end

%% Initialize constants for random set

tag_width_m_dft = 0.1; % WARNING: do not change tag size since this complicates FTag2 decoder node

%tag_tx_m_dft = 0.;
tag_tx_m_min = -0.3;
tag_tx_m_max = 0.3;

%tag_ty_m_dft = 0.;
tag_ty_m_min = -0.25;
tag_ty_m_max = 0.25;

%tag_tz_m_dft = 1.0;
tag_tz_m_min = 0.5;
tag_tz_m_max = 1.5;

%tag_pitch_deg_dft = 0.;
tag_pitch_deg_min = -45.0;
tag_pitch_deg_max = 45.0;

%tag_yaw_deg_dft = 0.;
tag_yaw_deg_min = -45.0;
tag_yaw_deg_max = 45.0;

%tag_roll_deg_dft = 0.;
tag_roll_deg_min = 0.;
tag_roll_deg_max = 360.0;

max_tag_oop_deg = 50;

%% Enumerate all images in dataset folder

random_tag_files = dir(fullfile(pwd, random_set_dir, '*.png'));
num_random_tag_files = length(random_tag_files);

%% Generate random TagPose targets
target_seq = {};
clear t;
radians = pi/180;
for pose_i = 1:num_rand_poses,
  while true,
    %t.tag_width_m = rand()*(tag_width_m_max-tag_width_m_min) + tag_width_m_min;
    t.tag_width_m = tag_width_m_dft;
    t.tag_tx_m = rand()*(tag_tx_m_max-tag_tx_m_min) + tag_tx_m_min;
    t.tag_ty_m = rand()*(tag_ty_m_max-tag_ty_m_min) + tag_ty_m_min;
    t.tag_tz_m = rand()*(tag_tz_m_max-tag_tz_m_min) + tag_tz_m_min;
    t.tag_rx_deg = rand()*(tag_pitch_deg_max-tag_pitch_deg_min) + tag_pitch_deg_min;
    t.tag_ry_deg = rand()*(tag_yaw_deg_max-tag_yaw_deg_min) + tag_yaw_deg_min;
    t.tag_rz_deg = rand()*(tag_roll_deg_max-tag_roll_deg_min) + tag_roll_deg_min;
    
    % Reject pose if its out-of-plane angle is too steep
    [tag_pitch_rad, tag_yaw_rad, tag_roll_rad] = ...
      tf_euler_inverse(t.tag_rx_deg*radians, t.tag_ry_deg*radians, t.tag_rz_deg*radians);
    tag_oop_deg = tf_angle_between_euler_poses(tag_pitch_rad/radians, ...
      tag_yaw_rad/radians, tag_roll_rad/radians, 0, 0, 0);
    if tag_oop_deg > max_tag_oop_deg,
      %fprintf('Rejecting pose (%.4f, %.4f, %.4f) -> (%.4f, %.4f, %.4f) due to sharp OOP angle %.4f\n', ...
      %  t.tag_rx_deg, t.tag_ry_deg, t.tag_rz_deg, ...
      %  tag_pitch_rad/radians, tag_yaw_rad/radians, tag_roll_rad/radians, ...
      %  tag_oop_deg);
      continue;
    else
      break;
    end
  end
  
  rand_ids = randperm(num_random_tag_files);
  for tag_j = 1:num_rand_tags_per_pose,
    t.tag_source = fullfile(pwd, random_set_dir, random_tag_files(rand_ids(tag_j)).name);
    target_seq{length(target_seq)+1} = t; %#ok<*SAGROW>
  end
end

%% Aggregate manual trials
target_trials = {};
clear t;
t.target_seq = targets_sweep_tx;
t.label = strcat(tag_type, '_sweep_tx');
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_ty;
t.label = strcat(tag_type, '_sweep_ty');
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_tz;
t.label = strcat(tag_type, '_sweep_tz');
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_pitch;
t.label = strcat(tag_type, '_sweep_pitch');
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_yaw;
t.label = strcat(tag_type, '_sweep_yaw');
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_roll;
t.label = strcat(tag_type, '_sweep_roll');
target_trials{length(target_trials)+1} = t;
t.target_seq = target_seq;
t.label = strcat(tag_type, '_random');
target_trials{length(target_trials)+1} = t;

%% Run all trials

node = [];
for trial_i = 1:length(target_trials),
  t = target_trials{trial_i};
  fprintf('> TRIAL %d/%d: %s\n', trial_i, length(target_trials), t.label);
  if trial_i == 1,
    node = SyntheticTestbenchNode(t.target_seq, t.label);
  else
    node.reset(t.target_seq, t.label);
  end
  node.waitTillIdle(inf);
  dest_filename = sprintf('%s/%s.mat', trials_dir, t.label);
  copyfile(node.log_file, dest_filename);
end

fprintf('ALL DONE!\n');
delete(node);
clear node;
