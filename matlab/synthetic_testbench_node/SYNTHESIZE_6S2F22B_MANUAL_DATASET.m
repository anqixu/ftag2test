%% Reset workspace

clear all;
INIT_WORKSPACE;

%% Specify configuration and initialize constants
images_dir = '../ftag2_datasets/6S2F22B_manual_set/';
trials_dir = '../ftag2_datasets/trials/';
rng_seed = 1729;

sweep_num_samples = 1000;

tag_width_m = 0.125;

tag_tx_m_dft = 0.;
tag_tx_m_min = -0.5;
tag_tx_m_max = 0.5;

tag_ty_m_dft = 0.;
tag_ty_m_min = -0.4;
tag_ty_m_max = 0.4;

tag_tz_m_dft = 1.0;
tag_tz_m_min = 0.1;
tag_tz_m_max = 2.0;

tag_pitch_deg_dft = 0.;
tag_pitch_deg_min = -85.0;
tag_pitch_deg_max = 85.0;

tag_yaw_deg_dft = 0.;
tag_yaw_deg_min = -85.0;
tag_yaw_deg_max = 85.0;

tag_roll_deg_dft = 0.;
tag_roll_deg_min = 0.;
tag_roll_deg_max = 360.0;

rng(rng_seed);

if exist(trials_dir, 'dir') ~= 7,
  mkdir(trials_dir);
  fprintf('Created folder: %s\n', trials_dir);
end

%% Enumerate all images in dataset folder

tag_files = dir(fullfile(pwd, images_dir, '*.png'));
num_tag_files = length(tag_files);

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
for file_idx = 1:num_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, images_dir, tag_files(file_idx).name);
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
for file_idx = 1:num_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, images_dir, tag_files(file_idx).name);
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
for file_idx = 1:num_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, images_dir, tag_files(file_idx).name);
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
for file_idx = 1:num_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, images_dir, tag_files(file_idx).name);
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
for file_idx = 1:num_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, images_dir, tag_files(file_idx).name);
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
for file_idx = 1:num_tag_files,
  targets_template{1}.tag_source = fullfile(pwd, images_dir, tag_files(file_idx).name);
  targets_sweep_roll = [targets_sweep_roll, targets_template];
end

%% Aggregate trials
target_trials = {};
clear t;
t.target_seq = targets_sweep_tx;
t.label = '6s2f22b_sweep_tx';
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_ty;
t.label = '6s2f22b_sweep_ty';
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_tz;
t.label = '6s2f22b_sweep_tz';
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_pitch;
t.label = '6s2f22b_sweep_pitch';
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_yaw;
t.label = '6s2f22b_sweep_yaw';
target_trials{length(target_trials)+1} = t;
t.target_seq = targets_sweep_roll;
t.label = '6s2f22b_sweep_roll';
target_trials{length(target_trials)+1} = t;

%% Run trials

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
