%% Reset workspace

clear all;
INIT_WORKSPACE;

%% Specify configuration and initialize constants
images_dir = '../ftag2_datasets/6S2F22B_random_set/';
trials_dir = '../ftag2_datasets/trials/';
tag_type = '6s2f22b';
rng_seed = 1731;

num_rand_poses = 2000;
num_rand_tags_per_pose = 10;

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

rng(rng_seed);

if exist(trials_dir, 'dir') ~= 7,
  mkdir(trials_dir);
  fprintf('Created folder: %s\n', trials_dir);
end

%% Enumerate all images in dataset folder

tag_files = dir(fullfile(pwd, images_dir, '*.png'));
num_tag_files = length(tag_files);

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
  
  rand_ids = randperm(num_tag_files);
  for tag_j = 1:num_rand_tags_per_pose,
    t.tag_source = fullfile(pwd, images_dir, tag_files(rand_ids(tag_j)).name);
    target_seq{length(target_seq)+1} = t; %#ok<*SAGROW>
  end
end

clear target_trials;
target_trials{1}.target_seq = target_seq;
target_trials{1}.label = strcat(tag_type, '_random');

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
