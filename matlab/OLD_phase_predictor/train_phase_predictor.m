clear all
MIN_NUM_IMGS = 2
MIN_NUM_TAGS = 13
OUTLIER_NUM_STDS = 3
load_file = 1
filename = 'variables_loaded.mat'
if load_file == 1
    disp 'Loading file'
    %tag_data = load('/media/dacocp/4DB4FDD92C569739/ftag2bags/multi_image_per_pose/var_predictor_2014-10-28-01-49-05.bag.mat');
    %tag_data = load('/media/dacocp/4DB4FDD92C569739/ftag2bags/multi_image_per_pose/var_predictor_2014-10-28-19-02-41.bag.mat');
    tag_data = load('var_predictor_2014-10-28-01-49-05_new_2015-04-06-16-19-47.bag.mat')
    tag_data = tag_data.tag_data;
    disp '1'
    orig_poses = zeros(length(tag_data), 6); for j = 1:6; orig_poses(:, j) = cellfun(@(t) t.tags{1}.pose_rpy(j), tag_data)'; end;
    disp '2'
    orig_phases = zeros(length(tag_data), 30); for j = 1:30; orig_phases(:, j) = cellfun(@(t) t.tags{1}.phases(j), tag_data)'; end;
    %disp '3'
    %orig_mags = zeros(length(tag_data), 30); for j = 1:30; orig_mags(:, j) = cellfun(@(t) t.tags{1}.mags(j), tag_data)'; end;
    disp '4'
    orig_markerPixelWidth = zeros(length(tag_data), 1); orig_markerPixelWidth(:) = cellfun(@(t) t.tags{1}.markerPixelWidth, tag_data)'; 
    disp '4.1'
    orig_tagImgRot = zeros(length(tag_data), 1); orig_tagImgRot(:) = cellfun(@(t) t.tags{1}.tagImgRot, tag_data)'; 
    disp '5'
    for i = 1:length(tag_data), tag_data{i}.ground_truth = arrayfun(@(d) uint8(str2double(d)), strrep(tag_data{i}.ground_truth_payload, '_', '')); end
    disp '6'
    orig_ground_truth_decimal = zeros(length(tag_data), 30); for j = 1:30; orig_ground_truth_decimal(:, j) = cellfun(@(t) t.ground_truth(j), tag_data)'; end;
    disp '7'
    orig_ground_truth_phases = orig_ground_truth_decimal()*360/8;
    %disp '8'
    %orig_frameID = cellfun(@(t) t.frameID, tag_data)';
    disp '9'
    orig_image_count = cellfun(@(t) t.total_image_count, tag_data)';
    %disp '10'
    %orig_tag_count_in_pose = cellfun(@(t) t.tag_count_in_pose, tag_data)';
    %disp '11'
    %orig_detection_count = cellfun(@(t) t.detection_count, tag_data)';
    disp '12'
    orig_pose_count = cellfun(@(t) t.pose_count, tag_data)';
    %disp '13'
    %orig_num_successful_detections = cellfun(@(t) t.num_successful_detections, tag_data)';
    disp '14'
    save(filename)
    disp 'Variables saved'
else
    load(filename);
    disp 'Variables loaded'
end
%%

orig_diffs = angleDiff(orig_phases, orig_ground_truth_phases);

for i = 1:30
    diffs_per_phase = orig_diffs(:,i);
    mean_diffs_per_phase = mean(diffs_per_phase);
    std_diffs_per_phase = std(diffs_per_phase);
    idx = mean_diffs_per_phase - std_diffs_per_phase * OUTLIER_NUM_STDS < orig_diffs(:,i) & orig_diffs(:,i) < mean_diffs_per_phase + std_diffs_per_phase * OUTLIER_NUM_STDS;
    orig_diffs = orig_diffs(idx,:);
    orig_ground_truth_decimal = orig_ground_truth_decimal(idx,:);
    orig_ground_truth_phases = orig_ground_truth_phases(idx,:);
    orig_phases = orig_phases(idx,:);
    %orig_frameID = orig_frameID(idx,:);
    orig_image_count = orig_image_count(idx,:); 
    %orig_tag_count_in_pose = orig_tag_count_in_pose(idx,:);
    %orig_detection_count = orig_detection_count(idx,:);
    orig_pose_count = orig_pose_count(idx,:);
    %orig_num_successful_detections = orig_num_successful_detections(idx,:);
    orig_poses = orig_poses(idx,:);
    orig_markerPixelWidth = orig_markerPixelWidth(idx,:);
    orig_tagImgRot = orig_tagImgRot(idx,:);
    %mags = mags(idx,:);
end
%%

Titles = {'img_ct', 'tag_ct_i_p', 'det_ct', 'pos_ct', 'num_ss_det', 'frameID', 'poses', 'poses','poses','poses','poses','poses', 'mpw' };
X = [];
for i = 1:1000
    %X = [ X; [ repmat([image_count(i), tag_count_in_pose(i), detection_count(i), pose_count(i), num_successful_detections(i), frameID(i), poses(i,:), markerPixelWidth(i)],4,1) ], [diffs(i,:); ground_truth_decimal(i,:); ground_truth_phases(i,:); phases(i,:) ] ];
    %X = [ X; [orig_ima_tagImgRotge_count(i), orig_tag_count_in_pose(i), orig_detection_count(i), orig_pose_count(i), orig_num_successful_detections(i), orig_frameID(i), orig_poses(i,:), orig_markerPixelWidth(i)] ];
    X = [ X; [orig_image_count(i), orig_pose_count(i), orig_poses(i,:), orig_markerPixelWidth(i), orig_tagImgRot(i)] ];
    %X = [ X; 99999*ones(1,size(X,2)); 99999*ones(1,size(X,2)) ];
end
%%

tag_diffs_means = [];
tag_diffs_stds = [];
tag_poses_means = [];
tag_poses_stds = [];
tag_markerPixelWidth_means = [];
tag_markerPixelWidth_stds = [];
tag_ground_truth_phases = [];
tag_pose_count = [];
tag_tagImgRot = [];
for i = 1:max(orig_image_count)
    batch_of_diffs = orig_diffs(orig_image_count==i,:);
    batch_of_poses = orig_poses(orig_image_count==i,:);
    batch_of_ground_truth_phases = orig_ground_truth_phases(orig_image_count==i,:);
    batch_of_markerPixelWidths = orig_markerPixelWidth(orig_image_count==i,:);
    batch_of_tagImgRots = orig_tagImgRot(orig_image_count==i,:);
    if size(batch_of_diffs,1) >= MIN_NUM_IMGS
        tag_diffs_means = [tag_diffs_means; mean(batch_of_diffs)];
        tag_diffs_stds = [tag_diffs_stds; std(batch_of_diffs)];
        
        tag_poses_means = [tag_poses_means; mean(batch_of_poses)];
        tag_poses_stds = [tag_poses_stds; std(batch_of_poses)];
        
        tag_markerPixelWidth_means = [tag_markerPixelWidth_means; mean(batch_of_markerPixelWidths)];
        tag_markerPixelWidth_stds = [tag_markerPixelWidth_stds; std(batch_of_markerPixelWidths)];
        
        tag_ground_truth_phases = [ tag_ground_truth_phases; batch_of_ground_truth_phases(1,:) ];
        curr_pose_count = orig_pose_count(orig_image_count == i);
        curr_pose_count = curr_pose_count(1);
        tag_pose_count = [ tag_pose_count; curr_pose_count];
        
        tag_tagImgRot = [ tag_tagImgRot; batch_of_tagImgRots ];
    end
end
%%

pose_diffs_means = [];
pose_diffs_stds = [];
pose_poses_means = [];
pose_poses_stds = [];
pose_markerPixelWidth_means = [];
pose_markerPixelWidth_stds = [];
pose_ground_truth_phases = [];
pose_pose_count = [];
tagImgRot = [];
for i = 1:max(tag_pose_count)
    batch_of_diffs = tag_diffs_means(tag_pose_count==i,:);
    batch_of_poses = tag_poses_means(tag_pose_count==i,:);
    batch_of_ground_truth_phases = tag_ground_truth_phases(tag_pose_count==i,:);
    batch_of_markerPixelWidths = tag_markerPixelWidth_means(tag_pose_count==i,:);
    batch_of_tagImgRots = tag_tagImgRot(tag_pose_count==i,:);
    if size(batch_of_diffs,1) >= MIN_NUM_TAGS
        pose_diffs_means = [pose_diffs_means; mean(batch_of_diffs)];
        pose_diffs_stds = [pose_diffs_stds; std(batch_of_diffs)];
        
        pose_poses_means = [pose_poses_means; mean(batch_of_poses)];
        pose_poses_stds = [pose_poses_stds; std(batch_of_poses)];
        
        pose_markerPixelWidth_means = [pose_markerPixelWidth_means; mean(batch_of_markerPixelWidths)];
        pose_markerPixelWidth_stds = [pose_markerPixelWidth_stds; std(batch_of_markerPixelWidths)];
        
        pose_ground_truth_phases = [ pose_ground_truth_phases; batch_of_ground_truth_phases(1,:) ];
        curr_pose_count = orig_pose_count(tag_pose_count == i);
        curr_pose_count = curr_pose_count(1);
        pose_pose_count = [ pose_pose_count; curr_pose_count];
        tagImgRot = [ tagImgRot; median(batch_of_tagImgRots) ];
    end
end
%%

%phases_in_rows_std = [];
%phases_in_rows_mean = [];
%for i = 1:size(tag_diffs_means,1)
%    means_rows = reshape([tag_diffs_means(i,:)]', 5, 6)';
%    phases_in_rows_mean = [ phases_in_rows_mean; mean(means_rows) ];
%    phases_in_rows_std = [ phases_in_rows_std; std(means_rows) ];
%end
%%

x = [];
y = [];
freqs = [];
freqs_num_bits = [];
freq_diffs_means = [];
freq_diffs_stds = [];
freq_phases_in_rows_stds = [];
freq_poses = [];
freqs_ground_truth_phases = [];
freq_markerPixelWidth = [];
freq_tagImgRot = [];
for freq = 1:5
    for row = 1:6
        freq_col_idx = (freq-1)+5*(row-1)+1;
        freq_diffs_means = [ freq_diffs_means; pose_diffs_means(:,freq_col_idx) ];
        freq_diffs_stds = [ freq_diffs_stds; pose_diffs_stds(:,freq_col_idx) ];
        freq_poses = [ freq_poses; pose_poses_means ];
        freqs_ground_truth_phases = [ freqs_ground_truth_phases; pose_ground_truth_phases];
        freq_markerPixelWidth = [ freq_markerPixelWidth; pose_markerPixelWidth_means ] ;
        freq_tagImgRot = [ freq_tagImgRot; tagImgRot ];
        freqs = [freqs; ones(size(pose_diffs_means(:,freq_col_idx))) * freq ];
        if freq <= 3
            freqs_num_bits = [freqs_num_bits; ones(size(pose_diffs_means(:,freq_col_idx))) * 3 ];
        else
            freqs_num_bits = [freqs_num_bits; ones(size(pose_diffs_means(:,freq_col_idx))) * 2 ];
        end
    end
    y = [y,0];
end

%for freq = 1:5
%    freq_col_idx = freq;
%    freq_diffs_means = [ freq_diffs_means; phases_in_rows_mean(:,freq_col_idx) ];
%    freq_diffs_stds = [ freq_diffs_stds; phases_in_rows_std(:,freq_col_idx) ];
%    freq_poses = [ freq_poses; tag_poses_means ];
%    freqs_ground_truth_phases = [ freqs_ground_truth_phases; tag_ground_truth_phases];
%    freq_markerPixelWidth = [ freq_markerPixelWidth; tag_markerPixelWidth_means ] ;
%    freqs = [freqs; ones(size(tag_diffs_means(:,freq_col_idx))) * freq ];
%    if freq <= 3
%        freqs_num_bits = [freqs_num_bits; ones(size(tag_diffs_means(:,freq_col_idx))) * 3 ];
%    else
%        freqs_num_bits = [freqs_num_bits; ones(size(tag_diffs_means(:,freq_col_idx))) * 2 ];
%    end
%end
%%

ph = 180;
phase_180 = [orig_ground_truth_phases(pose_ground_truth_phases(:,1)==ph,1:5), orig_phases(pose_ground_truth_phases(:,1)==ph,1:5)];
%phase_180 = [ground_truth_phases(:,1:5), phases(:,1:5)];
diff_nomod = phase_180(:,1)-phase_180(:,6);
sync_phase_diff = angleDiff( phase_180(:,1) , phase_180(:,6) );
%sync_phase_diff = diff_nomod
%sync_phase_diff( abs(diff_nomod) >180 & diff_nomod >= 0 ) = sync_phase_diff( abs(diff_nomod) >180 & diff_nomod >= 0 ) - 360
%sync_phase_diff( abs(diff_nomod) >180 & diff_nomod < 0 ) = sync_phase_diff( abs(diff_nomod) >180 & diff_nomod < 0 ) + 360
new_phases = zeros(size(phase_180,1),5);
for f = 1:5
    col = f+5;
    new_phases(:,f) = mod(phase_180(:,col) + sync_phase_diff, 360);
end

phase_180 = [phase_180, new_phases];
phase_diffs = [ angleDiff( phase_180(:,6:10), phase_180(:,1:5) ), angleDiff( phase_180(:,11:15), phase_180(:,1:5) ) ];
%phase_diffs = [ mod(phase_180(:,6:10) - phase_180(:,1:5), 360), mod(phase_180(:,11:15)-phase_180(:,1:5),360) ];
mean(phase_diffs)



%%
freq_pos_x = freq_poses(:,1);
freq_pos_y = freq_poses(:,2);
freq_pos_z = freq_poses(:,3);
freq_rot_r = freq_poses(:,4);
freq_rot_p = freq_poses(:,5);
freq_rot_y = freq_poses(:,6);

freq_pos_x_norm = ( freq_pos_x - mean(freq_pos_x) )/ std(freq_pos_x);
freq_pos_y_norm = ( freq_pos_y - mean(freq_pos_y) )/ std(freq_pos_y);
freq_pos_z_norm = ( freq_pos_z - mean(freq_pos_z) )/ std(freq_pos_z);
freq_rot_r_norm = ( freq_rot_r - mean(freq_rot_r) )/ std(freq_rot_r);
freq_rot_p_norm = ( freq_rot_p - mean(freq_rot_p) )/ std(freq_rot_p);
freq_rot_y_norm = ( freq_rot_y - mean(freq_rot_y) )/ std(freq_rot_y);

freq_radius = sqrt(freq_pos_x.^2 + freq_pos_y.^2);
freq_radius_norm = ( freq_radius - mean(freq_radius) )/ std(freq_radius);
freq_rho = sqrt(freq_radius.^2 + freq_pos_z.^2);
freq_rho_norm = ( freq_rho - mean(freq_rho) )/ std(freq_rho);

freq_markerPixelWidth_norm = (freq_markerPixelWidth - mean(freq_markerPixelWidth)) / std(freq_markerPixelWidth);
%% 

marker_Power = 2.^(freq_markerPixelWidth_norm);
yaw = 2.^(freq_rot_y_norm)
tbl = dataset(  freqs, log(freq_diffs_stds));
%ff = fitlm(tbl,'interactions')
%f1 = LinearModel.stepwise(tbl)
%f3 = LinearModel.stepwise(tbl,'interactions')

[X,Y] = meshgrid( [0:400]', [1:0.1:5]' );
XX = X(:);
YY = Y(:);
%ZZ = f3.feval(XX,YY);
%scatter3(XX,YY,ZZ)
hold on
scatter(freq_markerPixelWidth_norm, freq_diffs_stds)
hold off

%f2 = LinearModel.stepwise(tbl,'interactions')
%f3 = LinearModel.fit(tbl,'interactions')
%%%%%%

tbl = dataset(freq_pos_z, freq_markerPixelWidth, freq_rot_p, freq_rot_y, freqs, freq_diffs_means);
f3 = LinearModel.stepwise(tbl,'interactions');
%f4 = LinearModel.fit(tbl)
%f5 = LinearModel.fit(tbl,'interactions')

%%

%tbl = dataset(freq_markerPixelWidth, freq_rot_p, freq_rot_y, freqs, freq_diffs_stds);
%f5 = LinearModel.stepwise(tbl,'interactions')

%tbl = dataset(freq_markerPixelWidth, freq_rot_p, freq_rot_y, freqs, freq_diffs_stds);
%f6 = LinearModel.stepwise(tbl,'interactions')

%X = [ freqs, freq_diffs_means, freq_diffs_stds];


