clear all
%MIN_NUM_IMGS = 10
%OUTLIER_NUM_STDS = 3
%tag_data = load('final_var_predictor_2014-09-18-20-16-53.bag.mat');
%tag_data = tag_data.tag_data;
%disp '1'
%poses = zeros(length(tag_data), 6); for j = 1:6; poses(:, j) = cellfun(@(t) t.tags{1}.pose_rpy(j), tag_data)'; end;
%disp '2'
%phases = zeros(length(tag_data), 30); for j = 1:30; phases(:, j) = cellfun(@(t) t.tags{1}.phases(j), tag_data)'; end;
%disp '3'
%mags = zeros(length(tag_data), 30); for j = 1:30; mags(:, j) = cellfun(@(t) t.tags{1}.mags(j), tag_data)'; end;
%disp '4'
%markerPixelWidth = zeros(length(tag_data), 1); markerPixelWidth(:) = cellfun(@(t) t.tags{1}.markerPixelWidth, tag_data)'; 
%disp '5'
%for i = 1:length(tag_data), tag_data{i}.ground_truth = arrayfun(@(d) uint8(str2double(d)), strrep(tag_data{i}.ground_truth_payload, '_', '')); end
%disp '6'
%ground_truth_decimal = zeros(length(tag_data), 30); for j = 1:30; ground_truth_decimal(:, j) = cellfun(@(t) t.ground_truth(j), tag_data)'; end;
%disp '7'
%ground_truth_phases = ground_truth_decimal()*360/8;
%disp '8'
%frameID = cellfun(@(t) t.frameID, tag_data)';
%disp '9'
%image_count = cellfun(@(t) t.image_count, tag_data)';
%disp '10'
%image_count_in_pos = cellfun(@(t) t.image_count_in_pos, tag_data)';
%disp '11'

filename = 'variables_loaded.mat'
%save(filename)
load(filename);

%{
diffs = zeros(size(ground_truth_phases));
for i=1:size(diffs,1)
    for j=1:size(diffs,2)
        if ground_truth_decimal(i,j) == 0
            d0 = phases(i,j) -  ground_truth_phases(i,j);
            d360 = phases(i,j) -  360;
            if abs(d0) < abs(d360)
                diffs(i,j) = d0;
            else
                diffs(i,j) = d360;
            end
        else 
            diffs(i,j) = phases(i,j) - ground_truth_phases(i,j);
        end
    end
end
%}

diffs = angleDiff(phases, ground_truth_phases);

X = [ground_truth_decimal(1,:); ground_truth_phases(1,:); phases(1,:); diffs(1,:)];

%diffs_means = zeros(max(image_count),30);
%diffs_stds = zeros(max(image_count),30);

for i = 1:30
    diffs_per_phase = diffs(:,i);
    std_diffs_per_phase = std(diffs_per_phase);
    idx = abs(diffs(:,i)) < std_diffs_per_phase * OUTLIER_NUM_STDS;
    diffs = diffs(idx,:);
    ground_truth_decimal = ground_truth_decimal(idx,:);
    ground_truth_phases = ground_truth_phases(idx,:);
    phases = phases(idx,:);
    frameID = frameID(idx,:);
    image_count = image_count(idx,:); 
    image_count_in_pos = image_count_in_pos(idx,:);
    poses = poses(idx,:);
    markerPixelWidth = markerPixelWidth(idx,:);
    %mags = mags(idx,:);
end

diffs_means = [];
diffs_stds = [];
poses_means = [];
poses_stds = [];
markerPixelWidth_means = [];
markerPixelWidth_stds = [];
ground_truth_phases_vector = [];
for i = 1:max(image_count)
    batch_of_diffs = diffs(image_count==i,:);
    batch_of_poses = poses(image_count==i,:);
    batch_of_ground_truth_phases = ground_truth_phases(image_count==i,:);
    batch_of_markerPixelWidths = markerPixelWidth(image_count==i,:);
    if size(batch_of_diffs,1) >= MIN_NUM_IMGS
        diffs_means = [diffs_means; mean(batch_of_diffs)];
        diffs_stds = [diffs_stds; std(batch_of_diffs)];
        
        poses_means = [poses_means; mean(batch_of_poses)];
        poses_stds = [poses_stds; std(batch_of_poses)];
        
        markerPixelWidth_means = [markerPixelWidth_means; mean(batch_of_markerPixelWidths)];
        markerPixelWidth_stds = [markerPixelWidth_stds; std(batch_of_markerPixelWidths)];
        
        ground_truth_phases_vector = [ ground_truth_phases_vector; batch_of_ground_truth_phases ];
    end
end


x = [];
y = [];
freqs = [];
freqs_num_bits = [];
freq_diffs_means = [];
freq_diffs_stds = [];
freq_poses = [];
freqs_ground_truth_phases = [];
freq_markerPixelWidth = [];
for freq = 1:5
    for row = 1:6
        freq_col_idx = (freq-1)+5*(row-1)+1;
        freq_diffs_means = [ freq_diffs_means; diffs_means(:,freq_col_idx) ];
        freq_diffs_stds = [ freq_diffs_stds; diffs_stds(:,freq_col_idx) ];
        freq_poses = [ freq_poses; poses_means ];
        freqs_ground_truth_phases = [ freqs_ground_truth_phases; ground_truth_phases_vector];
        freq_markerPixelWidth = [ freq_markerPixelWidth; markerPixelWidth_means ] ;
        freqs = [freqs; ones(size(diffs_means(:,freq_col_idx))) * freq ];
        if freq <= 3
            freqs_num_bits = [freqs_num_bits; ones(size(diffs_means(:,freq_col_idx))) * 3 ];
        else
            freqs_num_bits = [freqs_num_bits; ones(size(diffs_means(:,freq_col_idx))) * 2 ];
        end
    end
    y = [y,0];
end
%%

ph = 180;
phase_180 = [ground_truth_phases(ground_truth_phases(:,1)==ph,1:5), phases(ground_truth_phases(:,1)==ph,1:5)];
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
f3 = LinearModel.stepwise(tbl,'interactions')

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

tbl = dataset(freq_markerPixelWidth, freqs_num_bits, freq_rot_p, freq_rot_y, freqs, freq_diffs_stds);
f3 = LinearModel.stepwise(tbl,'interactions')
%f4 = LinearModel.fit(tbl)
%f5 = LinearModel.fit(tbl,'interactions')

%%

%tbl = dataset(freq_markerPixelWidth, freq_rot_p, freq_rot_y, freqs, freq_diffs_stds);
%f5 = LinearModel.stepwise(tbl,'interactions')

%tbl = dataset(freq_markerPixelWidth, freq_rot_p, freq_rot_y, freqs, freq_diffs_stds);
%f6 = LinearModel.stepwise(tbl,'interactions')

%X = [ freqs, freq_diffs_means, freq_diffs_stds];


