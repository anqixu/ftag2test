tag_data = load('final_random_extracted_2014-09-12-00-29-54.bag.mat')
tag_data = tag_data.tag_data
poses = zeros(length(tag_data), 6); for j = 1:6; poses(:, j) = cellfun(@(t) t.tags{1}.pose_rpy(j), tag_data)'; end;
phases = zeros(length(tag_data), 30); for j = 1:30; phases(:, j) = cellfun(@(t) t.tags{1}.phases(j), tag_data)'; end;
mags = zeros(length(tag_data), 30); for j = 1:30; mags(:, j) = cellfun(@(t) t.tags{1}.mags(j), tag_data)'; end;
for i = 1:length(tag_data), tag_data{i}.ground_truth = arrayfun(@(d) uint8(str2double(d)), strrep(tag_data{i}.ground_truth_payload, '_', '')); end;
ground_truths = zeros(length(tag_data), 30); for j = 1:30; ground_truths(:, j) = cellfun(@(t) t.ground_truth(j), tag_data)'; end;
