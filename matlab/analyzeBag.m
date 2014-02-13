clear all;
data = load('dump.csv');

%  1: batch_id
%  2: segment_id
%  3: pan_angle
%  4: tilt_angle
%  5: display_tag_rotation
%  6: frame_id
%  7: position_x
%  8: position_y
%  9: position_z
% 10: orientation_w
% 11: orientation_x
% 12: orientation_y
% 13: orientation_z
% 14: marker_width
% 15-44: detected_mags
% 45-74: detected_phases
% 75-104: encoded_phases

%%
detected_phases = data(:, 45:74);
encoded_phases = data(:, 75:104);
diff_phases = angularDiff(detected_phases, encoded_phases, 360);
freq_idx = repmat(1:5, 1, 6);

%%

clear entries;
entries = {};

max_batch_id = data(end, 1);
for batch_id = 0:max_batch_id,
    curr_data = data(data(:, 1) == batch_id, :);
    curr_diff_phases = diff_phases(data(:, 1) == batch_id, :);
    
    num_entries = size(curr_data, 1);
    if num_entries <= 0,
        continue;
    end;
    
    pan_angle = curr_data(1, 3);
    tilt_angle = curr_data(1, 4);
    display_tag_rotation = curr_data(1, 5);
    
    position_xyz = mean(curr_data(:, 7:9));
    curr_orientations_xyzw = [curr_data(:, 11:13), curr_data(:, 10)];
    orientation_xyzw = avg_quaternion_markley(curr_orientations_xyzw);
    
    marker_widths = curr_data(:, 14);
    
    for freq = 1:5,
        diff = (curr_diff_phases(:, freq_idx == freq));
        diff = diff(:);
        errors = abs(diff);
        
        clear curr_entry;
        curr_entry.batch_id = batch_id;
        curr_entry.num_entries = num_entries;
        curr_entry.pan_angle = pan_angle;
        curr_entry.tilt_angle = tilt_angle;
        curr_entry.display_tag_rotation = display_tag_rotation;
        curr_entry.position_xyz = position_xyz;
        curr_entry.orientation_xyzw = orientation_xyzw;
        curr_entry.marker_width_avg = mean(marker_widths);
        curr_entry.marker_width_max = max(marker_widths);
        curr_entry.freq = freq;
        curr_entry.phase_diff_avg = mean(diff);
        curr_entry.phase_diff_std = std(diff);
        curr_entry.phase_diff_min = min(diff);
        curr_entry.phase_diff_max = max(diff);
        curr_entry.phase_error_avg = mean(errors);
        curr_entry.phase_error_std = std(errors);
        curr_entry.phase_errors_min = min(errors);
        curr_entry.phase_errors_max = max(errors);

        entries{length(entries)+1} = curr_entry;
    end;
end;
