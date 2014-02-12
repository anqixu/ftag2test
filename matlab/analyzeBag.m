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
phase_errors = abs(diff_phases);

%%

% For the same pan and tilt angle, aggregate all errors on 