clear all;
INIT_WORKSPACE;

target.tag_source = '/home/thalassa/anqixu/indigo_ws/1_ftag/src/tag_renderer/nodes/ftag2_6s2f22b_20_00_03_13_30_21.png';
target.tag_width_m = 0.125;
target.tag_tx_m = 0.0;
target.tag_ty_m = 0.0;
target.tag_tz_m = 0.7;
target.tag_rx_deg = 0.0;
target.tag_ry_deg = 0.0;
target.tag_rz_deg = 0.0;

NUM_SAMPLES = 200;
HALF_RANGE_DEG = 60;
target_seq = {};
for i=0:(NUM_SAMPLES-1),
  target.tag_ry_deg = (double(i)/(NUM_SAMPLES-1)-0.5)*2*HALF_RANGE_DEG;
  target_seq{i+1} = target;
end
      
% The constructor automatically starts processing the targets
if run_from_scratch,
  node = SyntheticTestbenchNode(target_seq, 'yaw sweep for single random tag');
  node.waitTillIdle();
else
  node.reset(target_seq, 'yaw sweep for single random tag');
  node.waitTillIdle();
end

stats = [];
failed_ids = [];
seq = node.progress_seq;
for i=1:NUM_SAMPLES,
  s = seq{i};
  if s.ftag2_num_tags_detected > 0,
    stats = [stats; ...
      i, ...
      s.tag_ry_deg, ...
      angularDiff(s.tag_phases_vec, s.ftag2_phases_vec), ...
      s.ftag2_tx_m-s.tag_tx_m, ...
      s.ftag2_ty_m-s.tag_ty_m, ...
      s.ftag2_tz_m-s.tag_tz_m, ...
      angularDiff(s.tag_rx_deg, s.ftag2_rx_deg), ...
      angularDiff(s.tag_ry_deg, s.ftag2_ry_deg), ...
      angularDiff(s.tag_rz_deg, s.ftag2_rz_deg)];
  else
    failed_ids = [failed_ids, i];
  end
end
