classdef SyntheticTestbenchNode < handle
  properties (Constant)
  end
  
  
  properties (SetAccess='protected')
    node
    %synth_img_sub
    decoded_tags_sub
    update_tag_pose_pub
    update_tag_source_pub
    
    ftag2_timer
    
    init_target_seq
    
    ftag2_frame_id_offset
    progress_seq_i % -1: idle, 0: waiting for stub TagPose, >0: waiting for target i's TagPose
    curr_target
    prev_target
  end
  
    
  properties
    progress_seq
    
    log_file
    test_set_desc
    
    FTAG2_DELAY_SEC
    FTAG2_TIMEOUT_SEC
    LOG_DIR
  end
  
  
  methods(Static)
    function target = preprocessTarget(curr_target, prev_target)
      % Inherit fields from previous target
      target = curr_target;
      for field_struct = {'tag_source', 'tag_width_m', ...
          'tag_tx_m', 'tag_ty_m', 'tag_tz_m', ...
          'tag_rx_deg', 'tag_ry_deg', 'tag_rz_deg'},
        field = field_struct{1};
        if ~isfield(target, field),
          if isempty(prev_target),
            error('SyntheticTestbenchNode:preprocessTarget', 'Missing field "%s"', field);
          else
            target.(field) = prev_target.(field);
          end
        end
      end
      
      % Decode type and payload from filename
      [target.tag_type_id, target.tag_num_slices, target.tag_num_freqs, ...
         target.tag_bit_pattern, target.tag_phases] = ...
         parseTagTypeAndPhases(target.tag_source);
       target.tag_phases_vec = reshape(target.tag_phases', 1, numel(target.tag_phases));
       
      target.ftag2_frame_id = -2; % Indicate that TagPose request had not been sent out
      target.ftag2_num_tags_detected = -1;
    end
    
    
    function node = runExampleNode(tag_source_filename)
      % Define a cell vector of tag (source+pose) targets
      target_seq = cell(1);

      % The very first target needs to fully specify source filename
      % (with proper format), as well as desired 3D pose
      target.tag_source = tag_source_filename;
      target.tag_width_m = 0.125;
      target.tag_tx_m = 0.1;
      target.tag_ty_m = -0.2;
      target.tag_tz_m = 1.0;
      target.tag_rx_deg = 0.0;
      target.tag_ry_deg = 0.0;
      target.tag_rz_deg = 0.0;
      target_seq{1} = target;
      
      % Subsequent targets only needs to specify incremental changes
      clear target;
      target.tag_tz_m = 1.2;
      target_seq{2} = target;
      target.tag_tz_m = 2.0;
      target_seq{3} = target;
      
      if true,
        rand_count = 1000;
        target.tag_source = tag_source_filename;
        target.tag_width_m = 0.125;
        while length(target_seq) < rand_count,
          target.tag_tx_m = (rand()*2-1)*0.5;
          target.tag_ty_m = (rand()*2-1)*0.5;
          target.tag_tz_m = 1.0 + (rand()-0.4);
          target.tag_rx_deg = (rand()*2-1)*60.0;
          target.tag_ry_deg = (rand()*2-1)*60.0;
          target.tag_rz_deg = (rand()*2-1)*60.0;
          target_seq{length(target_seq)+1} = target;
        end
      end
      
      % The constructor automatically starts processing the targets
      node = SyntheticTestbenchNode(target_seq);
    end
  end
  
  
  methods(Access='public')
    function obj=SyntheticTestbenchNode(target_seq, test_set_desc, ftag2_timeout_sec)
      % Parse arguments
      if nargin < 2,
        test_set_desc = '';
      end
      if nargin < 3,
        ftag2_timeout_sec = 1.0;
      end
      
      % Initialize internal variables
      obj.FTAG2_DELAY_SEC = 0.003; % Pausing for a bit to prevent ftag2 node from skipping images RIGHT after processing
      obj.FTAG2_TIMEOUT_SEC = ftag2_timeout_sec;
      obj.ftag2_timer = timer('ExecutionMode', 'singleShot', ...
        'StartDelay', obj.FTAG2_TIMEOUT_SEC, ...
        'TimerFcn', @handleTimeout, 'UserData', obj);
      obj.LOG_DIR = '/localdata/anqixu/ftag2/synth_testbench/';
      obj.test_set_desc = test_set_desc;
      obj.init_target_seq = target_seq;
      obj.ftag2_frame_id_offset = NaN;
      obj.progress_seq = target_seq;
      obj.progress_seq_i = -1;
      obj.prev_target = [];
      obj.curr_target = [];

      % Initialize ROS hooks
      %obj.roscore = rosmatlab.roscore(11311); % assume that roscore has been initiated externally
      obj.node = rosmatlab.node('synthetic_testbench_node');
      
      obj.update_tag_pose_pub = obj.node.addPublisher('/tag_renderer_node/set_tag_pose', 'tag_renderer/TagPose');
      obj.update_tag_pose_pub.Publisher.setLatchMode(0);
      obj.update_tag_source_pub = obj.node.addPublisher('/tag_renderer_node/set_tag_source', 'std_msgs/String');
      obj.update_tag_source_pub.Publisher.setLatchMode(0);
      obj.decoded_tags_sub = obj.node.addSubscriber('/ftag2/detected_tags', 'ftag2_core/TagDetections', 10);
      obj.decoded_tags_sub.addCustomMessageListener({@obj.handleTagsMsg, obj.node.Node});
      %obj.synth_img_sub = obj.node.addSubscriber('/camera/image_raw', 'sensor_msgs/Image', 10);
      %obj.synth_img_sub.addCustomMessageListener({@obj.handleSynthImgMsg, obj.node.Node});

      % Setup logging
      obj.log_file = sprintf('%s/STN_%s.mat', obj.LOG_DIR, datestr(now, 'yymmdd-HHMMSS-FFF'));
      if exist(obj.LOG_DIR, 'dir') ~= 7,
        obj.node.Node.getLog().warn(sprintf('! Logging disabled since LOG_DIR does not exist: %s', obj.LOG_DIR));
      else
        obj.saveLog();
      end

      obj.node.Node.getLog().info('. SyntheticTestbenchNode initialized');
      
      % Publish stub TagPose target to obtain msg and deduce ftag2_frame_id_offset
      obj.publishStubTarget();
    end

    
    function delete(obj)
      obj.shutdown();
    end

    
    function shutdown(obj) % shutdown node, but keep contents still
      stop(obj.ftag2_timer);
      obj.progress_seq_i = -1; % Disable processing
      if obj.decoded_tags_sub.isvalid,
        obj.node.removeSubscriber(obj.decoded_tags_sub);
      end
      if obj.update_tag_pose_pub.isvalid,
        obj.node.removePublisher(obj.update_tag_pose_pub);
      end
      if obj.update_tag_source_pub.isvalid,
        obj.node.removePublisher(obj.update_tag_source_pub);
      end
    end
    
    
    function reset(obj, target_seq, test_set_desc, ftag2_timeout_sec)
      % Stop processing incoming messages
      reset_ROS = false;
      if obj.progress_seq_i >= 0,
        reset_ROS = true;
        obj.shutdown();
      end
      
      % Reinitialize progress
      if nargin == 1,
        obj.progress_seq = obj.init_target_seq;
      else
        obj.progress_seq = target_seq;
      end
      obj.progress_seq_i = -1;
      obj.prev_target = [];
      obj.curr_target = [];
      obj.ftag2_frame_id_offset = NaN;
      if nargin > 2,
        obj.test_set_desc = test_set_desc;
      end
      if nargin > 3,
        obj.FTAG2_TIMEOUT_SEC = ftag2_timeout_sec;
      end
      
      % Reinitialize logging
      obj.log_file = sprintf('%s/STN_%s.mat', obj.LOG_DIR, datestr(now, 'yymmdd-HHMMSS-FFF'));
      obj.saveLog();
      
      % Re-initialize ROS hooks
      if reset_ROS,
        obj.update_tag_pose_pub = obj.node.addPublisher('/tag_renderer_node/set_tag_pose', 'tag_renderer/TagPose');
        obj.update_tag_pose_pub.Publisher.setLatchMode(0);
        obj.update_tag_source_pub = obj.node.addPublisher('/tag_renderer_node/set_tag_source', 'std_msgs/String');
        obj.update_tag_source_pub.Publisher.setLatchMode(0);
        obj.decoded_tags_sub = obj.node.addSubscriber('/ftag2/detected_tags', 'ftag2_core/TagDetections', 10);
        obj.decoded_tags_sub.addCustomMessageListener({@obj.handleTagsMsg, obj.node.Node});
        %obj.synth_img_sub = obj.node.addSubscriber('/camera/image_raw', 'sensor_msgs/Image', 10);
        %obj.synth_img_sub.addCustomMessageListener({@obj.handleSynthImgMsg, obj.node.Node});
      end
      
      % Publish stub TagPose target to obtain msg and deduce ftag2_frame_id_offset
      obj.publishStubTarget();
    end

    
    function handleTimeout(obj)
      if obj.progress_seq_i >= 0 && obj.progress_seq_i <= length(obj.progress_seq),
        error_msg = sprintf('Target %d/%d timed out (expected_id=%d)', ...
          obj.progress_seq_i, length(obj.progress_seq), obj.curr_target.ftag2_frame_id);
        obj.node.Node.getLog().error(error_msg);
        obj.shutdown();
        obj.progress_seq_i = -1;
        obj.ftag2_frame_id_offset = NaN;
        error('SyntheticTestbenchNode:TimedOut', error_msg); %#ok<SPERR>
      end
    end
    
  
    function waitTillIdle(obj, max_wait_sec)
      if obj.progress_seq_i < 0,
        return;
      else
        if nargin < 2,
          max_wait_sec = 60.0;
        end
        t_start = tic;
        while obj.progress_seq_i >= 0,
          t_elapsed = toc(t_start);
          if t_elapsed > max_wait_sec,
            return;
          end
          pause(0.001);
        end
      end
    end
  end
  
  
  methods(Access='private')
    function saveLog(obj)
      if exist('/localdata/anqixu/ftag2/synth_testbench', 'dir') == 7,
        progress_seq = obj.progress_seq; %#ok<*PROP,*NASGU>
        test_set_desc = obj.test_set_desc;
        save(obj.log_file, 'progress_seq', 'test_set_desc');
      end % else silently terminate without saving log
    end
    
    
    function publishStubTarget(obj)
      position_msg = obj.node.newMessage('geometry_msgs/Point');
      position_msg.setX(0);
      position_msg.setY(0);
      position_msg.setZ(0);
      orientation_msg = obj.node.newMessage('geometry_msgs/Quaternion');
      xyzw = tf_quaternion_from_euler(0, 0, 0);
      orientation_msg.setX(xyzw(1));
      orientation_msg.setY(xyzw(2));
      orientation_msg.setZ(xyzw(3));
      orientation_msg.setW(xyzw(4));
      pose_msg = obj.node.newMessage('geometry_msgs/Pose');
      pose_msg.setPosition(position_msg);
      pose_msg.setOrientation(orientation_msg);
      tag_pose_msg = obj.node.newMessage('tag_renderer/TagPose');
      tag_pose_msg.setPose(pose_msg);
      tag_pose_msg.setWidth(0.125);
      obj.update_tag_pose_pub.publish(tag_pose_msg);
      obj.node.Node.getLog().info('> Published stub TagPose request');
      obj.progress_seq_i = 0;
    end

    
    %function handleSynthImgMsg(~, ~, ~, ~) % obj, handle, event, node
    %  return;
    %end

    
    function publishTarget(obj)
      if obj.progress_seq_i < 0 || obj.progress_seq_i > length(obj.progress_seq),
        obj.node.Node.getLog().warn(sprintf('! Attempting to publish target %d/%d', obj.progress_seq_i, length(obj.progress_seq)));
        obj.progress_seq_i = -1;
        obj.ftag2_frame_id_offset = NaN;
        return;
      elseif obj.progress_seq_i == 0,
        error('Unexpected logic flow, publishTarget() called with progress_seq_i==0');
      else
        % Only publish tag source if different from prev_target
        if isempty(obj.prev_target) || ...
            ~strcmp(obj.prev_target.tag_source, obj.curr_target.tag_source),
          source_msg = obj.node.newMessage('std_msgs/String');
          source_msg.setData(obj.curr_target.tag_source);
          obj.update_tag_source_pub.publish(source_msg);
        end
        
        % Define TagPose message
        position_msg = obj.node.newMessage('geometry_msgs/Point');
        position_msg.setX(obj.curr_target.tag_tx_m);
        position_msg.setY(obj.curr_target.tag_ty_m);
        position_msg.setZ(obj.curr_target.tag_tz_m);
        orientation_msg = obj.node.newMessage('geometry_msgs/Quaternion');
        radians = pi/180.0;
        xyzw = tf_quaternion_from_euler( ...
          radians*obj.curr_target.tag_rx_deg, ...
          radians*obj.curr_target.tag_ry_deg, ...
          radians*obj.curr_target.tag_rz_deg);
        orientation_msg.setX(xyzw(1));
        orientation_msg.setY(xyzw(2));
        orientation_msg.setZ(xyzw(3));
        orientation_msg.setW(xyzw(4));
        pose_msg = obj.node.newMessage('geometry_msgs/Pose');
        pose_msg.setPosition(position_msg);
        pose_msg.setOrientation(orientation_msg);
        tag_pose_msg = obj.node.newMessage('tag_renderer/TagPose');
        tag_pose_msg.setPose(pose_msg);
        tag_pose_msg.setWidth(obj.curr_target.tag_width_m);
        
        % Publish message
        obj.curr_target.ftag2_frame_id = obj.ftag2_frame_id_offset + obj.progress_seq_i; % Indicate waiting to receive decoded ftag2 message
        obj.update_tag_pose_pub.publish(tag_pose_msg);
        obj.node.Node.getLog().info(sprintf('> Published TagPose request %d/%d (expected_id=%d)', obj.progress_seq_i, length(obj.progress_seq), obj.ftag2_frame_id_offset+obj.progress_seq_i));
        
        % Start timer
        stop(obj.ftag2_timer);
        start(obj.ftag2_timer);
      end
    end

    
    function handleTagsMsg(obj, ~, event, ~) % obj, handle, event, node
      msg = event.JavaEvent.getSource();
      frameID = msg.getFrameID();
      
      % Log ID offset if previously sent stub TagPose target
      if obj.progress_seq_i == 0 && isnan(obj.ftag2_frame_id_offset),
        obj.ftag2_frame_id_offset = frameID;
        obj.node.Node.getLog().info(sprintf('. Received stub ftag2 msg (offset=%d)', obj.ftag2_frame_id_offset));
        
        % Start publishing first target
        obj.progress_seq_i = 1;
        obj.curr_target = SyntheticTestbenchNode.preprocessTarget(obj.progress_seq{obj.progress_seq_i}, obj.prev_target);
        pause(obj.FTAG2_DELAY_SEC);
        obj.publishTarget();
        return;
      
      % Detect out-of-sequence events
      elseif obj.progress_seq_i <= 0,
        %obj.node.Node.getLog().warn(sprintf('! Received ftag2 msg while idle (recv_id=%d)', frameID));
        return;

      elseif ~(obj.curr_target.ftag2_frame_id > 0),
        obj.node.Node.getLog().warn(sprintf('! Received spurious ftag2 msg (expect_id=%d, recv_id=%d)', obj.curr_target.ftag2_frame_id, frameID));
        return;
      
      elseif obj.curr_target.ftag2_frame_id ~= frameID,
        obj.node.Node.getLog().warn(sprintf('! Received unexpected ftag2 msg (expect_id=%d, recv_id=%d)', obj.curr_target.ftag2_frame_id, frameID));
        return;
      end
      
      % Extract first tag
      stop(obj.ftag2_timer);
      tagsMsg = msg.getTags();
      obj.curr_target.ftag2_num_tags_detected = tagsMsg.size();
      obj.node.Node.getLog().info(sprintf('. Received ftag2 msg (id=%d) with %d tags', frameID, tagsMsg.size()));
      if tagsMsg.size() >= 1,
        tagMsg = tagsMsg.get(0);
        poseMsg = tagMsg.getPose();
        positionMsg = poseMsg.getPosition();
        orientationMsg = poseMsg.getOrientation();
        
        % Parse tag pose and payload
        obj.curr_target.ftag2_width_px = tagMsg.getMarkerPixelWidth();
        obj.curr_target.ftag2_tx_m = positionMsg.getX();
        obj.curr_target.ftag2_ty_m = positionMsg.getY();
        obj.curr_target.ftag2_tz_m = positionMsg.getZ();
        xyzw = [orientationMsg.getX(), orientationMsg.getY(), ...
          orientationMsg.getZ(), orientationMsg.getW()];
        rot_xyz_rad = tf_euler_from_quaternion(xyzw);
        degrees = 180.0/pi;
        obj.curr_target.ftag2_rx_deg = rot_xyz_rad(1)*degrees;
        obj.curr_target.ftag2_ry_deg = rot_xyz_rad(2)*degrees;
        obj.curr_target.ftag2_rz_deg = rot_xyz_rad(3)*degrees;
        obj.curr_target.ftag2_tag_img_rot = tagMsg.getTagImgRot();
        obj.curr_target.ftag2_mags_vec = tagMsg.getMags()';
        obj.curr_target.ftag2_phases_vec = tagMsg.getPhases()';
        obj.curr_target.ftag2_payload_str = char(tagMsg.getDecodedPayloadStr());
      end
        
      % Save curr_target
      obj.progress_seq{obj.progress_seq_i} = obj.curr_target;
      obj.saveLog();
        
      % Increment progress
      if obj.progress_seq_i <= 0 || obj.progress_seq_i >= length(obj.progress_seq),
        obj.node.Node.getLog().info(sprintf('v SyntheticTestbenchNode finished processing %d targets', length(obj.progress_seq)));
        obj.progress_seq_i = -1;
        obj.ftag2_frame_id_offset = NaN;
        return;
      else
        obj.progress_seq_i = obj.progress_seq_i + 1;
        obj.prev_target = obj.curr_target;
        obj.curr_target = SyntheticTestbenchNode.preprocessTarget(obj.progress_seq{obj.progress_seq_i}, obj.prev_target);
        pause(obj.FTAG2_DELAY_SEC);
        obj.publishTarget();
      end
    end
    
  end
end


function handleTimeout(timer_obj, ~) % event
node_obj = timer_obj.UserData;
node_obj.handleTimeout();
end
