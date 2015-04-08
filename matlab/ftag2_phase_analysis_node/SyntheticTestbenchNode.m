classdef SyntheticTestbenchNode < handle
  properties (Constant)
  end
  
  
  properties (SetAccess='protected')
    node
    decoded_tags_sub
    update_tag_pose_pub
    update_tag_source_pub
    ftag2_timer
  end
  
    
  properties
    progress_seq
    progress_seq_i
    curr_target
    prev_target
    
    log_file
    
    FTAG2_TIMEOUT_SEC
    LOG_DIR
  end
  
  
  methods(Static)
    function target = preprocessTarget(curr_target, prev_target)
      % Inherit fields from previous target
      target = curr_target;
      for field_struct = {'tag_source', 'tag_width_m', ...
          'tag_x_m', 'tag_y_m', 'tag_z_m', ...
          'tag_roll_deg', 'tag_pitch_deg', 'tag_yaw_deg'},
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
         target.tag_bit_pattern, target.tag_phases_vec] = ...
         parseTagTypeAndPhases(target.tag_source);
    end
    
    
    function node = runExampleNode(tag_source_filename)
      % Define a cell vector of tag (source+pose) targets
      target_seq = cell(2, 1);

      % The very first target needs to fully specify source filename
      % (with proper format), as well as desired 3D pose
      target.tag_source = tag_source_filename;
      target.tag_width_m = 0.125;
      target.tag_x_m = 0.0;
      target.tag_y_m = 0.0;
      target.tag_z_m = 1.0;
      target.tag_roll_deg = 0.0;
      target.tag_pitch_deg = 0.0;
      target.tag_yaw_deg = 0.0;
      target_seq{1} = target;
      
      % Subsequent targets only needs to specify incremental changes
      clear target;
      target.tag_z_m = 1.2;
      target_seq{2} = target;
      
      % The constructor automatically starts processing the targets
      node = SyntheticTestbenchNode(target_seq);
    end
  end
  
  
  methods(Access='public')
    function obj=SyntheticTestbenchNode(target_seq, ftag2_timeout_sec)
      % Parse arguments
      if nargin < 2,
        ftag2_timeout_sec = 1.0;
      end
      
      % Initialize internal variables
      obj.LOG_DIR = '/localdata/anqixu/ftag2/synth_testbench/';
      obj.FTAG2_TIMEOUT_SEC = ftag2_timeout_sec;
      obj.progress_seq = target_seq;
      obj.progress_seq_i = 1;
      obj.prev_target = [];
      obj.curr_target = SyntheticTestbenchNode.preprocessTarget(obj.progress_seq{obj.progress_seq_i}, obj.prev_target);
      obj.ftag2_timer = timer('ExecutionMode', 'singleShot', ...
        'StartDelay', obj.FTAG2_TIMEOUT_SEC, ...
        'TimerFcn', @handleTimeout, 'UserData', obj);

      % Setup logging
      obj.log_file = sprintf('%s/STN_%s.mat', obj.LOG_DIR, datestr(now, 'yymmdd-HHMMSS-FFF'));
      progress_seq = obj.progress_seq; %#ok<*PROP,*NASGU>
      save(obj.log_file, 'progress_seq');
      
      % Initialize ROS hooks
      %obj.roscore = rosmatlab.roscore(11311); % assume that roscore has been initiated externally
      obj.node = rosmatlab.node('synthetic_testbench_node');
      
      obj.update_tag_pose_pub = obj.node.addPublisher('/tag_renderer/tag_pose', 'tag_renderer/TagPose.msg');
      obj.update_tag_source_pub = obj.node.addPublisher('/tag_renderer/tag_source', 'std_msgs/String.msg');
      obj.decoded_tags_sub = obj.node.addSubscriber('/ftag2/detected_tags', 'ftag2_core/TagDetections', 10);
      obj.decoded_tags_sub.addCustomMessageListener({@obj.handleTagsMsg, obj.node.Node});
      
      obj.node.Node.getLog().info('SyntheticTestbenchNode initialized');
      
      % Start processing targets
      obj.publishTarget();
    end

    
    function delete(obj)
      obj.shutdown();
    end

    
    function shutdown(obj) % shutdown node, but keep contents still
      stop(obj.ftag2_timer);
      obj.progress_seq_i = 0; % Disable processing
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
    
    
    function reset(obj, target_seq, ftag2_timeout_sec)
      % Stop processing incoming messages
      obj.shutdown();
      
      % Re-parse arguments
      obj.progress_seq = target_seq;
      obj.progress_seq_i = 1;
      obj.prev_target = [];
      obj.curr_target = SyntheticTestbenchNode.preprocessTarget(obj.progress_seq{obj.progress_seq_i});
      if nargin > 2,
        obj.FTAG2_TIMEOUT_SEC = ftag2_timeout_sec;
      end
      
      % Re-initialize ROS hooks
      obj.update_tag_pose_pub = obj.node.addPublisher('/tag_renderer/tag_pose', 'tag_renderer/TagPose.msg');
      obj.update_tag_source_pub = obj.node.addPublisher('/tag_renderer/tag_source', 'std_msgs/String.msg');
      obj.decoded_tags_sub = obj.node.addSubscriber('/ftag2/detected_tags', 'ftag2_core/TagDetections', 10);
      obj.decoded_tags_sub.addCustomMessageListener({@obj.handleTagsMsg, obj.node.Node});

        % Start processing targets
        obj.publishTarget();
      end

    
    function publishTarget(obj)
      if obj.progress_seq_i <= 0 || obj.progress_seq_i > length(obj.progress_seq),
        obj.node.Node.getLog().info(sprintf('SyntheticTestbenchNode finished processing %d targets', length(obj.progress_seq)));
        obj.progress_seq_i = 0;
        return;
      else
        % Only publish tag source if different from prev_target
        if isempty(obj.prev_target) || ...
            ~strcmp(obj.prev_target.tag_source, obj.curr_target.tag_source),
          source_msg = obj.node.newMessage(std_msgs/String');
          source_msg.setData(obj.curr_target.tag_source);
          obj.update_tag_source_pub.publish(source_msg);
        end
        
        % Define TagPose message
        position_msg = obj.node.newMessage('geometry_msgs/Point');
        position_msg.setX(obj.curr_target.tag_x_m);
        position_msg.setY(obj.curr_target.tag_y_m);
        position_msg.setZ(obj.curr_target.tag_z_m);
        orientation_msg = obj.node.newMessage('geometry_msgs/Quaternion');
        radians = pi/180.0;
        xyzw = tf_quaternion_from_euler( ...
          radians*obj.curr_target.tag_roll_deg, ...
          radians*obj.curr_target.tag_pitch_deg, ...
          radians*obj.curr_target.tag_yaw_deg);
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
        obj.update_tag_pose_pub.publish(tag_pose_msg);
        
        % Start timer
        stop(obj.ftag2_timer);
        start(obj.ftag2_timer);
      end
    end
    
    
    function handleTagsMsg(obj, ~, event, ~) % obj, handle, event, node
      % Extract first tag
      msg = event.JavaEvent.getSource();
      %frameID = msg.getFrameID();
      tagsMsg = msg.getTags();
      if tagsMsg.size() >= 1,
        if tagsMsg.size() > 1,
          obj.node.Node.getLog().warn('Received ftag2 msg with %d tags', tagsMsg.size());
        end
        tagMsg = tagsMsg.get(0);
        poseMsg = tagMsg.getPose();
        positionMsg = poseMsg.getPosition();
        orientationMsg = poseMsg.getOrientation();
        
        % Parse tag pose and payload
        obj.curr_target.ftag2_x_m = positionMsg.getX();
        obj.curr_target.ftag2_y_m = positionMsg.getY();
        obj.curr_target.ftag2_z_m = positionMsg.getZ();
        xyzw = [orientationMsg.getX(), orientationMsg.getY(), ...
          orientationMsg.getZ(), orientationMsg.getW()];
        [obj.curr_target.ftag2_roll_deg, ...
          obj.curr_target.ftag2_pitch_deg, ...
          obj.curr_target.ftag2_yaw_deg] = tf_euler_from_quaternion(xyzw);
        obj.curr_target.ftag2_width_px = tagMsg.getMarkerPixelWidth();
        obj.curr_target.ftag2_tag_img_rot = tagMsg.getTagImgRot();
        obj.curr_target.ftag2_mags_vec = tagMsg.getMags()';
        obj.curr_target.ftag2_phases_vec = tagMsg.getPhases()';
        obj.curr_target.ftag2_payload_str = tagMsg.getDecodedPayloadStr();
        
        % Save curr_target
        if obj.progress_seq_i <= 0, % Terminate immediately if requested elsewhere
          return;
        end
        obj.progress_seq{obj.progress_seq_i} = obj.curr_target;
        progress_seq = obj.progress_seq;
        save(obj.log_file, 'progress_seq');
        
        % Increment progress
        if obj.progress_seq_i <= 0 || obj.progress_seq_i >= length(obj.progress_seq),
          obj.node.Node.getLog().info(sprintf('SyntheticTestbenchNode finished processing %d targets', length(obj.progress_seq)));
          obj.progress_seq_i = 0;
          return;
        else
          obj.progress_seq_i = obj.progress_seq_i + 1;
          obj.prev_target = obj.curr_target;
          obj.curr_target = SyntheticTestbenchNode.preprocessTarget(obj.progress_seq{obj.progress_seq_i}, obj.prev_target);
          obj.publishTarget();
        end
      else
        obj.node.Node.getLog().warn('Received ftag2 msg with 0 tags');
      end
    end
    
    
    function handleTimeout(obj)
      obj.node.Node.getLog().error( ...
        sprintf('Target %d/%d timed out', ...
        obj.progress_seq_i, length(obj.progress_seq)));
      obj.shutdown();
      obj.progress_seq_i = 0;
    end
  end
end


function handleTimeout(timer_obj, ~) % event
timer_obj.UserData.handleTimeout();
end
