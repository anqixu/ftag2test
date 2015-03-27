classdef PhaseAnalysisNode < handle
  properties (Constant)
  end
  
  
  properties (SetAccess='protected')
    node
    tags_sub
  end
  
    
  properties
    tag_type
    
    num_slices
    num_freqs
    bit_pattern
    
    gt_payload_str
    gt_phases
    gt_phases_vec % order: s1f1 s1f2 s1f3 s2f1 ...
    
    recv_discr_vecs % order: s1f1 s1f2 s1f3 s2f1 ...
    
    last_display_time

    HIST_MARGIN_RATIO;
    
    SHOW_ALL_SUBPLOTS;
    SINGLE_SUBPLOT_SLICE;
    SINGLE_SUBPLOT_FREQ;
    
    PLOT_REFRESH_MIN_SEC;
  end
  
  
  methods(Static)
  end
  
  
  methods(Access='public')
    function obj=PhaseAnalysisNode(tag_type, payload_str)
      % Set global constants
      obj.HIST_MARGIN_RATIO = 0.1;
      obj.SHOW_ALL_SUBPLOTS = false;
      obj.SINGLE_SUBPLOT_SLICE = 1;
      obj.SINGLE_SUBPLOT_FREQ = 2;  
      obj.PLOT_REFRESH_MIN_SEC = 0.5;
      
      % Initialize internal variables
      obj.recv_discr_vecs = [];
      
      % Parse arguments
      obj.parseGroundTruth(tag_type, payload_str);

      % Create/focus on figure
      if isempty(findobj('type', 'figure', 'name', 'FTag2 Phase Analysis')),
        figure('Name', 'FTag2 Phase Analysis');
      end
      obj.last_display_time = [];

      % Initialize ROS hooks
      %obj.roscore = rosmatlab.roscore(11311); % assume that roscore has been initiated externally
      obj.node = rosmatlab.node('phase_analysis_node');
      
      obj.tags_sub = obj.node.addSubscriber('/ftag2/detected_tags', 'ftag2_core/TagDetections', 10);
      obj.tags_sub.addCustomMessageListener({@obj.handleTagsMsg, obj.node.Node});
      
      obj.node.Node.getLog().info('PhaseAnalysisNode initialized');
      
      tic;
    end

    
    function delete(obj)
      obj.shutdown();
    end

    
    function shutdown(obj) % shutdown node, but keep contents still
      if obj.tags_sub.isvalid,
        obj.node.removeSubscriber(obj.tags_sub);
      end
    end
    
    
    function reset(obj, new_tag_type, new_payload_str)
      % Stop processing incoming messages
      obj.shutdown();
      
      % Re-parse arguments
      if nargin == 2,
        obj.parseGroundTruth(new_tag_type);
      elseif nargin == 3,
        obj.parseGroundTruth(new_tag_type, new_payload_str);
      end
      
      % Clear received contents
      obj.recv_discr_vecs = [];
      
      % Reset/re-create figure
      figID = findobj('type', 'figure', 'name', 'FTag2 Phase Analysis');
      if isempty(figID),
        figure('Name', 'FTag2 Phase Analysis');
      end
      clf;
      obj.last_display_time = [];
      
      % Re-initialize ROS hooks
      obj.tags_sub = obj.node.addSubscriber('/ftag2/detected_tags', 'ftag2_core/TagDetections', 10);
      obj.tags_sub.addCustomMessageListener({@obj.handleTagsMsg, obj.node.Node});
    end
    
    
    function parseGroundTruth(obj, tag_type, payload_str)
      % Parse tag type
      if nargin < 3, % argument is a tag filename
        payload_str = tag_type;
        tag_type = regexpi(tag_type, '[0-9]s[0-9]f[0-9]+b', 'match');
        if isempty(tag_type),
          error('PhaseAnalysisNode:TagType', 'Unrecognized tag_type in filename: %s', tag_type);
        elseif iscell(tag_type) && length(tag_type) ~= 1,
          error('PhaseAnalysisNode:TagType', 'Multiple tag_type found in filename: %s', tag_type);
        else
          tag_type = tag_type{1};
        end
      end
      
      if strcmpi(tag_type, '6S2F21B'),
        obj.tag_type = 6221;
      elseif strcmpi(tag_type, '6S2F22B'),
        obj.tag_type = 6222;
      elseif strcmpi(tag_type, '6S3F211B'),
        obj.tag_type = 63211;
      elseif strcmpi(tag_type, '6S5F3B'),
        obj.tag_type = 653;
      elseif strcmpi(tag_type, '6S5F33322B'),
        obj.tag_type = 6533322;
      else
        error('PhaseAnalysisNode:TagType', 'Unrecognized tag_type, expecting format #S#F#<...>B: %s', tag_type);
      end
      obj.num_slices = floor(obj.tag_type/10^floor(log10(obj.tag_type)));
      obj.num_freqs = mod(floor(obj.tag_type/10^floor(log10(obj.tag_type)-1)), 10);
      bit_pattern_id = mod(obj.tag_type, 10^(floor(log10(obj.tag_type))-1));
      if bit_pattern_id < 10 && obj.num_freqs > 1,
        obj.bit_pattern = ones(1, obj.num_freqs)*bit_pattern_id;
      else
        obj.bit_pattern = arrayfun(@(c) str2double(c), mat2str(bit_pattern_id));
      end
      
      % Parse payload string
      payload_expr = '';
      for s_i = 1:obj.num_slices,
        for f_j = 1:obj.num_freqs,
          payload_expr = strcat(payload_expr, '[0-9]');
        end
        if s_i < obj.num_slices,
          payload_expr = strcat(payload_expr, '_');
        end
      end
      obj.gt_payload_str = regexpi(payload_str, payload_expr, 'match');
      if isempty(obj.gt_payload_str),
        error('PhaseAnalysisNode:PayloadStr', 'Unrecognized payload_str, expecting format ##_##_...: %s', payload_str);
      elseif iscell(obj.gt_payload_str) && length(obj.gt_payload_str) ~= 1,
        error('PhaseAnalysisNode:PayloadStr', 'Multiple payload_str found: %s', payload_str);
      else
        obj.gt_payload_str = obj.gt_payload_str{1};
      end
      bit_chunks_str = strrep(obj.gt_payload_str, '_', '');
      bit_chunks_vec = arrayfun(@(c) str2double(c), bit_chunks_str);
      bit_chunks = reshape(bit_chunks_vec, obj.num_freqs, obj.num_slices)';
      obj.gt_phases = bit_chunks .* repmat(360./(2.^obj.bit_pattern), obj.num_slices, 1);
      obj.gt_phases_vec = reshape(obj.gt_phases', 1, obj.num_freqs*obj.num_slices);
      
      if ~obj.SHOW_ALL_SUBPLOTS,
        if obj.SINGLE_SUBPLOT_SLICE > obj.num_slices,
          error('PhaseAnalysisNode:BadSetting', 'Requested subplot slice %d exceeds num_slices %d', ...
            obj.SINGLE_SUBPLOT_SLICE, obj.num_slices);
        end
        if obj.SINGLE_SUBPLOT_FREQ > obj.num_freqs,
          error('PhaseAnalysisNode:BadSetting', 'Requested subplot freq %d exceeds num_freqs %d', ...
            obj.SINGLE_SUBPLOT_FREQ, obj.num_freqs);
        end
      end
    end
    
    
    function handleTagsMsg(obj, ~, event, ~) % obj, handle, event, node
      % Extract first/matching tag
      msg = event.JavaEvent.getSource();
      %frameID = msg.getFrameID();
      tagsMsg = msg.getTags();
      recvTagMsg = [];
      for i = 1:tagsMsg.size(),
        tagMsg = tagsMsg.get(i-1);
        if isempty(recvTagMsg),
          recvTagMsg = tagMsg;
        elseif strcmp(tagMsg.getDecodedPayloadStr(), obj.gt_payload_str),
          recvTagMsg = tagMsg;
        end
      end
      recv_phases_vec = [];
      if ~isempty(recvTagMsg),
        recv_phases_vec = recvTagMsg.getPhases()';
      end

      % Store received phase discrepancy and refresh display
      if numel(recv_phases_vec) == obj.num_freqs*obj.num_slices,
        recv_discr_vec = angularDiff(recv_phases_vec, obj.gt_phases_vec, 360.0);
        obj.recv_discr_vecs = [obj.recv_discr_vecs; recv_discr_vec];
        obj.refreshFig(msg.getFrameID(), tagsMsg.size(), true);
      elseif ~isempty(recv_phases_vec),
        warning('PhaseAnalysisNode:MsgCallback', ...
          'Received msg with %d phases, expecting %d x %d', ...
          numel(recv_phases_vec), obj.num_slices, obj.num_freqs);
        % NOTE: disabled to reduce refresh lag
        %obj.refreshFig(msg.getFrameID(), tagsMsg.size(), false);
      else
        % NOTE: disabled to reduce refresh lag
        %obj.refreshFig(msg.getFrameID(), tagsMsg.size(), false);
      end
    end
    
    function refreshFig(obj, recvFrameID, recvNumTags, hasTargetTag)
      % Rate-limit refresh (since plotting takes time)
      now = toc;
      if ~isempty(obj.last_display_time) && ...
          (now - obj.last_display_time) < obj.PLOT_REFRESH_MIN_SEC, % do not refresh too fast
        return;
      else
        obj.last_display_time = now;
      end
      
      % Initialize figure
      figID = findobj('type', 'figure', 'name', 'FTag2 Phase Analysis');
      if isempty(figID),
        %error('Could not locate figure');
        obj.shutdown();
        return;
      end
      figure(figID);
      clf;

      if obj.SHOW_ALL_SUBPLOTS,
        subplot_k = 1;
        for s_i = 1:obj.num_slices,
          for f_j = 1:obj.num_freqs,
            discr_entries = obj.recv_discr_vecs(:, subplot_k);

            subplot(obj.num_slices, obj.num_freqs, subplot_k);
            hold on;

            hist(discr_entries, min(20, floor(length(discr_entries)/2)));
            bar_h = findobj(gca, 'Type', 'patch');
            set(bar_h, 'FaceColor', 'g');

            discr_mag = max(abs(discr_entries));
            ax = axis();
            ax(1) = -discr_mag*(1+obj.HIST_MARGIN_RATIO);
            ax(2) = discr_mag*(1+obj.HIST_MARGIN_RATIO);
            ax_height = ax(4)-ax(3);
            ax(3) = ax(3) - ax_height*obj.HIST_MARGIN_RATIO;
            ax(4) = ax(4) + ax_height*obj.HIST_MARGIN_RATIO;
            axis(ax);

            plot(discr_entries, (-ax_height*obj.HIST_MARGIN_RATIO/2).*ones(size(discr_entries)), ...
              'bx', 'MarkerSize', 10, 'LineWidth', 1);

            if hasTargetTag,
              plot(discr_entries(end), (-ax_height*obj.HIST_MARGIN_RATIO/2), ...
                'rx', 'MarkerSize', 12, 'LineWidth', 2);
            end

            hold off;

            if f_j == 1,
              ylabel(sprintf('s=%d', s_i));
            end
            if s_i == obj.num_slices,
              xlabel(sprintf('f=%d', f_j));
            end
            if subplot_k == 1,
              title(sprintf('Frame %d: %d tags (%d recv)', ...
                recvFrameID, recvNumTags, size(obj.recv_discr_vecs, 1)));
            end

            subplot_k = subplot_k + 1;
          end
        end
      else % ~obj.SHOW_ALL_SUBPLOTS
        s_i = obj.SINGLE_SUBPLOT_SLICE;
        f_j = obj.SINGLE_SUBPLOT_FREQ;
        entry_k = 1 + (s_i-1)*obj.num_slices + f_j;
        
        discr_entries = obj.recv_discr_vecs(:, entry_k);
        
        clf;
        hold on;
        
        hist(discr_entries, min(20, floor(length(discr_entries)/2)));
        bar_h = findobj(gca, 'Type', 'patch');
        set(bar_h, 'FaceColor', 'g');
        
        discr_mag = max(abs(discr_entries));
        ax = axis();
        ax(1) = -discr_mag*(1+obj.HIST_MARGIN_RATIO);
        ax(2) = discr_mag*(1+obj.HIST_MARGIN_RATIO);
        ax_height = ax(4)-ax(3);
        ax(3) = ax(3) - ax_height*obj.HIST_MARGIN_RATIO;
        ax(4) = ax(4) + ax_height*obj.HIST_MARGIN_RATIO;
        axis(ax);
        
        plot(discr_entries, (-ax_height*obj.HIST_MARGIN_RATIO/2).*ones(size(discr_entries)), ...
          'bx', 'MarkerSize', 10, 'LineWidth', 1);
        
        if hasTargetTag,
          plot(discr_entries(end), (-ax_height*obj.HIST_MARGIN_RATIO/2), ...
            'rx', 'MarkerSize', 12, 'LineWidth', 2);
        end
        
        hold off;
        
        ylabel(sprintf('s=%d', s_i));
        xlabel(sprintf('f=%d', f_j));
        title(sprintf('Frame %d: %d tags (%d recv)', ...
          recvFrameID, recvNumTags, size(obj.recv_discr_vecs, 1)));
      end
    end
  end
end
