classdef PhaseAnalysisNode < handle
  properties (Constant)
  end
  
  
  properties (SetAccess='protected')
    node
    
    tags_sub
  end
  
  
  
  properties
    gt_payload_string
    gt_phases
    
    recv_phase_discrepancies
  end
  
  
  
  methods(Static)
  end
  
  
  methods(Access='public')
    function obj=PhaseAnalysisNode(payload_string)
      obj.gt_payload_string = payload_string;
      
      % TODO: Parse ground truth phases
      obj.gt_phases = zeros(6, 2);
      
      % Initialize ROS hooks
      %obj.roscore = rosmatlab.roscore(11311); % assume that roscore has been initiated externally
      obj.node = rosmatlab.node('phase_analysis_node');
      
      obj.tags_sub = obj.node.addSubscriber('/ftag2/detected_tags', 'ftag2_core/TagDetections', 10);
      obj.tags_sub.addCustomMessageListener({@obj.handleTagsMsg, obj.node.Node});
      
      figure('Name', 'FTag2 Phase Analysis');
      
      obj.node.Node.getLog().info('PhaseAnalysisNode initialized');
      
      % TODO: add timeout timer
    end
    
    
    function handleTagsMsg(obj, ~, event, ~) % obj, handle, event, node
      figID = 1; % TODO: if figure does not exist (e.g. is closed) then kill node
    
      msg = event.JavaEvent.getSource();
      frameID = msg.getFrameID();
      
      tagsMsg = msg.getTags();
      % TODO: find correct tag in list of tags, then extract phases
      tag_phases = [];
      for i = 1:length(tagsMsg),
        % tagsMsg(i).getPose()
        % tagsMsg(i).getMarkerPixelWidth()
        % tagsMsg(i).getIDString()
        % tagsMsg(i).getMags()
        % tagsMsg(i).getPhases()
        % tagsMsg(i).getPhaseVariances()
        % tagsMsg(i).getBitChunksStr()
      end

      if all(size(tag_phases) == size(obj.gt_phases)),
        tag_discrepancy = tag_phases - obj.gt_phases;
        obj.recv_phase_discrepancies(size(obj.recv_phase_discrepancies, 1)+1, :) = reshape(tag_discrepancy, 1, numel(tag_discrepancy));
        
        figure(figID);
        clf;
        
        subplot_k = 1;
        for s_i = 1:size(obj.gt_phases, 1),
          for f_j = 1:size(obj.gt_phases, 2),
            dphase_data = obj.recv_phase_discrepancies(:, subplot_k);
          
            subplot(size(obj.gt_phases, 1), size(obj.gt_phases, 2), subplot_k);
            hold on;
            hist(dphase_data, min(20, floor(length(dphase_data)/2)));
            plot(zeros(size(dphase_data)), dphase_data, 'kx', 'MarkerSize', 10);
            hold off;
            
            subplot_k = subplot_k + 1;
            
            if f_j == 1,
              ylabel(sprintf('Slice %d', s_i));
            end
            if s_i == 1,
              title(sprintf('Freq %d', f_j));
            end
          end
        end
      end
    end
  end
end
