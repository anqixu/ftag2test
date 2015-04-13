% TEST_ROS_MATLAB.m

%clear;
%INIT_WORKSPACE;

%setenv('ROS_MASTER_URI', 'http://127.0.0.1:11311');
%setenv('ROS_IP', '127.0.0.1');

% Assume that roscore has been initiated externally
%roscore = rosmatlab.roscore(11311);

node = rosmatlab.node('rosmatlab_test_node');
publisher = node.addPublisher('/test', 'ftag2_core/TagDetection');
subscriber = node.addSubscriber('/test','ftag2_core/TagDetection',10);
subscriber.addCustomMessageListener({ ...
  @(h,e,n) n.getLog().info(['TagDetection received:',num2str(e.JavaEvent.getSource.getMarkerPixelWidth())]), ...
  node.Node});

msg = node.newMessage('ftag2_core/TagDetection');
for i = 1:10
    msg.setMarkerPixelWidth(rand(1))
    publisher.publish(msg);
    fprintf('Published msg on %s.\n', publisher.TopicName);
    pause(1.0);
end

clear('node');
