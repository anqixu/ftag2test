% SET FOLDERS AND COMMON VARIABLES

% Fix issue possibly introduced by drake
restoredefaultpath;

% Add ROS-MATLAB support
if true,
  prevpath = pwd;
  rosmatlabpath = sprintf('%s/toolbox/psp/rosmatlab/', matlabroot);
  cd(rosmatlabpath);
  rosmatlab_AddClassPath;
  addpath(rosmatlabpath);
  cd(prevpath);
end

% Add all subfolders of current folder to the top of the search path
%addpath(genpath('..'));
