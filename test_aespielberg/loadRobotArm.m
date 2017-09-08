% script to make loading the robot arm and playing back its trajectory
% as easy as typing 'loadRobotArm'

load('robotArmTest1.mat');
xtraj = xtraj.setOutputFrame(xtraj_outputFrame);
v2 = p.constructVisualizer();
v2.playback(xtraj);
p.getParams().double
F
utraj_cost