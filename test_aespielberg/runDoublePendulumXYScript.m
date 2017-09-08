megaclear;

use_l1_norm = false; %if false, use normal regularizer
add_obstacles = 0; %0, 1, 2
use_position_diff_constraint = true; %if false, use xf as final constraint
cost_function = 'torque'; %torque, torquetime, none
%cost_function = '';


n = 15;
for i = 1:1:n
    file_name = runDoublePendulumXY(use_l1_norm,add_obstacles,use_position_diff_constraint,cost_function,'results/DoublePendulum',i);
end
load(file_name);
xtraj = xtraj.setOutputFrame(xtraj_outputFrame);
v2 = p.constructVisualizer();
v2.playback(xtraj);
p.getParams().double
F
utraj_cost

disp('you"re looking for v2.playback(xtraj);')
