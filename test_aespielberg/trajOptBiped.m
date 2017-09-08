function trajOptBiped(num_runs, options)


if vargin < 1
    num_runs = 1;
    visualize = true;
    
end
if vargin < 2
   visualize = false;
else
    visualize = options.visualize;
   
end


megaclear

ant = 1;


tic
try
    xtraj = options.xtraj;
    utraj = options.utraj;
    ltraj = options.ltraj;
    ljltraj = options.ljltraj;
    [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = bipedTrajOpt(xtraj, utraj, ltraj, ljltraj, 0.001);
catch
    [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = bipedTrajOpt;
end

toc


if ~isempty(traj_opt.param_inds)
    p = p.setParams(z(traj_opt.param_inds, 1));
end

if visualize
    v = p.constructVisualizer; v.playback(xtraj);
end


end





% [p,xtraj2,utraj2,ltraj2,ljltraj2,z2,F2,info2,traj_opt2] = antTrajOpt(xtraj,utraj,ltraj,ljltraj,.1);
% v = p.constructVisualizer; v.playback(xtraj2);
% display('iter 2')
% %%
% [p,xtraj3,utraj3,ltraj3,ljltraj3,z3,F3,info3,traj_opt3] = antTrajOpt(xtraj2,utraj2,ltraj2,ljltraj2,.01);
% v = p.constructVisualizer; v.playback(xtraj3);
% display('iter 3')
% %%
% [p,xtraj4,utraj4,ltraj4,ljltraj4,z4,F4,info4,traj_opt4] = antTrajOpt(xtraj3,utraj3,ltraj3,ljltraj3,1e-3);
% v = p.constructVisualizer; v.playback(xtraj4);
% display('iter 4')
% %%
% [p,xtraj5,utraj5,ltraj5,ljltraj5,z5,F5,info5,traj_opt5] = antTrajOpt(xtraj4,utraj4,ltraj4,ljltraj4,0);
% v = p.constructVisualizer; v.playback(xtraj5);
% display('iter 5')