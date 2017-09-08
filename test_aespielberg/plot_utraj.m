function plot_utraj(utraj)

time = utraj.tspan(2);
samples = 1000;
utraj_plot = [];
sample_list = [];
for i = 1:1:samples
sample_time = time / samples * i;
utraj_value = utraj.eval(sample_time);
sample_list = [sample_list sample_time];
utraj_plot = [utraj_plot utraj_value];

end

plot(sample_list, utraj_plot)

end