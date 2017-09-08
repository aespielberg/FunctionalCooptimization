function [ cost ] = computeCost(utraj, dt )

%find the cost of utraj

N = length(dt)+1; %dt doesn't include time 0
u = zeros(N,utraj.getNumOutputs());
t = 1:N;
t(1) = 0;
u(1,:) = utraj.eval(0)';
for i=2:N
    t(i) = t(i-1) + dt(i-1);
    u(i,:) = utraj.eval(t(i))';
end

cost = 0;
for i=1:N-1
    addCost = 0.5*(u(i,:)*u(i,:)' + u(i+1,:)*u(i+1,:)')*dt(i);
    cost = cost + addCost;
end

end

