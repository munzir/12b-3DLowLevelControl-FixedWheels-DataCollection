clc; clear; close all;
dof   = 18;

q = tdfread('./dataQ.txt', '\t');
q = q.dataQ;

qref = tdfread('./dataQref.txt', '\t');
qref = qref.dataQref;

time = tdfread('./dataTime.txt');
time = time.dataTime;

rows = 3; cols = 3;
for i = 1:dof/2
    subplot(rows, cols, i);
    plot(time, q(:,i), time, qref(:,i)); grid on;
    xlabel('Time');
    ylabel('State Trajectories');
    leg1 = legend('$q(t)$','$q_{ref}(t)$');
    set(leg1,'Interpreter','latex');
    xlabel(['Joint ' num2str(i)])
end

figure;
for i = (dof/2+1):dof
    subplot(rows, cols, i-9);
    plot(time, q(:,i), time, qref(:,i)); grid on;
    xlabel('Time');
    ylabel('State Trajectories');
    leg1 = legend('$q(t)$','$q_{ref}(t)$');
    set(leg1,'Interpreter','latex');
    xlabel(['Joint ' num2str(i)])
end

title('Tracking Error');