clc; clear;
wf = 0.558048373585;
a = [-0.009, -0.36, 0.311, -0.362; 0.095, -0.132, -0.363, 0.474; -0.418, -0.25, -0.12, 0.119; 0.023, 0.113, 0.497, 0.213; ...
    -0.23, -0.237, 0.153, -0.147; 0.366, 0.366, 0.302, -0.373; -0.247, -0.166, 0.315, 0.031];

b = [-0.051, 0.027, 0.003, -0.332; -0.292, 0.358, -0.056, -0.436; -0.355, 0.039, -0.397, -0.445; 0.328, 0.256, -0.36, 0.143;...
    0.428, 0.093, 0.035, -0.28; -0.39, -0.085, 0.388, 0.46; -0.046, 0.135, -0.428, 0.387];

q = [0.235, -0.004, -0.071, 0.095, -0.141, 0.208, -0.182];

% time = round(4*2*pi/wf);
% time = linspace(0,time,100);
time = 0;

angles = zeros(7,length(time));
%%
for joint = 1:size(a,1)
    %     for t = 1:length(time)
    for l = 1:size(a,2)
%         angles(joint, t) = angles(joint, t) + (a(joint,l)/(wf*l))*sin(wf*l*time(t)) -  (b(joint,l)/(wf*l))*cos(wf*l*time(t) ) + q(joint);
        angles(joint) = angles(joint) + (a(joint,l)/(wf*l))*sin(wf*l*time) -  (b(joint,l)/(wf*l))*cos(wf*l*time );
    end
    %     end
end
angles = angles + q'
% angles = angles*180/pi