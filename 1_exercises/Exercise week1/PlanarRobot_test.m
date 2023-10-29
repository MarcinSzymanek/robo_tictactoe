%% Test Forward Kinematics
a1 = 20; % link lenghts
a2 = 30;

q1 = pi/4;
q2 = -0.5;
[x,y] = PlanarRobotFK(q1, q2, a1, a2)

PlanarRobotPlot([q1 q2],[a1 a2], [-100 100 -100 100])
grid minor
%% Test Inverse Kinematics
x = -5;
y = -40;

[q1, q2] = PlanarRobotIK(x, y, a1, a2)
[x,y] = PlanarRobotFK(q1, q2, a1, a2)

PlanarRobotPlot([q1 q2], [a1 a2], [-100 100 -100 100])
grid minor

%% Trajectory test 1
xarr = -20:2:30; % x-values
yarr = 15*ones(size(xarr)); % y-values (constant)

for i=1:length(xarr),
%     [q1, q2] = PlanarRobotIK_solution(xarr(i), yarr(i), a1, a2);
    [q1, q2] = PlanarRobotIK(xarr(i), yarr(i), a1, a2);
    
    q = [q1 q2];
    PlanarRobotPlot(q, [a1 a2], [-100 100 -100 100])
    grid minor
    
    pause() % until keyboard button pushed..
end

%% Plotting a (redundant) 8-DOF planar robot ("snake")
a = [40 40 30 30 20 10 10 10];
q = [pi/4 pi/4 pi/4 pi/4 pi/4 0 pi/4 pi/4];

PlanarRobotPlot(q, a, [-100 100 -100 100])
grid minor

