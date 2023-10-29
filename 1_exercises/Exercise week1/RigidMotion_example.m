%% Planar robot 2D

% link lenghts
a1 = 20 
a2 = 30

% Robot configuration
q1 = pi/2

% Rigid motion
d0 = [a1*cos(q1) ; a1*sin(q1)]
R01 = [cos(q1) -sin(q1); sin(q1) cos(q1)] 

% Point, in coord 1
p1 = [1; 1]

% in coord 0 (base coords)
p0 = R01*p1 + d0



