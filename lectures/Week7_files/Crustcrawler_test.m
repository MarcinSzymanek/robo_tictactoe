%% Init model
clear

% Init Robotics Toolbox
mydir = pwd;
cd('U:\Kurser_undervisning\ITROB1\PeterCorke_Matlab_robotics\robot-10.2\rvctools')
startup_rvc
cd(mydir)

% Load crustcrawler
load CrustCrawler_object

%% Forward Kinematic
qn = [0 0 0 0.3];
T1 = crawl.fkine(qn)
crawl.plot(qn); %, 'workspace', [-10 10 -10 10 -10 10])
crawl.teach()

%% Testing Inverse Kinematics
o = [35 0 0];
qn = CrustInvKin(o);
crawl.plot(qn); %, 'workspace', [-10 10 -10 10 -10 10])
crawl.teach()

o = [-15 15 9];
tic
for i=1:100000,
    qn = CrustInvKin(o);
end
toc
crawl.plot(qn); %, 'workspace', [-10 10 -10 10 -10 10])
crawl.teach()

% Compare with built-in func
T1 = crawl.fkine(qn)
tic
for i=1:100
    qn2 = crawl.ikine(T1, 'q0', [0 0 0 0], 'mask', [1 1 1 0 0 0]);
end
toc
crawl.plot(qn2); %, 'workspace', [-10 10 -10 10 -10 10])
crawl.teach()



%% Generate path
xarr = -30:1:30; % y-values
yarr = -25*ones(size(xarr)); % x-values (constant)
zarr = 1*ones(size(xarr)); % z-values (constant)
qseq=[];
for i=1:length(xarr),
    qseq(i,:) = CrustInvKin( [xarr(i) yarr(i) zarr(i)] );
end
% qseq = mtraj(@lspb, CrustInvKin([-20 -25 1]), CrustInvKin([20 -15 10]), 50); % smooth (linear segment parabolic blend) curve in joint-space
crawl.plot(qseq, 'trail', 'r')



%% Jacobian
q0 = [0 0 0 0];
J = crawl.jacob0(q0)
% NOTES: 
% dv_x = 0 (due to singularity)
% dv_y - only depend on q1
% dv_z - depend on both both q2 and q3. 
% dw_x - only depend on q4. 
% dw_y - depend on q2 and q3. 
% dw_z - only depend on q1.

% Manipulability measure ("closeness" to singularity) 
crawl.maniplty(q0, 'yoshikawa')



%% Resolved rate motion control
% oinit = [-30 -25 1]; % initial position
oinit = [-30 0 0]; % initial position
dx = [0.5 0 0]; % velocity
N = 100; % number steps (to reach x=30)

qi = CrustInvKin(oinit); % initial joints
qseq = qi; % first element of sequence
for i=1:N,
    % Finding jacobian
    J = crawl.jacob0(qi);
    Jcut = J(1:3, 1:3); % only consider position..
    
    manip(i) = abs(det(Jcut));  % calculate determinant (=yoshikawa manipulability for square J)
    
    % update qi
    dq = inv(Jcut)*dx';
    qi = qi + [dq' 0]; % just setting q4 = 0
    
    % update sequence
    qseq = [qseq; qi]; % adding extra joint values to sequence
end

crawl.plot(qseq, 'trail', 'r')
figure, plot(manip)
