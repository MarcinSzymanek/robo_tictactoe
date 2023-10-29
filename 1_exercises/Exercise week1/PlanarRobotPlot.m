function PlanarRobotPlot( q, a, plotArea)
% Plot function for planar 2D robots
% Inputs :  q [1xN] array of angles
%           a [1xN] array of link lengths (a1, a2, ..)
%           plotArea [1x4] array of form [x_start x_end y_start y_end] to
%           indicate plot area

N = length(q);

o = [0 0]; % base frame origo
qinc = cumsum(q); % Incremental angle counter - sum of angles up to link i
plot(o(1), o(2), 'k.', 'MarkerSize', 40)
hold on
for i=1:N,
    % calculate frame i origo
    oi = [a(i)*cos(qinc(i)) a(i)*sin(qinc(i))] + o;
    
    % plot LINK from frame i-1 to frame i
    line1 = line([o(1) oi(1)], [o(2) oi(2)]);
    set(line1, 'LineWidth', 2)
    set(line1, 'Color', [1 0 0])
    
    % plot JOINT
    plot(oi(1), oi(2), 'k.', 'MarkerSize', 40)
    
    % update frame origo
    o = oi;
end
hold off
axis(plotArea)

end



