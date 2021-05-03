%% Animation 

% In this script we animate the motorcycle based on the results obtained
% with the simulink model

ts_time = 0 : 0.1 : q_out.time(end);
ts_show = resample(q_out, ts_time, 'linear');
ts_show_ref = resample(ref_t, ts_time, 'linear');

% vidfile = VideoWriter('parls.avi');
% vidfile.FrameRate = 50; 
% vidfile.Quality = 100;
% open(vidfile)

% Set figure
f = figure('Name','Animation','units','pixels');
title('Rearing-up motorcycle','Interpreter','latex','FontSize',14)
grid on

% Animation for front wheel (initialization)
w_f = animatedline('color','#B00E0A','Marker','o','MarkerSize',50*R*2.5,'MarkerFaceColor','k'); 

% Animation for back wheel (initialization)
w_b = animatedline('color','#B00E0A','Marker','o','MarkerSize',50*R*2.5,'MarkerFaceColor','k');

c = animatedline('color','#B00E0A','Marker','o','MarkerSize',40*R*2.5,'MarkerFaceColor','#B00E0A');

% Animation for reference (initialization)
t = animatedline('color','#ff2400','LineStyle','-', 'LineWidth', 3);
r = animatedline('color','k','LineStyle','--', 'LineWidth', 3);

%Animation

for k = 1 : size(ts_show.time, 1)
    
    % Importing data from simulink
    q = ts_show.Data(k, :);        
    
    % Importing reference from simulink
    q_ref = ts_show_ref.Data(k, :);
    
    % System parameters
    x = q(1);
    theta = q(2);
    y = L*sin(theta);
    
    % Back wheel coordinates
    x1 = x - (L/2)*cos(theta);
    y1 = R;
    
    % Front wheel coordinates
    x2 = x + (L/2)*cos(theta);
    y2 = y1 + y;
    
    % Reference coordinates
    theta_ref = q_ref;
    x_ref = x + (L/2)*cos(theta_ref);
    y_ref = R + L*sin(theta_ref);
    
    % Axis
    axis([x1-1 x1+3 0 4])
    ylabel('y [m]')
    xlabel('time [s]')

    % Back wheel animation
    clearpoints(w_b);
    addpoints(w_b, x1, y1);
    
    % Front wheel animation
    clearpoints(w_f);
    addpoints(w_f, x2, y2);
    addpoints(t, x2, y2);
    
    % Reference animation
    addpoints(r, x_ref, y_ref);
    
    % Reference animation
    clearpoints(c);
    for i = 1 : 1 : 19
        xi = x - (L/2)*cos(theta) + (i/20)*L*cos(theta);
        yi = 1.5*R + (i/20)*L*sin(theta);
        addpoints(c, xi, yi);
    end
    
    drawnow
    
%     frame = getframe(f);
%     writeVideo(vidfile, frame);
    
end

% close(vidfile)