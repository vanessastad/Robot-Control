%% Animation 

% In this script we animate the drone based on the results obtained
% with the simulink model

ts_time = 0 : 0.1 : q_out.time(end);
ts_show = resample(q_out, ts_time, 'linear');
ts_show_ref = resample(ref, ts_time, 'linear');

% vidfile = VideoWriter('hypo.avi');
% vidfile.FrameRate = 50; 
% vidfile.Quality = 100;
% open(vidfile)

% Set figure
f = figure('Name','Animation','units','pixels');
% title('Drone animation','Interpreter','latex','FontSize',14)
grid on

% Initialization
w_b = animatedline('color','#C6C00E','LineWidth', 3);
r = animatedline('color','k','LineStyle','--', 'LineWidth', 3);

% Animation

for k = 1 : size(ts_show.time, 1)
    
    % Importing data from simulink
    q = ts_show.Data(k, :); 
    q_ref = ts_show_ref.Data(k, :);
    
    % System parameters
    x = q(1);
    y = q(2);
    
    % References
    x_ref = q_ref(1);
    y_ref = q_ref(6);    
    
    % Axis
    if selector == 3
        axis([-50 50 -50 50]) 
    else
        axis([-30 30 -30 30]) 
    end
    %axis([-50 50 -50 50])
    ylabel('$z$ [m]','Interpreter','latex','FontSize',18)
    xlabel('$x$ [m]','Interpreter','latex','FontSize',18)
    
    % Animation
    addpoints(w_b, x, y);
    addpoints(r, x_ref, y_ref);
    
    drawnow
    
%     frame = getframe(f);
%     writeVideo(vidfile, frame);

end

% close(vidfile)