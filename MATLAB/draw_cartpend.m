function draw_cartpend(t_hist, y_hist, params)
    % Extract the CURRENT state for the animation from the end of the history
    y_curr = y_hist(end, :);
    x = y_curr(1);
    dx = y_curr(2);
    th = y_curr(3);
    dth = y_curr(4);

    % Dimensions
    g = params.g;
    L = params.L; 
    m = params.m;
    M = params.M;
    W = params.W;  
    H = params.H; 
    wr = params.wr; 
    mr = params.mr; 

    % Positions for the cart
    y = wr/2 + H/2; 
    w1x = x - .9*W/2;
    w1y = 0;
    w2x = x + .9*W/2 - wr;
    w2y = 0;
    px = x + L*sin(th);
    py = y - L*cos(th);

    % Ensure figure background is black
    set(gcf, 'Color', 'k', 'InvertHardcopy', 'off');

    % ==========================================
    % Subplot 1: The Cart Animation
    % ==========================================
    subplot(2, 1, 1);
    hold off; % Clear previous frame
    
    % Draw ground
    plot([-50 50], [0 0], 'w', 'LineWidth', 2);
    hold on;
    
    % Draw cart, wheels, pendulum, and mass
    rectangle('Position', [x-W/2, y-H/2, W, H], 'Curvature', .1, 'FaceColor', [1 0.1 0.1], 'EdgeColor', [1 1 1]);
    rectangle('Position', [w1x, w1y, wr, wr], 'Curvature', 1, 'FaceColor', [1 1 1], 'EdgeColor', [1 1 1]);
    rectangle('Position', [w2x, w2y, wr, wr], 'Curvature', 1, 'FaceColor', [1 1 1], 'EdgeColor', [1 1 1]);
    plot([x px], [y py], 'w', 'LineWidth', 2);
    rectangle('Position', [px-mr/2, py-mr/2, mr, mr], 'Curvature', 1, 'FaceColor', [.3 0.3 1], 'EdgeColor', [1 1 1]);
    
    % Formatting
    xlim([-5.1 5.1]);
    ylim([-.2 .2]);
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
    title('Cart-Pendulum Animation', 'Color', 'w');

    % ==========================================
    % Subplot 2: Live State Tracking
    % ==========================================
    subplot(2, 1, 2);
    hold off;
    
    % Plot position and angle history
    plot(t_hist, y_hist(:, 1), 'c', 'LineWidth', 1.5); % Position (Cyan)
    hold on;
    plot(t_hist, y_hist(:, 2), 'y', 'LineWidth', 1.5); % Angle (Magenta)
    grid on;
    plot(t_hist, y_hist(:, 3), 'm', 'LineWidth', 1.5); % Angle (Magenta)
    grid on;
    plot(t_hist, y_hist(:, 4), 'g', 'LineWidth', 1.5); % Angle (Magenta)
    grid on;

    % Dynamic x-axis that grows with time, fixed y-axis
    xlim([0, max(10, t_hist(end))]); 
    ylim([-10 10]); % Adjust bounds based on your expected swing range
    
    % Formatting to match the black theme
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    title('State Variables', 'Color', 'w');
    xlabel('Time (s)', 'Color', 'w');
    legend('Cart Position (m)', 'Cart Velocity (m/s)', 'Pendulum Angle (rad)', 'Pendulum Angular Velocity (rad/s)', 'TextColor', 'w', 'Location', 'northwest');

    drawnow;
end