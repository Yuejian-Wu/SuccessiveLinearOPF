function plot_results(mpc, solution, P_flow, Q_flow)
% PLOT_RESULTS Visualize OPF results
%
% Syntax:
%   plot_results(mpc, solution, P_flow, Q_flow)
%
% Description:
%   Creates comprehensive visualization plots of OPF results including:
%   - Bus voltage magnitudes
%   - Bus voltage angles
%   - Branch power flows
%   - Generator reactive power dispatch
%
% Author: Successive Linear OPF Team
% Date: 2026

    bus = mpc.bus;
    gen = mpc.gen;
    n_bus = size(bus, 1);
    n_gen = size(gen, 1);
    n_branch = size(mpc.branch, 1);
    
    % Create figure with subplots
    fig = figure('Name', 'OPF Results', 'NumberTitle', 'off');
    
    % ===== SUBPLOT 1: VOLTAGE MAGNITUDES =====
    subplot(2, 3, 1);
    V_min = bus(:, 13);
    V_max = bus(:, 12);
    V = solution.V;
    
    bar(1:n_bus, V, 'FaceColor', [0.2 0.6 1.0]);
    hold on;
    plot(1:n_bus, V_min, 'r--', 'LineWidth', 1.5, 'DisplayName', 'V_{min}');
    plot(1:n_bus, V_max, 'g--', 'LineWidth', 1.5, 'DisplayName', 'V_{max}');
    hold off;
    
    xlabel('Bus Number', 'FontSize', 11);
    ylabel('Voltage (p.u.)', 'FontSize', 11);
    title('Bus Voltage Magnitudes', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best');
    grid on;
    ylim([0.8 1.2]);
    
    % ===== SUBPLOT 2: VOLTAGE ANGLES =====
    subplot(2, 3, 2);
    theta_deg = solution.theta * 180 / pi;
    
    bar(1:n_bus, theta_deg, 'FaceColor', [1.0 0.6 0.2]);
    xlabel('Bus Number', 'FontSize', 11);
    ylabel('Angle (degrees)', 'FontSize', 11);
    title('Bus Voltage Angles', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    
    % ===== SUBPLOT 3: ACTIVE POWER FLOW =====
    subplot(2, 3, 3);
    Pij = P_flow.Pij;
    
    plot(1:n_branch, Pij, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    xlabel('Branch Number', 'FontSize', 11);
    ylabel('Active Power (MW)', 'FontSize', 11);
    title('Branch Active Power Flows', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    axhline(0, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 0.5);
    
    % ===== SUBPLOT 4: REACTIVE POWER FLOW =====
    subplot(2, 3, 4);
    Qij = P_flow.Qij;
    
    plot(1:n_branch, Qij, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
    xlabel('Branch Number', 'FontSize', 11);
    ylabel('Reactive Power (MVar)', 'FontSize', 11);
    title('Branch Reactive Power Flows', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    axhline(0, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 0.5);
    
    % ===== SUBPLOT 5: GENERATOR DISPATCH =====
    subplot(2, 3, 5);
    P_gen = gen(:, 2);
    Q_gen = gen(:, 3);
    Q_gen_max = gen(:, 4);
    Q_gen_min = gen(:, 5);
    
    x_pos = 1:n_gen;
    bar(x_pos - 0.2, Q_gen, 0.4, 'FaceColor', [0.2 0.8 0.2], 'DisplayName', 'Q_{actual}');
    hold on;
    plot(x_pos, Q_gen_max, 'g^--', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', 'Q_{max}');
    plot(x_pos, Q_gen_min, 'rv--', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', 'Q_{min}');
    hold off;
    
    xlabel('Generator Number', 'FontSize', 11);
    ylabel('Reactive Power (MVar)', 'FontSize', 11);
    title('Generator Reactive Power Dispatch', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best');
    grid on;
    
    % ===== SUBPLOT 6: SYSTEM LOSSES =====
    subplot(2, 3, 6);
    P_loss = P_flow.loss_P;
    Q_loss = P_flow.loss_Q;
    S_loss = P_flow.loss_S;
    
    data = [P_loss; Q_loss; S_loss];
    labels = {'P_{loss} (MW)', 'Q_{loss} (MVar)', 'S_{loss} (MVA)'};
    
    bar(data, 'FaceColor', [1.0 0.4 0.2]);
    set(gca, 'XTickLabel', labels);
    ylabel('Loss Magnitude', 'FontSize', 11);
    title('System Losses', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    
    fprintf('Results visualization plotted successfully.\n');
    
end
