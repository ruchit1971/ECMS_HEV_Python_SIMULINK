% =====================================
% Plot results of example "CVT Control"
% =====================================



% #####################################################################################################################

close all
clc

% #####################################################################################################################

% Global variables
% ----------------
    
    % Combustion Engine map
    global eta_CE_map
    global w_CE_row
    global T_CE_col
    global w_CE_max
    global T_CE_max    

% #####################################################################################################################

init_CVT_ctrl;

% Combustion engine
% -----------------
fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Combustion Engine')

    val_vec=[ 0.33 0.32 0.3 0.275 0.2  0.1];
    cs=contour(w_CE_row, T_CE_col, eta_CE_map, val_vec);
    clabel(cs)
    title('Combustion Engine map')
    xlabel('\omega_{CE} [rad/s]')
    ylabel('T_{CE} [Nm]')
    hold on
    plot(w_CE_max, T_CE_max)
    plot(w_CE, T_CE, 'o')
    plot(w_CVT_opt, T_CVT_opt, 'r-', 'LineWidth', 2)
    hold off

% #####################################################################################################################
    