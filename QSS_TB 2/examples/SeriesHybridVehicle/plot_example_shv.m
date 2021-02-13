% ===============================================
% Plot results of example "Series Hybrid Vehicle"
% ===============================================



% #####################################################################################################################

close all
clc

% #####################################################################################################################

% Global variables
% ----------------
    
    % Electric Motor map
    global eta_EM_map
    global w_EM_row
    global T_EM_col
    global w_EM_max
    global T_EM_max

    % Electric Generator map
    global eta_EG_map
    global w_EG_row
    global T_EG_col
    global w_EG_max
    global T_EG_max

    % Combustion Engine map
    global eta_CE_map
    global w_CE_row
    global T_CE_col
    global w_CE_max
    global T_CE_max    

% #####################################################################################################################

% Battery
% -------
fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Battery')

    subplot(2,2,1)
    plot(t, q_BT)
    xlabel('Time [s]')
    ylabel('q_{BT} [0-1]')
    title('Battery charge ratio')

    subplot(2,2,2)
    plot(t, U_BT)
    xlabel('Time [s]')
    ylabel('U_{BT} [V]')
    title('Battery voltage')

    subplot(2,2,3)
    plot(t, I_BT)
    xlabel('Time [s]')
    ylabel('I_{BT} [A]')
    title('Battery current')

    subplot(2,2,4)
    plot(t, P_BT./1000)
    xlabel('Time [s]')
    ylabel('P_{BT} [kW]')
    title('Battery power')
    
% #####################################################################################################################

% Electric motor
% --------------
fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Electric Motor')
    
    vvv        = [0.7,0.8,0.85,1.0,1.1,1.15,1.2,1.3,1.4];
    [cc,hh]    = contour(w_EM_row, T_EM_col, eta_EM_map', vvv);
    clabel(cc,hh)
    hold on
    plot(w_EM_max, T_EM_max)
    plot(w_EM_max, -T_EM_max)
    xlabel('\omega_{EM} [rad/s]')
    ylabel('T_{EM} [Nm]')
    title('Electric Motor map')
    plot(w_EM, T_EM, 'o')
    hold off

% #####################################################################################################################

% Electric generator
% ------------------
fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Electric Generator')

    vvv     = [0.7,0.75,0.8,0.83,0.85,0.87,0.88,0.89,0.90,0.91,0.92,0.93,0.94];
    [cc,hh] = contour(w_EG_row, T_EG_col, eta_EG_map', vvv);
    clabel(cc,hh)
    hold on
    plot(w_EG_max, T_EG_max)
    xlabel('\omega_{EG} [rad/s]')
    ylabel('T_{EG} [Nm]')
    title('Electric Generator map')
    plot(w_EG, T_EG,'o')
    hold off

% #####################################################################################################################

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
    hold off

% #####################################################################################################################

