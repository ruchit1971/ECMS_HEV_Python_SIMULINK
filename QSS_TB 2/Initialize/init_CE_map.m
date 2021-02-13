% ============================================
% Combustion Engine (based on consumption map)
% ============================================



% #####################################################################################################################

% Global variables
% ----------------
    global h                                    % Stepsize [s] from block "Driving Cycle"
    global eta_CE_map
    global w_CE_row
    global T_CE_col
    global w_CE_max
    global T_CE_max
    
% #####################################################################################################################

% Load data
% ---------
    load V_CE_map                               % Consumption map                           [kg/s]
    load eta_CE_map                             % Efficiency map                            [-]
    load w_CE_row                               % Engine speed range                        [rad/s]
    load p_me_col                               % Brake mean effective pressure range       [Pa]
    load w_CE_max                               % Maximum engine speed                      [rad/s]
    load p_me_max                               % Maximum brake mean effective pressure     [Pa]

    w_CE_upper      = max(w_CE_max);            % Upper limit engine speed                  [rad/s]

    V_d 			= V_d/1000;                 % Displaced volume                          [l] -> [m^3]
    V_CE_map        = V_CE_map .* scale_CE;     % Scale engine consumption                   
        
    T_CE_col        = p_me_col .* V_d/(4*pi);   % Torque range                              [Nm]
    T_CE_max        = p_me_max .* V_d/(4*pi);   % Maximum torque                            [Nm]

    T_CE_idle = P_CE_idle / w_CE_idle;          % Torque at idle                            [Nm]   
    
% #####################################################################################################################

% Which engine type?
% ------------------
    switch engine_type

    case 1                                      % -> Otto
        H_u         = 42.7e6;                   % Bosch-Manual
        rho_f       = 0.745;                    % Bosch-Manual

    case 2                                      % -> Diesel
        H_u         = 42.5e6;                   % Bosch-Manual
        rho_f       = 0.84;                     % Bosch-Manual
    end
    
% #####################################################################################################################
