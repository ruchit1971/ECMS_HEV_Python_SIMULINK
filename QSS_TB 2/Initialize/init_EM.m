% ==============
% Electric Motor
% ==============



% #####################################################################################################################

% Global variables
% ----------------
    global h                            % Stepsize [s] from block "Driving Cycle"
    global eta_EM_map
    global w_EM_row
    global T_EM_col
    global w_EM_max
    global T_EM_max
        
% #####################################################################################################################

% Load data
% ---------
    load eta_EM_map                     % Efficiency map                [-]
    
    load w_EM_row                       % Motor speed range             [rad/s]
    load T_EM_col                       % Motor torque range            [Nm]
    load w_EM_max                       % Maximum motor speed           [rad/s]    
    load T_EM_max                       % Maximum motor torque          [Nm]

    w_EM_upper = max(w_EM_max);         % Upper limit motor speed       [rad/s]
    
    T_EM_col = scale_EM * T_EM_col;     % Scale motor torque
    T_EM_max = scale_EM * T_EM_max;

% #####################################################################################################################


