% =========
% Fuel Cell
% =========



% #####################################################################################################################

% Global variables
% ----------------
    global h                                                                                % Stepsize [s] from block "Driving Cycle"

% #####################################################################################################################

% Air 
% ---
    T_air           = T_air + 273.15;                                                       %                                           [°C] -> [K]
    kappa_air       = 1.4;                                                                  % Isentropic exponent                       [-]
    R_air     		= 287;                                                                  % Gas constant                              [J/(mol*K)]
    cp_air          = kappa_air/(kappa_air-1)*R_air;                                        % Specific heat                             [J/(mol*K)]
    M_air           = 29.0;                                                                 % Molecular weight                          [kg/kmol]    
    delta_h_C_air 	= cp_air*T_air/eta_C*(pi_C^((kappa_air-1)/kappa_air)-1);                % Enthalpy difference                       [J]
    
% Fuel (hydrogen)
% ---------------
    H_o             = 142.9e6;                                                              %                                           [J/kg] 
    H_o_m           = 285.9e6;                                                              %                                           [J/kmol]

% #####################################################################################################################

% Load Fuel Cell characteristics
% ------------------------------
    load i_FC_in                                                                            % Fuel cell model input: current density    [A/m^2] 
    load V_FC_out                                                                           % Fuel cel model output: voltage            [V]

    i_FC_max        = i_FC_in(length(i_FC_in));                                             % Maximum current density                   [A/m^2]
    P_FC_max		= i_FC_in(length(i_FC_in)) * V_FC_out(length(V_FC_out)) * A_FC*N_FC;    % Maximum power                             [W]

% #####################################################################################################################