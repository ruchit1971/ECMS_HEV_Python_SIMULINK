% ===================
%  Battery Controller
% ===================



% #####################################################################################################################

% Global variables
% ----------------
    global h                                                            % Stepsize [s] from block "Driving Cycle"
    global Q_BT_IC                                                      % Initial charge [C] from block "Battery"

% #####################################################################################################################

% Controller parameters
% ---------------------
    Q_BT_min   = 0.9*Q_BT_IC;											% Threshold
    length_c   = 200;													% Time interval loading (s)
    t_c        = 0:h:length_c*h;
    w_GG_c     = [0,[0:96:480],480*ones(1,length_c-13),[480:-96:0],0];  % Speed profile (smooth start/stop)
    nnn        = max(size(t_c));		
    dw_GG_c    = [0,w_GG_c(2:nnn)-w_GG_c(1:nnn-1)]/h;					% Acceleration
    scale_T_EG = 25;			
                                                                        % Torque profile (smooth start/stop)
    T_EG_c     = [0*ones(1,4),[0:scale_T_EG/10:scale_T_EG],scale_T_EG*ones(1,length_c-29),[scale_T_EG:-scale_T_EG/10:0],0*ones(1,4)];
    cc_done    = nnn-1;

% #####################################################################################################################
    