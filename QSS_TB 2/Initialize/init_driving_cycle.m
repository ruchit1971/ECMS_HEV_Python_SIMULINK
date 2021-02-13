% ==============
% Driving Cycles
% ==============



% #####################################################################################################################

% Global variables
% ----------------
    global h            % Stepsize [s] for all other blocks
    global N_sim        % Number of computational steps required to complete the simulation of the cycle [-] for all other blocks
    
% #####################################################################################################################

% File names of the cycles
% ------------------------
cyclestring = {'ECE_MAN'			
   'ECE_R15%2f04'
   'ECE_R15'
   'EUDC'
   'EUDC_MAN'
   'City'
   'City_MAN'
   'NEDC'
   'NEDC_MAN'
   '10_15_MODE'
   '10_MODE'
   '10_MODE_3'
   '11_MODE'
   '11_MODE_4'
   '15_MODE'
   'CITY_I'
   'CITY_II'
   'FTP_75'
   'FTP_HIGHWAY'};

% #####################################################################################################################

% Load data
% ---------
    data = load(char(cyclestring(cyclenr)));

    T_z = data.T_z;
    V_z = data.V_z;
    D_z = data.D_z;
    G_z = data.G_z;
    
    clear data

% #####################################################################################################################

% Prepare data (here: to plot a nice picture onto the block)
% ----------------------------------------------------------
    T_zplot = T_z;
    V_zplot = V_z;

    for nnn = length(T_zplot):round(length(T_zplot)*1.25)
       T_zplot(nnn+1) = T_zplot(nnn) + 1;
       V_zplot(nnn+1) = 0;
    end

% #####################################################################################################################

% Store specific data
% -------------------
    stoptime = T_z(length(T_z));
    
    if autostop == 1
        set_param(char(bdroot(gcb)),'StopTime',num2str(stoptime));
    end     
   
    cl_par = struct('stepsize', stepsize, 'stoptime', stoptime);
    set_param(gcb, 'UserData', cl_par);

% #####################################################################################################################

% Simulation parameters
% ---------------------
    h = stepsize; 
    N_sim = stoptime;
    
% #####################################################################################################################
    
