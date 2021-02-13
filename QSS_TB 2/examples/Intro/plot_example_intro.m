% ========================================
% Plot results of the introductory example 
% ========================================



% #####################################################################################################################

close all
clc

% #####################################################################################################################

% Fuel consumption [g/s]
% ----------------------
fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Fuel consumption [g/s]')

    plot(t, m_dot_fuel .* 1000)
    xlabel('Time [s]')
    ylabel('Fuel consumption [g/s]')
    axis([0 1200 0 1.4])

% Fuel consumption [liters/100 km]
% --------------------------------
fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Fuel consumption [liters/100 km]')

    plot(t, V_liter)
    xlabel('Time [s]')
    ylabel('Fuel consumption [liters/100 km]')
    axis([0 1200 0 10])
        
% Drivetrain efficiency 
% ---------------------
fig = figure;
set(fig,'NumberTitle', 'off')
set(fig,'Name', 'Drivetrain efficiency')

    % Consider only traction phase
    for i = 1 : length(P_wheel)
        if (P_MGB(i) > 0)
            eta_DT(i) = P_wheel(i)/P_MGB(i);
        else
            eta_DT(i) = 0;
        end
    end

    plot(t, eta_DT)
    xlabel('Time [s]')
    ylabel('\eta_{DT,trac} [-]')
    axis([600 1200 0 1])

% #####################################################################################################################
