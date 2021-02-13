% #####################################################################################################
%
% =====================================
% CVT Control (QSS TB manual, Sec. 2.7)
% =====================================
%
%
% Description
% -----------
%   QSS model of a small advanced light-weight vehicle with CVT.      
%
%
% Running the QSS program
% -----------------------
%   1. Open the Simulink file "qss_example_cvtctrl.mdl".
%   2. Double-click on the block "CVT Controller".
%   3. Set "Lower gear ratio limit [-]" to 3 and "Upper gear ratio limit [-]" to 15 (bandwith = 1:5).
%   4. Start the simulation.
%   5. Use the m-file "plot_example_cvtctrl.m" to display the results.
%   6. Now set "Lower gear ratio limit [-]" to 2 and "Upper gear ratio limit [-]" to 15 (bandwith = 1:7.5).
%   7. Repeat steps 4 - 5 and notice the differences.
%
% #####################################################################################################