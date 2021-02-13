function h=plot_fuelcell(fc, xfc)
% Fuel cell system (FCS) efficiency as a function of electrical power.
%   h=plot_fuelcell(fc) provides an FCS efficiency plot as a function of
%   electrical FCS power. FCS data is provided in the structure fc. The
%   output h is a a handle to the efficiency plot.
%
%   h=plot_fuelcell(fc, xfc) scales the FCS with a value xfc. If fc is a
%   fuel cell, then xfc can also be considered as number of cells in the
%   resulting stack.

if nargin < 2
    xfc=1;    % scale
end
% fuel power of the baseline fuel cell system.
Pf=[ones(numel(fc.Pix),1), fc.Pix, fc.Pix.^2]*(fc.a+[0;1;0]);
eta=fc.Pix./Pf;         % efficiency
h=plot(fc.Pix*xfc/1000,eta*100,'k','DisplayName','Efficiency [%]');
xlabel('Electrical power [kW]'); ylabel('Efficiency [%]');
title('Fuel cell system');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.