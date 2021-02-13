function h=plot_egu(egu, xegu)
% Engine generator unit (EGU) efficiency as a function of electrical power.
%   h=plot_egu(egu) provides an EGU efficiency plot as a function of
%   electrical EGU power. EGU data is provided in the structure egu. The
%   output h is a handle to the efficiency plot.
%
%   h=plot_egu(egu, xegu) linearly scales the EGU with a value xegu. 

if nargin < 2
    xegu=1;    % scale
end
% fuel power of the baseline fuel cell system.
Pf=[ones(numel(egu.Pix),1), egu.Pix, egu.Pix.^2]*(egu.a+[0;1;0]);
eta=egu.Pix./Pf;         % efficiency
h=plot(egu.Pix*xegu/1000,eta*100,'k','DisplayName','Efficiency [%]');
xlabel('Electrical power [kW]'); ylabel('Efficiency [%]');
title('Engine generator unit');
xlim([0 egu.Pmax*xegu/1000]);
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-03.