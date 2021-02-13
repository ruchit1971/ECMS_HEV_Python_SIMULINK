function h=plot_ice(ice, xice)
% Internal combustion engine (ICE) efficiency as a function of speed and
% torque.
%   h=plot_ice(ice) provides a contour plot of ICE efficiency as a function
%   of ICE speed and torque. ICE data is provided in the structure ice. The
%   output h contains two handles, one to the efficiency plot, and one to
%   the torque limit.
%
%   h=plot_ice(ice, xice) scales the ICE with a value xice. ICE torque and
%   losses are scaled linearly, while the speed is kept unchanged.

if nargin < 2
    xice=1;    % scale
end

[W,T]=ndgrid(ice.wix,ice.Tix);
TMAX=interp1(ice.wix,ice.Tmax,W);
ok=T <= TMAX;
eta=ice.eta';
eta(~ok)=NaN; % infeasible points are not shown on contour plot

h=NaN(2,1);
[~,h(1)]=contour(W*30/pi,T*xice,eta*100,[10:5:35,36:50],'ShowText','on','DisplayName','Efficiency [%]'); hold on;
h(2)=plot(ice.wix*30/pi,ice.Tmax*xice,'k','LineWidth',2,'DisplayName','Torque limit');
xlabel('Speed [rpm]'); ylabel('Torque [Nm]');
legend(h);
title('Internal combustion engine');
xlim([ice.wix(1) ice.Wmax]*30/pi);
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.