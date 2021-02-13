function h=plot_em(em, xem)
% Electric machine (EM) efficiency as a function of speed and torque.
%   h=plot_em(em)) provides a contour plot of EM efficiency as a function
%   of EM speed and torque. EM data is provided in the structure em. The
%   output h contains two handles, one to the efficiency plot, and one to
%   the torque limits.
%
%   h=plot_em(em, xem) scales the EM with a value xem. EM torque and losses
%   are scaled linearly, while the speed is kept unchanged.

if nargin < 2
    xem=1;    % scale
end

[W,T]=ndgrid(em.wix,em.Tix);
TMAX=interp1(em.wix,em.Tmax,W);
TMIN=interp1(em.wix,em.Tmin,W);
ok=T >= TMIN & T <= TMAX;
eta=em.eta';
eta(~ok)=NaN; % infeasible points are not shown on contour plot

h=NaN(2,1);
[~,h(1)]=contour(W*30/pi,T*xem,eta*100,[10:20:70,75:5:90,91:100],'ShowText','on','DisplayName','Efficiency [%]'); hold on;
h(2)=plot(em.wix*30/pi,em.Tmax*xem,'k','LineWidth',2,'DisplayName','Torque limits');
plot(em.wix*30/pi,em.Tmin*xem,'k','LineWidth',2);
xlabel('Speed [rpm]'); ylabel('Torque [Nm]');
legend(h);
title('Electric machine');
xlim([0 em.Wmax*30/pi]);
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.