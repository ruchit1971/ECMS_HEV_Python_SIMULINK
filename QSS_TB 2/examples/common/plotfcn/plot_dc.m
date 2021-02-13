function h=plot_dc(dc,initalt)
% Reference velocity and road altitude as functions of distance.
%   h=plot_dc(dc) plots reference velocity and road altitude as functions
%   of distance. Initial road altitude is assumed zero. The output h
%   contains two handles, one to the velocity plot, and one to the altitude
%   plot.
%
%   h=plot_dc(dc,initalt) uses initalt as initial road altitude.

if nargin < 2
    initalt=0;    % scale
end

isflat=isempty(dc.slope) | all(dc.slope == 0);
dt=diff(dc.t); dt=[dt;min(dt)];
ds=dc.v.*dt;                    % distance traveled in one time sample
d=cumsum(cos(dc.slope).*ds);    % distance between the start and end point of the cycle
if ~isflat
    alt=initalt + cumsum(sin(dc.slope).*ds);
    h(2)=fill([d;flipud(d)]/1000,[alt;zeros(size(alt))],0.9*[1 1 1],'EdgeColor',0.7*[1 1 1],'DisplayName','Altitude [m]');
    hold on;
    legend('show');
end

h(1)=plot(d/1000,dc.v*3.6,'k','DisplayName','Speed');
xlabel('Distance [km]'); xlim([0 d(end)/1000]);
ylabel('Speed [km/h]');
title(dc.info.description);
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.
%   Modified by Nikolce Murgovski, 2015-02.
