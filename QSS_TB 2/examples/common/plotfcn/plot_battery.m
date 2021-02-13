function h=plot_battery(bat, n)
% Battery efficiency as a function of state of charge and power.
%   h=plot_battery(bat) provides a contour plot of battery efficiency as a
%   function of state of charge and power at the terminals. Battery data is
%   provided in the structure bat. The output h contains two handles, one
%   to the efficiency plot, and one to the power limits.
%
%   h=plot_battery(bat,n) scales the battery with a value n. If bat is a
%   battery cell, then n can also be considered as number of cells in the
%   resulting pack.

if nargin < 2
    n=1;    % number of cells/scale
end
% cell data
soclin=linspace(0,1,30)';
Vlin=interp1(bat.socix,bat.Voc,soclin);
Ilin=linspace(bat.Imin,bat.Imax,31)';
[soc,I]=ndgrid(soclin,Ilin);
V=interp1(bat.socix,bat.Voc,soc);
Pmin=(bat.Imin*Vlin-bat.R*bat.Imin^2);
Pmax=(bat.Imax*Vlin-bat.R*bat.Imax^2);
ix=Vlin < 2*bat.R*bat.Imax;
Pmax(ix)=Vlin(ix).^2/4/bat.R;
P=V.*I;             % internal power
Pt=P-bat.R*I.^2;    % power at the terminals
eta=Pt./P;          % efficiency
ix=I < 0;
eta(ix)=1./eta(ix); 
eta(eta < 0)=0;
eta(eta > 1)=1;
[~,h(1)]=contour(Pt*n/1000,soc*100,eta*100,[30:20:90,92:2:98, 99],'ShowText','on','DisplayName','Efficiency [%]'); hold on;
h(2)=plot(Pmax*n/1000,soclin*100,'k','LineWidth',2,'DisplayName','Power limits');
plot(Pmin*n/1000,soclin*100,'k','LineWidth',2);
ylabel('State of charge [%]');
xlabel('Power at pack terminals [kW]');
legend(h);
title('Battery');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-02.