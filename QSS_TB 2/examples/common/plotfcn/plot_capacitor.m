function h=plot_capacitor(cap, n)
% Capacitor efficiency as a function of state of charge and power.
%   h=plot_capacitor(cap) provides a contour plot of capacitor efficiency
%   as a function of state of charge and power at the terminals. Capacitor
%   data is provided in the structure cap. The output h contains two
%   handles, one to the efficiency plot, and one to the power limits.
%
%   h=plot_capacitor(cap,n) scales the capacitor with a value n. If cap is
%   a capacitor cell, then n can also be considered as number of cells in
%   the resulting pack.

if nargin < 2
    n=1;    % number of cells/scale
end
% cell data
Vlin=linspace(0,cap.Vmax,41)';
soclin=Vlin/cap.Vmax;
Ilin=linspace(cap.Imin,cap.Imax,51)';
[V,I]=ndgrid(Vlin,Ilin);
E=cap.C*V.^2/2;
Pmin=(cap.Imin*Vlin-cap.R*cap.Imin^2);
Pmax=(cap.Imax*Vlin-cap.R*cap.Imax^2);
ix=Vlin < 2*cap.R*cap.Imax;
Pmax(ix)=Vlin(ix).^2/4/cap.R;
soc=V/cap.Vmax;
P=V.*I;             % internal power
Pt=P-cap.R*I.^2;    % power at the terminals
eta=Pt./P;
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
title('Supercapacitor');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.