% Initialize the conventional vehicle.
datapath='../vehdata/';
load([datapath,'drivingcycle/dc_buslineGBG17']);    task.dc=dc;    % driving cylce including speed, time, and slope
load([datapath,'environment/env_25degC.mat']);      task.env=env;  % environment
load([datapath,'chassis/chs_14_5t.mat']);           task.chs=chs;  % vehicle chassis 
load([datapath,'ice/ice_130kW_2600rpm.mat']);       task.ice=ice;  % internal combustion engine
load([datapath,'transmission/trn_12gears.mat']);    task.trn=trn;  % transmission

task.t=dc.t; task.dt=dc.t(2)-dc.t(1); task.N=numel(dc.t);

%% Scale up the ICE. 
xice=1.1;            % scaling factor.
ice.Tix=ice.Tix*xice;
ice.Tmax=ice.Tmax*xice;
ice.Ploss=ice.Ploss*xice;
ice.a(1,:)=ice.a(1,:)*xice;
ice.a(3,:)=ice.a(3,:)/xice;
ice.J=ice.J*xice;
task.ice=ice;
                      
%% Costs and prices
d=sum(dc.v.*task.dt);                               %[m] length of the bus line
fuelprice=1.6;                                      %[eur/l]
fuelenergy=38.6e6;                                  %[J/l] diesel

%% Weighting factors
% The cost function to be minimized is computed as Wf*Efuel, where
% Efuel is fuel energy and Wf is a weighting factor.

task.Wf=fuelprice/fuelenergy/d*1e5;                 %[eur/W/100km] cost for fuel


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-11.
