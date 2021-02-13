% Initialize the hybrid vehicle.
datapath='../vehdata/';
load([datapath,'drivingcycle/dc_buslineGBG17']);    task.dc=dc;    % driving cylce including speed, time, and slope
load([datapath,'environment/env_25degC.mat']);      task.env=env;  % environment
load([datapath,'chassis/chs_14_5t.mat']);           task.chs=chs;  % vehicle chassis 
load([datapath,'ice/ice_130kW_2600rpm.mat']);       task.ice=ice;  % internal combustion engine
load([datapath,'em/em_220kW_3000rpm.mat']);         task.em=em;    % electic machine
load([datapath,'battery/bat_2_3Ah.mat']);           task.bat=bat;  % battery cell
load([datapath,'transmission/trn_12gears.mat']);    task.trn=trn;  % transmission

task.t=dc.t; task.dt=dc.t(2)-dc.t(1); task.N=numel(dc.t);

%% Scale down the EM. 
xem=0.5;            % scaling factor.
em.Wmax=ice.Wmax;   % constrain ICE and EM to equal speed range.
em.Tix=em.Tix*xem;
em.Tmax=em.Tmax*xem;
em.Tmin=em.Tmin*xem;
em.Ploss=em.Ploss*xem;
em.a(1,:)=em.a(1,:)*xem;
em.a(3,:)=em.a(3,:)/xem;
task.em=em;
                      
%% Costs and prices
d=sum(dc.v.*task.dt);                               %[m] length of the bus line
batcost=900/3.6e6;                                  %[eur/Ws], i.e. 900 EUR/kWh, cost for battery
fuelprice=1.6;                                      %[eur/l]
fuelenergy=38.6e6;                                  %[J/l] diesel
batcellcost=batcost*bat.Q*bat.Vnom;                 %[eur] cost for one cell
nybus=5;                                            %[-] bus service period
interestbat=5/100;                                  %[-] yearly interest rate for the battery
avgtravel=70e6;                                     %[m] average anual mileage

%% Weighting factors
% The cost function to be minimized is computed as Wf*Efuel + Wb*xb, where
% Efuel is fuel energy, Wf and Wb are weighting factors, and xb is a buffer
% scale. The buffer's cost is first normalized per km and then multiplied
% with the length of the cycle. Depreciation expenses are included.

task.Wf=fuelprice/fuelenergy/d*1e5;                                                 %[eur/W/100km] cost for fuel
task.Wb=task.nbbase*batcellcost/nybus/avgtravel*(1+interestbat*(nybus+1)/2)*1e5;    %[eur/100km]




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.
