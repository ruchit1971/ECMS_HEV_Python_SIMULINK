% Initialize the hybrid vehicle.
datapath='../vehdata/';
load([datapath,'drivingcycle/dc_buslineGBG17']);    task.dc=dc;    % driving cylce including speed, time, and slope
load([datapath,'environment/env_25degC.mat']);      task.env=env;  % environment
load([datapath,'chassis/chs_14_5t.mat']);           task.chs=chs;  % vehicle chassis 
load([datapath,'egu/egu_100kW.mat']);               task.egu=egu;  % EGU system
load([datapath,'em/em_220kW_3000rpm.mat']);         task.em=em;    % electic machine
load([datapath,'battery/bat_2_3Ah.mat']);           task.bat=bat;  % battery cell

task.t=dc.t; task.dt=dc.t(2)-dc.t(1); task.N=numel(dc.t);
                      
%% Costs and prices
d=sum(dc.v.*task.dt);                               %[m] length of the bus line
batcost=900/3.6e6;                                  %[eur/Ws], i.e. 900 EUR/kWh, cost for battery
fuelprice=1.6;                                      %[eur/l]
fuelenergy=38.6e6;                                  %[J/l] diesel
egucost=43e-3;                                      %[eur/W] cost for EGU
batcellcost=batcost*bat.Q*bat.Vnom;                 %[eur] cost for one cell
nybus=5;                                            %[-] bus service period
interestbat=5/100;                                  %[-] yearly interest rate for the battery
interestegu=5/100;                                  %[-] yearly interest rate for the EGU
avgtravel=70e6;                                     %[m] average anual mileage

%% Weighting factors
% The cost function to be minimized is computed as Wf*Efuel + Wb*xb +
% Wegu*xegu, where Efuel is fuel energy, Wf, Wb and Wegu are weighting
% factors, and xb and xegu are scales of buffer and EGU system. Costs of
% EGU and buffer are first normalized per km and then multiplied with the
% length of the cycle. Depreciation expenses are included.

task.Wf=fuelprice/fuelenergy/d*1e5;                                                 %[eur/W/100km] cost for fuel energy
task.Wb=task.nbbase*batcellcost/nybus/avgtravel*(1+interestbat*(nybus+1)/2)*1e5;    %[eur/100km]
task.Wegu=egucost*egu.Pmax/nybus/avgtravel*(1+interestegu*(nybus+1)/2)*1e5;         %[eur/100km]




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.
