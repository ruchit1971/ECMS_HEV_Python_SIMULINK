% Initialize the hybrid vehicle.
datapath='../vehdata/';
load([datapath,'drivingcycle/dc_buslineGBG17']);    task.dc=dc;    % driving cylce including speed, time, and slope
load([datapath,'environment/env_25degC.mat']);      task.env=env;  % environment
load([datapath,'chassis/chs_14_5t.mat']);           task.chs=chs;  % vehicle chasis 
load([datapath,'fuelcell/fc_50kW.mat']);            task.fc=fc;    % fuel cell system
load([datapath,'em/em_220kW_3000rpm.mat']);         task.em=em;    % electic machine
load([datapath,'capacitor/cap_2_7V_3000F.mat']);    task.cap=cap;  % supercapacitor cell

task.t=dc.t; task.dt=dc.t(2)-dc.t(1); task.N=numel(dc.t);
                      
%% Costs and prices
d=sum(dc.v.*task.dt);                               %[m] length of the bus line
capcost=8/3.6e3;                                    %[eur/Ws], i.e. 10 kEUR/kWh, cost for capacitor
fuelprice=4.44;                                     %[eur/kg]
fuelenergy=120e6;                                   %[J/kg] hydrogen
fccost=34.78e-3;                                    %[eur/W]  fuel cell cost
capcellcost=capcost*cap.C*cap.Vmax^2/2;             %[eur] cost for one cell
nybus=2;                                            %[-] bus service period
interestcap=5/100;                                  %[-] yearly interest rate for the capacitor
interest_fc=5/100;                                  %[-] yearly interest rate for the fuel cell
avgtravel=70e6;                                     %[m] average anual mileage

%% Weighting factors
% The cost function to be minimized is computed as Wh*Efuel + Wb*xb +
% Wfc*xfc, where Efuel is fuel energy, Wh, Wb and Wfc are weighting
% factors, and xb and xfc are scales of capacitor and fuel cell system.
% Costs of fuel and buffer cells are first normalized per km and then
% multiplied with the length of the cycle. Depreciation expenses are
% included.
task.Wh=fuelprice/fuelenergy/d*1e5;                                                 %[eur/W/100km] cost for fuel
task.Wb=task.nbbase*capcellcost/nybus/avgtravel*(1+interestcap*(nybus+1)/2)*1e5;    %[eur/100km]
task.Wfc=fccost*fc.Pmax/nybus/avgtravel*(1+interest_fc*(nybus+1)/2)*1e5;            %[eur/100km]




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.
