function [u]=parallelhybrid_ECMS(x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This files calculates torque split for simulink model. 
% Given inputs from the driving cycle and controller this script should return
% The torques on the electric motor and combustion engine.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Input data from Simulink Model
w_ice=x(1);   % Angular Speed for ICE
dw_ice=x(2);  % Angular Acceleration for ICE
T_req=x(3);   % Troque Request or demand
lambda=x(4);  % Equivalence Factor

%% Fuel and Air Parameters
H_l= 44.6e6; % Lower Heating Value [J/kg]
roha_petrol=732.2; % Fuel Density [Kg/m3]
roha_air=1.18; % Air Density [kg/m3] 

%% Engine Configuration
J_e=0.2; % Engine Inertia [kgm2]
T_ice_max= 115; % Engine Maximum Torque [Nm]
V_d=1.497e-3; % Engine Displacment [m3]
n_r= 2;
m_e=1.2;  % [kg/KW]
  
%% Willans approximation of engine efficiency
e=0.4;  % Willans approximation of engine efficiency
p_me_0=0.1e6; % Willans approximation of engine pressure [MPa]

%% Battery Configuration
Q_0=6.5; % Battery charging capacity [Ah]
U_oc=300; % Open circuit voltage [V]
I_max= 200;% Maximum dis-/charging current [A]
I_min=-200; % Maximum dis-/charging current [A]
R_i= 0.65; % Inner resistance [ohm]
m_batt= 45; % [kg]
  
%% Motor and Generator Configuration
n_machine=0.9; % Efficiency of electrical machine
T_em_max=400; % Maximum Electric machine Torque [Nm]
P_em_max=50; % Power of Electric Machine [kW]
m_em=1.5; %  Electric motor weight [kg/Kw]
  
%% TotalPowertrain Power
P_total_max=90.8; % Total powertrain power[kW]
  
%% Logitudinal Vehicle Model Parameters
g=9.81;  %  Gravity [m/s2]
c_D=0.32; % Drag coefficient [-]
c_R=0.015; % Rolling resistance coefficient [-]
A_f=2.31; % Frontal area [m2]
m= 1500; % Vehicle mass [kg]
r_w=0.3; % Wheel radius [m]
J_w=0.6; % Inertia of the wheels [kgm2]
m_wheel=J_w/(r_w^2); % Mass of Wheel [kg]
n_gearbox=0.98;  % Efficiency of Transmission

%% Range
N_e= 0:800:5000; % Engine RPM range
w_e= 300; % [rad/s2] % Maximum accelration of the engine


%-----------------------Cost Vector Calculations--------------------------%
%% Battery Model

I=linspace(I_min,I_max,50000);

Power_battery=(U_oc*I)-(R_i*I.^2);   % Power of Battery


%% Electric Machine Model

Power_electric_machine=Power_battery.*n_machine.^sign(I);  % Power of Electric Machine

T_em=(Power_battery.*n_machine.^sign(I))./w_ice;   % Torque of Electric Machine

%% Engine Model

T_ice=T_req-T_em;  % Torque of Engine

x=((p_me_0*V_d)/(4*pi));  % First Part to calculate Fuel consumption

y=J_e*dw_ice; % Second Part to calculate Fuel consumption

costVector=(w_ice/(e*H_l))*(T_ice+x+y); % Fuel Consumption Cost Vector for given t_vec


%% Hamiltonian Calculation [Torque Split]

P_f=H_l*costVector; % Fuel Power
P_f(costVector<0)=0; % Constraints
P_ech=I*U_oc;  % Electro-chemical Power for Battery

%%% If condition for torque split
if w_ice==0
    T_ice=0;
    T_em=0;
    u=[T_ice;T_em];
else
    H=P_f+(lambda*P_ech);
    H((T_ice+(J_e*dw_ice))>T_ice_max)=inf;
    H(T_em>T_em_max | T_em<-T_em_max)=Inf;
    H(Power_electric_machine<-P_em_max*1000 | Power_electric_machine>P_em_max*1000)=Inf;
    [q,i]=min(H);
    T_ice=T_ice(i);
    T_em=T_em(i);
    u=[T_ice;T_em];
end
   


