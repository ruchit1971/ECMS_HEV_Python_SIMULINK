% Pre-processing of vehicle data.

acc=[diff(dc.v)./diff(dc.t);0];     % acceleration
task.acc=acc;                       % temporarelly save acceleration in dc structure.
bufmassbase=bat.m*task.nbbase*1.15; % 15% additional mass for buffer packaging and circuitry.

% Find time instances when the vehicle is moving. 
task.drv=dc.v > 0;

Ademt=(env.gravity*chs.cr*cos(dc.slope)+acc+env.gravity*sin(dc.slope))*chs.Rw/chs.finalgear; 
Ademt(dc.v==0)=0;                    % exclude acceleration when vehicle is standing still.
J=chs.Jw;                           % rotational inertia of ICE, EM and transmission is neglected.
% Demanded torque between transmission and differential
Tdemt=(J*acc/chs.Rw^2+chs.Af*chs.cd*env.airdensity/2*dc.v.^2)*chs.Rw/chs.finalgear ...
    +Ademt*(chs.m + bufmassbase);    % torque demand 
Tdemt(~task.drv)=0;
task.wt=dc.v/chs.Rw*chs.finalgear;  % demanded speed between transmission and differential

% Split demadned torque in parts.
task.T0t=Tdemt(task.drv)-Ademt(task.drv)*bufmassbase;
task.T1t=Ademt(task.drv)*bufmassbase;
% Demanded torque can now be expressed as Tdemt=T0t+T1t*xb, where xb is
% buffer scale.
task.Pdemt_base=Tdemt.*task.wt;
task.Tdemt=Tdemt;

%% Data for the gear selection
% Rotational inertia of ICE, EM and transmission is neglected.
task.Ng=numel(trn.gix);                 % number of gears.
Rg=repmat(trn.gearratio(:)',task.N,1);  % gear ratio combinations at each time instance.
ETAg=repmat(trn.eta(:)',task.N,1);      % gear efficiency at each time instance.
ix=repmat(acc > 0,1,task.Ng);                
ETAg(ix)=1./ETAg(ix);                   % inverted efficiency for positive demands.
task.TD=repmat(task.Tdemt,1,task.Ng).*ETAg./Rg; % demanded torque between transmission and ICE/EM, for all gears.
task.WD=repmat(task.wt,1,task.Ng).*Rg;  % demanded speed between transmission and ICE/EM, for all gears.
task.WICE=max(ice.wix(1),task.WD);      % engine speed cannot be lower than idling speed.
task.TICEmax=interp1(ice.wix,ice.Tmax,task.WICE); % max torque the engine can deliver, for all gears.
task.TEMmax=interp1(em.wix,em.Tmax,task.WD); % max motoring torque the EM can deliver, for all gears.
task.TEMmin=interp1(em.wix,em.Tmin,task.WD); % max generating torque the EM can deliver, for all gears.

%% Scaling of optimization varaibles
% It is very important that all optimization varaibles have similar range
% of values.
task.Vmin=bat.a(1)+bat.a(2)*task.socmin;                    % minimum cell OCV
task.Vmax=bat.a(1)+bat.a(2)*task.socmax;                    % maximum cell OCV

task.Sp=max(task.Pdemt_base);                               % maximum power demand
task.St=max(max(em.Tmax),max(ice.Tmax));                    % torque scale
Sn=task.nbbase*task.xbmax;                                  % maximum number of cells per pack
task.Se=bat.Q/bat.a(2)/2*(task.Vmax^2-bat.a(1)^2)*Sn;       % maximum buffer energy
task.Su=task.Vmax*Sn;                                       % voltage scale
task.Su2=task.Vmax^2*Sn;                                    % square of voltage scale


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-03.