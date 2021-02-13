% Pre-processing of vehicle data.

acc=[diff(dc.v)./diff(dc.t);0];     % acceleration
task.acc=acc;                       % temporarelly save acceleration in dc structure.

% Find time instances when the vehicle is moving. 
task.drv=dc.v > 0;

Ademt=(env.gravity*chs.cr*cos(dc.slope)+acc+env.gravity*sin(dc.slope))*chs.Rw/chs.finalgear; 
Ademt(dc.v==0)=0;                   % exclude acceleration when vehicle is standing still.
J=chs.Jw;                           % rotational inertia of ICE and transmission is neglected.
% Demanded torque between transmission and differential
Tdemt=(J*acc/chs.Rw^2+chs.Af*chs.cd*env.airdensity/2*dc.v.^2)*chs.Rw/chs.finalgear+Ademt*chs.m;    % torque demand 
Tdemt(~task.drv)=0;
task.wt=dc.v/chs.Rw*chs.finalgear;  % demanded speed between transmission and differential
task.Tdemt=Tdemt;

%% Data for the gear selection
% Rotational inertia of ICE and transmission is neglected.
task.Ng=numel(trn.gix);                 % number of gears.
Rg=repmat(trn.gearratio(:)',task.N,1);  % gear ratio combinations at each time instance.
ETAg=repmat(trn.eta(:)',task.N,1);      % gear efficiency at each time instance.
ix=repmat(acc > 0,1,task.Ng);                
ETAg(ix)=1./ETAg(ix);                   % inverted efficiency for positive demands.
task.TD=repmat(task.Tdemt,1,task.Ng).*ETAg./Rg; % demanded torque between transmission and ICE, for all gears.
task.WD=repmat(task.wt,1,task.Ng).*Rg;  % demanded speed between transmission and ICE, for all gears.
task.WICE=max(ice.wix(1),task.WD);      % engine speed cannot be lower than idling speed.
task.TICEmax=interp1(ice.wix,ice.Tmax,task.WICE); % max torque the engine can deliver, for all gears.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-11.