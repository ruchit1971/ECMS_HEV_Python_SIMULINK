% Pre-processing of vehicle data.

acc=[diff(dc.v)./diff(dc.t);0];     % acceleration
bufmassbase=bat.m*task.nbbase*1.15; % 15% additional mass for buffer packaging and circuitry.

% Find time instance when the vehicle is moving. 
task.drv=dc.v > 0;

Adem=(env.gravity*chs.cr*cos(dc.slope)+acc+env.gravity*sin(dc.slope))*chs.Rw/chs.finalgear; 
Adem(dc.v==0)=0;                    % exclude acceleration when vehicle is standing still
J=chs.Jw + em.J*chs.finalgear^2;    % rotational inertia reflected at the wheels
% Demanded torque between EM and final gear
Tdem=(J*acc/chs.Rw^2+chs.Af*chs.cd*env.airdensity/2*dc.v.^2)*chs.Rw/chs.finalgear ...
    +Adem*(chs.m + bufmassbase + egu.m); % torque demand 
Tdem(~task.drv)=0;
task.w=dc.v/chs.Rw*chs.finalgear;   % demanded speed 

% Split demadned torque in parts.
task.T0=Tdem(task.drv)-Adem(task.drv)*(bufmassbase + egu.m);
task.T1=Adem(task.drv)*bufmassbase;
task.T2=Adem(task.drv)*egu.m;
% Demanded torque can now be expressed as Tdem=T0+T1*xb+T2*xegu, where xb
% and xegu are scales for buffer and EGU.
task.Pdem_base=Tdem.*task.w;

%% EM related coefficients and torque limits
wem=task.w(task.drv);
task.b0=interp1(em.wix,em.a(1,:),wem);
task.b1=interp1(em.wix,em.a(2,:),wem)+wem;
task.b2=interp1(em.wix,em.a(3,:),wem);
task.Tmin=interp1(em.wix,em.Tmin,wem);
task.Tmax=interp1(em.wix,em.Tmax,wem);
% The EM electric power is computed as Pel=b0+b1.*Tem+b2.*Tem.^2, where Tem
% is the EM torque.

%% Scaling of optimization varaibles
% It is very important that all optimization varaibles have similar range
% of values.
task.Vmin=bat.a(1)+bat.a(2)*task.socmin;                    % minimum cell OCV
task.Vmax=bat.a(1)+bat.a(2)*task.socmax;                    % maximum cell OCV

task.Sp=max(Tdem.*task.w);                                  % maximum power demand
task.St=max(max(em.Tmax),max(-em.Tmin));                    % maximum torque demanded by the EM
task.Sn=task.nbbase*task.xbmax;                             % maximum number of cells per pack
task.Se=bat.Q/bat.a(2)/2*(task.Vmax^2-bat.a(1)^2)*task.Sn;  % maximum buffer energy
task.Su=task.Vmax*task.Sn;                                  % voltage scale
task.Su2=task.Vmax^2*task.Sn;                               % square of voltage scale



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.