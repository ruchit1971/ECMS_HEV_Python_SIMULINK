% Reprocess data after gear has been selected.

%% Demanded speed and torque between transmission and ICE/EM, for the selected gear.
etag=trn.eta(gear(task.drv));       % gear efficiency at driving segments.
ix=task.acc(task.drv) > 0;
etag(ix)=1./etag(ix);               % inverted efficiency for positive demands.
rg=trn.gearratio(gear(task.drv));   % gear ratio at driving segments.
task.w=zeros(task.N,1);
task.w(task.drv)=task.wt(task.drv).*rg;

%% ICE related coefficients and torque limit
res.wice=max(task.w, ice.wix(1));  % engine speed cannot be lower than idling speed.
res.Tice=zeros(task.N,1);
res.Tice(task.drv)=max(0, task.Tdemt(task.drv).*etag./rg);
% obtain the chemical fuel power
Pf=res.wice.*res.Tice + interp2(ice.wix,ice.Tix,ice.Ploss,res.wice,res.Tice);
% compute fuel consumption in EUR/100km
res.cost=task.Wf*sum(Pf)*task.dt;     


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-11.