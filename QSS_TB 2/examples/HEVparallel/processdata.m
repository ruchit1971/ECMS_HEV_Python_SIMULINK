% Reprocess data after gear has been selected.

%% Demanded speed and torque between transmission and ICE/EM, for the selected gear.
etag=trn.eta(gear(task.drv));       % gear efficiency at driving segments.
ix=task.acc(task.drv) > 0;
etag(ix)=1./etag(ix);               % inverted efficiency for positive demands.
rg=trn.gearratio(gear(task.drv));   % gear ratio at driving segments.
task.T0=task.T0t.*etag./rg;
task.T1=task.T1t.*etag./rg;
% Demanded torque can now be expressed as Tdem=T0+T1*xb.
task.w=zeros(task.N,1);
task.w(task.drv)=task.wt(task.drv).*rg;

%% ICE related coefficients and torque limit
task.wice=max(task.w, ice.wix(1));  % engine speed cannot be lower than idling speed.
wice=task.wice(on);                 % prepare engine data only at instances where engine is on.
task.a0=interp1(ice.wix,ice.a(1,:),wice);
task.a1=interp1(ice.wix,ice.a(2,:),wice)+wice;
task.a2=interp1(ice.wix,ice.a(3,:),wice);
task.Ticemax=interp1(ice.wix,ice.Tmax,wice);
% The ICE's fuel power is computed as Pfuel=a0+a1.*Tice+a2.*Tice.^2, where
% Tice is the engine torque.

%% EM related coefficients and torque limits
wem=task.w(task.drv);
task.b0=interp1(em.wix,em.a(1,:),wem);
task.b1=interp1(em.wix,em.a(2,:),wem)+wem;
task.b2=interp1(em.wix,em.a(3,:),wem);
task.Temmin=interp1(em.wix,em.Tmin,wem);
task.Temmax=interp1(em.wix,em.Tmax,wem);
% The EM's electric power is computed as Pel=b0+b1.*Tem+b2.*Tem.^2, where Tem
% is the EM torque.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.