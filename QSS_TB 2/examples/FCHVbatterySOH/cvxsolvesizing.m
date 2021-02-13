function res=cvxsolvesizing(task)
% res=cvxsolvesizing(task) returns the optimal result, res. The
% optimization task is provided in the structure task.

bat=task.bat; chs=task.chs; fc=task.fc; dt=task.dt; 
N=task.N;                                       % total number of time samples
Wh=task.Wh; Wb=task.Wb; Wfc=task.Wfc;           % weigting coefficients
Sp=task.Sp; Se=task.Se; St=task.St; Su=task.Su; Sn=task.Sn; Su2=task.Su2; % scaling factors
dsohmax=task.dsohmax;   % maximum allowed change in SOH
Nd=sum(task.drv);       % number of time instances when the vehicle is driving
C=bat.Q/bat.a(2);       %[F] battery capacity
V0=bat.a(1);            %[V] OCV at zero charge  
n0=task.nbbase;         %base number of cells

%% CVX
cvx_clear;
cvx_precision medium;   % low accuracy seems to be sufficient as well, but improvement in speed is not high.
cvx_solver sedumi;      % in case of numerical problems, try using sdpt3.
cvx_quiet(false);
cvx_begin
    variables Eb(N+1) Pfc(N) Tem(Nd) xb xfc;
    expressions Pb(N) Ub2(N) Pbloss(N);
    % expressions hold operations over optimization variables.
    Pb = -(Eb(2:N+1)-Eb(1:N))/dt*Se/Sp;                 % internal buffer power   
    Ub2=2*Eb(1:N)/C*Se/Su2 + V0^2*n0*xb/Su2;            % square of voltage expression
    Pbloss=bat.R*quad_over_lin(Pb,Ub2,0)*Sp/Su2;        % buffer losses
    % In case of numerical problems try using the custom function quadlin:
    % Pbloss=bat.R*quadlin(Pb,Ub2)*Sp/Su2;
    
    if task.useSOHmodel
         variables sohx(N+1);
         expressions  Dsohx(N) Pbabs(N);
         Dsohx =(sohx(2:N+1)-sohx(1:N))/dt;             % SOHx derivative
         Pbabs=abs(Pb);
    else
        expressions Ub(N);
        Ub = sqrt(n0*Su2)*sqrtprod(Ub2,xb)/Su;          % voltage expression
        % In case of numerical problems try using sqrtprodgm which is
        % bassed on the CVX built in function geo_mean: Ub =
        % sqrt(n0*Su2)*sqrtprodgm(Etmp,xb)/Su;
    end 
    
    % The cost function is cost=Wh*sum(Pfuel)*dt + Wb*xb + Wfc*xfc,
    % where Pfuel=a1*xfc + (a2+1)*Pfc + a3*Pfc^2/xfc. From here, the
    % problem can be written as:
    minimize(Wb*xb + (Wh*fc.a(1)*N*dt + Wfc)*xfc ...
        + Wh*(fc.a(2)+1)*sum(Pfc)*dt*Sp ...
        + Wh*fc.a(3)*quad_over_lin(Pfc,xfc)*dt*Sp^2);
    % In case of numerical problems try using the custom function
    % quadlinsum instead of quad_over_lin.
    subject to   
    ix=task.drv;  % driving intstances.
    Pb(ix) + Pfc(ix) >= Pbloss(ix) + chs.Paux/Sp ...
           + (task.b0 + task.b1.*Tem*St + task.b2.*square(Tem)*St^2)/Sp; % electrical power balance while driving
    ix=~task.drv; % stand-still instances.
    Pb(ix) + Pfc(ix)  >= Pbloss(ix) + chs.Paux/Sp;      % electrical power balance while standing still
    Tem >= (task.T0 + task.T1*xb + task.T2*xfc)/St;     % mechanical torque balance
    Tem >= task.Tmin/St;                                % EM torque limit
    Tem <= task.Tmax/St;                                % EM torque limit
    Eb >= C/2*(task.Vmin^2-V0^2)*n0*xb/Se;              % min buffer energy
    Eb <= C/2*(task.Vmax^2-V0^2)*n0*xb/Se;              % max buffer energy
    Eb(1)==Eb(N+1);                                     % charge sustain
    if task.useSOHmodel
        for j=1:numel(bat.b)
            % piecewise affine approximation of state of health.
            Dsohx <= (bat.b{j}(1)*n0*xb + bat.b{j}(2)*Pbabs*Sp)/Sn/dsohmax;
        end
        sohx(1) == n0*xb/Sn;
        sohx(N+1) >= 0;        
        
        % actual power bounds are not needed, sice the SOH model avoids
        % operation at high power. However, nominal power bounds can be
        % considered which will not affect computaional performance
        % significantly.
        Pb >= bat.Imin*bat.Vnom*n0*xb/Sp;               % nominal buffer power limit
        Pb <= bat.Imax*bat.Vnom*n0*xb/Sp;               % nominal buffer power limit
    else
        Pb >= bat.Imin*Ub*Su/Sp;                        % buffer power limit
        Pb <= bat.Imax*Ub*Su/Sp;                        % buffer power limit
    end
    xb >= task.xbmin;                                   % buffer scale limit
    xb <= task.xbmax;                                   % buffer scale limit
    Pfc >= 0;                               
    Pfc <= fc.Pmax*xfc/Sp;                              % FCS power limit
    xfc >= task.xfcmin;                                 % FCS scale limit
    xfc <= task.xfcmax;                                 % FCS scale limit
cvx_end

%% post threat data
res.Pb=Pb*Sp; res.Pfc=Pfc*Sp; res.Eb=Eb*Se;
res.Tem=zeros(N,1); res.Tem(task.drv)=Tem*St;
res.xb=xb; res.xfc=xfc;
if task.useSOHmodel
    res.soh=sohx*Sn*dsohmax/n0/xb+(1-dsohmax);
else
    res.soh=ones(N+1,1);
end
res.cost=cvx_optval; 
res.status=cvx_status;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.
%   Modified by Nikolce Murgovski, 2014-04: Improved computational efficiency. 