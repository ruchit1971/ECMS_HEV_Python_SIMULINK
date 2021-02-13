function res=cvxsolvesizing(task)
% res=cvxsolvesizing(task) returns the optimal result, res. The
% optimization task is provided in the structure task.

cap=task.cap; chs=task.chs; fc=task.fc; dt=task.dt; 
N=task.N;                                       % total number of time samples
Wh=task.Wh; Wb=task.Wb; Wfc=task.Wfc;           % weigting coefficients
Sp=task.Sp; Se=task.Se; St=task.St; Su=task.Su; % scaling factors
Nd=sum(task.drv); % number of time instances when the vehicle is driving

%% CVX
cvx_precision medium;   % low accuracy seems to be sufficient as well, but improvement in speed is not high.
cvx_solver sedumi;      % in case of numerical problems, try using sdpt3. sedumi
cvx_quiet(false);
cvx_begin
    variables Eb(N+1) Pfc(N) Tem(Nd) xfc xb;
    expressions Pb(N) Pbloss(N) Ub(N);
    
    % expressions hold operations over optimization variables.
    Pb = -(Eb(2:N+1)-Eb(1:N))/dt*Se/Sp;                             % internal buffer power
    Pbloss=quad_over_lin(sqrt(Sp/Se*cap.R*cap.C/2)*Pb,Eb(1:N),0);   % buffer losses
    Ub = sqrt(2*task.nbbase/cap.C*Se)/Su*sqrtprod(Eb(1:N),xb);      % buffer open circuit voltage
    % In case of numerical problems try using the custom function quadlin:
    % Pbloss=quadlin(sqrt(Sp/Se*cap.R*cap.C/2)*Pb,Eb(1:N));

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
    Eb >= 0; 
    Eb <= cap.C*cap.Vmax^2/2*task.nbbase*xb/Se;         % max buffer energy
    Eb(1)==Eb(N+1);                                     % charge sustain
    Pb >= cap.Imin*Ub*Su/Sp;                            % buffer power limit
    Pb <= cap.Imax*Ub*Su/Sp;                            % buffer power limit
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
res.cost=cvx_optval; 
res.status=cvx_status;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.