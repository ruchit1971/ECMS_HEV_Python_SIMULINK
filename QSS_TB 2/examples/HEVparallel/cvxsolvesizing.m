function res=cvxsolvesizing(task,on)
% res=cvxsolvesizing(task,on) returns the optimal result, res. The
% optimization task is provided in the structure task. The engine on/off
% sequence is provided by on.

bat=task.bat; chs=task.chs; dt=task.dt; 
N=task.N;               % total number of time samples
Non=sum(on);            % number of time instances when the engine is on
Nd=sum(task.drv);       % number of time instances when the vehicle is driving
Wf=task.Wf; Wb=task.Wb; % weigting coefficients
Sp=task.Sp; Se=task.Se; St=task.St; Su=task.Su; Su2=task.Su2; % scaling factors
C=bat.Q/bat.a(2);       %[F] battery capacity
V0=bat.a(1);            %[V] OCV at zero charge  
n0=task.nbbase;         %base number of cells

%% CVX
cvx_clear;
cvx_precision medium;   % low accuracy seems to be sufficient as well, but improvement in speed is not high.
cvx_solver sedumi;      % in case of numerical problems, try using sdpt3.
cvx_quiet(false);
cvx_begin
    variables Eb(N+1) Tice(Non) Tem(Nd) xb;
    expressions Pb(N) Pbloss(N) Ub(N) Ub2(N);
    
    % expressions hold operations over optimization variables.
    Pb = -(Eb(2:N+1)-Eb(1:N))/dt*Se/Sp;                 % internal buffer power
    Ub2=2*Eb(1:N)/C*Se/Su2 + V0^2*n0*xb/Su2;            % square of voltage expression
    Ub = sqrt(n0*Su2)*sqrtprod(Ub2,xb)/Su;              % voltage expression
    Pbloss=bat.R*quad_over_lin(Pb,Ub2,0)*Sp/Su2;        % buffer losses
    % In case of numerical problems try using the custom function quadlin:
    % Pbloss=bat.R*quadlin(Pb,Ub2)*Sp/Su2;

    % The cost function is cost=Wf*sum(Pfuel)*dt + Wb*xb), where Pfuel=a0 +
    % a1*Tice + a2*Tice^2. The idling losses, a0, do not affect the optimal
    % values and can be removed from the objective. Instead, these losses
    % are added after the optimization is finished.
    minimize(Wf*sum(task.a1.*Tice*St + task.a2.*square(Tice)*St^2)*dt + Wb*xb);
    subject to
    % Electrical power balance:
    ix=task.drv;  % driving instances.
    Pb(ix) >= Pbloss(ix) + chs.Paux/Sp ...
               + (task.b0 + task.b1.*Tem*St + task.b2.*square(Tem)*St^2)/Sp; 
    ix=~task.drv; % stand-still instances.
    Pb(ix) >= Pbloss(ix) + chs.Paux/Sp; 
    % Mechanical torque balance:
    ix=on(task.drv); % driving instances where engine is on.
    Tem(ix) + Tice >= (task.T0(ix) + task.T1(ix)*xb)/St;
    ix=~on(task.drv); % driving instances where engine is off.
    Tem(ix) >= (task.T0(ix) + task.T1(ix)*xb)/St;
    % Remaining constraints:
    Tice >= 0;                                          % ICE torque limit
    Tice <= task.Ticemax/St;                           % ICE torque limit
    Tem >= task.Temmin/St;                              % EM torque limit
    Tem <= task.Temmax/St;                              % EM torque limit
    Eb >= C/2*(task.Vmin^2-V0^2)*n0*xb/Se;              % min buffer energy
    Eb <= C/2*(task.Vmax^2-V0^2)*n0*xb/Se;              % max buffer energy
    Eb(1)==Eb(N+1);                                     % charge sustain
    Pb >= bat.Imin*Ub*Su/Sp;                            % buffer power limit
    Pb <= bat.Imax*Ub*Su/Sp;                            % buffer power limit
    xb >= task.xbmin;                                   % buffer scale limit
    xb <= task.xbmax;                                   % buffer scale limit
cvx_end

%% post threat data
res.Pb=Pb*Sp; res.Eb=Eb*Se; 
res.Tem=zeros(N,1); res.Tem(task.drv)=Tem*St;
res.Tice=zeros(N,1); res.Tice(on)=Tice*St;
res.xb=xb; 
res.cost=cvx_optval + Wf*sum(task.a0)*dt; 
res.status=cvx_status;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.