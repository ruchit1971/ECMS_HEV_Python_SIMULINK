function res=cvxsolvesizing(task,on)
% res=cvxsolvesizing(task,on) returns the optimal result, res. The
% optimization task is provided by the structure task. The engine on/off
% sequence is provided by on.

bat=task.bat; chs=task.chs; egu=task.egu; dt=task.dt; 
N=task.N;               % total number of time samples
Non=sum(on);            % number of time instances when the engine is on
Nd=sum(task.drv);       % number of time instances when the vehicle is driving
Wf=task.Wf; Wb=task.Wb; Wegu=task.Wegu;         % weigting coefficients
Sp=task.Sp; Se=task.Se; St=task.St; Su=task.Su; Su2=task.Su2; % scaling factors
C=bat.Q/bat.a(2);       %[F] battery capacity
V0=bat.a(1);            %[V] OCV at zero charge  
n0=task.nbbase;         %base number of cells

%% CVX
cvx_precision medium;   % low accuracy seems to be sufficient as well, but improvement in speed is not high.
cvx_solver sedumi;      % in case of numerical problems, try using sdpt3.
cvx_quiet(false);
cvx_begin
    variables Eb(N+1) Pg(N) Tem(Nd) xegu xb;
    expressions Pb(N) Pbloss(N) Ub(N) Ub2(N);
    
    % expressions hold operations over optimization variables.
    Pb = -(Eb(2:N+1)-Eb(1:N))/dt*Se/Sp;                 % internal buffer power
    Ub2=2*Eb(1:N)/C*Se/Su2 + V0^2*n0*xb/Su2;            % square of voltage expression
    Ub = sqrt(n0*Su2)*sqrtprod(Ub2,xb)/Su;              % voltage expression
    Pbloss=bat.R*quad_over_lin(Pb,Ub2,0)*Sp/Su2;        % buffer losses
    % In case of numerical problems try using the custom function quadlin:
    % Pbloss=bat.R*quadlin(Pb,Ub2)*Sp/Su2;

    % The cost function is cost=Wf*sum(Pfuel)*dt + Wb*xb +Wegu*xegu,
    % where Pfuel=a1*xegu + (a2+1)*Pg + a3*Pg^2/xegu. From here, the
    % problem can be written as:
    minimize(Wb*xb + (Wf*egu.a(1)*Non*dt + Wegu)*xegu ...
        + Wf*(egu.a(2)+1)*sum(Pg(on))*dt*Sp ...
        + Wf*egu.a(3)*quad_over_lin(Pg(on),xegu)*dt*Sp^2);
    % In case of numerical problems try using the custom function
    % quadlinsum instead of quad_over_lin.
    
    subject to
    % Electrical power balance.
    ix=task.drv;  % driving instances.
    Pb(ix) + Pg(ix) >= Pbloss(ix) + chs.Paux/Sp ...
               + (task.b0 + task.b1.*Tem*St + task.b2.*square(Tem)*St^2)/Sp; 
    ix=~task.drv; % stand-still instances.
    Pb(ix) + Pg(ix)  >= Pbloss(ix) + chs.Paux/Sp; 
    Tem >= (task.T0 + task.T1*xb + task.T2*xegu)/St;    % mechanical torque balance
    Tem >= task.Tmin/St;                                % EM torque limit
    Tem <= task.Tmax/St;                                % EM torque limit
    Eb >= C/2*(task.Vmin^2-V0^2)*n0*xb/Se;              % min buffer energy
    Eb <= C/2*(task.Vmax^2-V0^2)*n0*xb/Se;              % max buffer energy
    Eb(1)==Eb(N+1);                                     % charge sustain
    Pb >= bat.Imin*Ub*Su/Sp;                            % buffer power limit
    Pb <= bat.Imax*Ub*Su/Sp;                            % buffer power limit
    xb >= task.xbmin;                                   % buffer scale limit
    xb <= task.xbmax;                                   % buffer scale limit
    Pg >= 0;                               
    Pg <= egu.Pmax*xegu*on/Sp;                          % FCS power limit. 
    % The upper limit on Pg forces the EGU power to be zero when the engine
    % is off. One could slightly speed up the optimization if the vector Pg
    % consists only of instaces where engine is on. This would require
    % additional two cases in the electric power balance equation above.
    xegu >= task.xegumin;                               % FCS scale limit
    xegu <= task.xegumax;                               % FCS scale limit
cvx_end

%% post threat data
res.Pb=Pb*Sp; res.Eb=Eb*Se; res.Pg=Pg*Sp;
res.Tem=zeros(N,1); res.Tem(task.drv)=Tem*St;
res.xb=xb; res.xegu=xegu;
res.cost=cvx_optval; 
res.status=cvx_status;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.