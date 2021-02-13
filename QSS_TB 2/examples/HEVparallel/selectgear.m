function [gear,on]=selectgear(task,on,maxupshift,shiftonbrake)
% Heuristic gear selection.
%   [gear,on]=selectgear(task,on,maxupshift,shiftonbrake), selects a
%   feasible gear which has the potential to optimize engine efficiency.
%   task is the optimization task, and on is the engine on/off state at
%   each time instant. maxupshift, when set to a positive value, limits the
%   number of gear upshifts, unless feasibility is affected. If
%   shiftonbrake is false, gear shifting is prevented while braking, unless
%   feasibility is affected. The gear selection may also turn the engine on
%   if this could avoid infesibility.

NTice=50; % number of grid points for the set of feasible engine torques.
if nargin < 3
    maxupshift=-1;
end
if nargin < 4
    shiftonbrake=true;
end
ice=task.ice; em=task.em;

ON=repmat(on,1,task.Ng);    % matrix of engine on/off state, for all gears.
TICEmax=task.TICEmax; TICEmax(~ON)=0; % maximum engine torque, for all gears.
TMAX=(TICEmax + task.TEMmax); % maximum torque that can be delivered by the ICE and EM.

OK=ice.Wmax >= task.WICE & TMAX >= task.TD;

%% Feasibility check
% There should be at least one feasible gear at each time instance. If
% infeasibility is detected when engine is off, check if it can be avoided
% by turning the engine on.
oklin=any(OK,2);
if ~all(oklin) && any(~ON(~oklin,1))
    fprintf('The engine has been turned on in additional %d time instances.\n',sum(~oklin));
    on(~oklin)=true;
    [gear,on]=selectgear(task,on,maxupshift,shiftonbrake);
    return;
elseif ~all(oklin)
    % The problem is infeasible. Issue a warning.
    ix=1:task.N; ix=ix(~oklin);
    if numel(ix) < 20
        warning(['The problem is infeasible at time intance: ',num2str(ix)]);
    else
        warning('The problem is infeasible at %d time instances.',numel(ix));
    end
end

%% Guess the optimal gear selection
% Get the interval of feasible engine torque combinations.
TICEminfeas=max(0, task.TD - task.TEMmax);
TICEmaxfeas=min(TICEmax, (task.TD-task.TEMmin));
TICE=zeros(task.N,task.Ng);     % best engine torque, for all gears. This is only a temporal guess. The optimization will later find the optimal engine torque.
ETA=NaN(task.N,task.Ng);        % efficiency matrix for all gear choices.
% When engine is on, ETA will hold optimal engine efficiency, for all
% gears. It is possible to improve the heuristics by adding the influence
% of the EM on the overall efficiency. The feasible ICE torque is gridded
% at NTice uiformly spaced points, and the peak engine efficiency is
% selected from all the feasible points.
for i=1:task.N
    for j=1:task.Ng
        if ON(i,j) && OK(i,j)
            Ticegrid=linspace(TICEminfeas(i,j), TICEmaxfeas(i,j),NTice)';
            etaice=interp2(ice.wix,ice.Tix,ice.eta,task.WICE(i,j)*ones(NTice,1),Ticegrid);
            [ETA(i,j),ix]=max(etaice);
            TICE(i,j)=Ticegrid(ix);
        end
    end
end

TEM=max(task.TD-TICE, task.TEMmin); % best EM torque, for all gears. This is only a temporal guess. The optimization will later find the optimal EM torque.
%When the engine is not on, the vehicle is driven by the EM. Maximize EM
%efficiency when EM is in motoring mode.
ix=~ON & TEM >= 0;
ETA(ix)=interp2(em.wix,em.Tix,em.eta,task.WD(ix),TEM(ix));
%Maximize recuperated energy, when EM is in generating mode.
ix=~ON & TEM < 0;
PEMgen=task.WD(ix).*TEM(ix).*interp2(em.wix,em.Tix,em.eta,task.WD(ix),TEM(ix));
ETA(ix)=PEMgen/min(PEMgen(:));

% choose gear that maximizes efficiency or recuperated energy.
[~,gear]=max(ETA,[],2);
gear=gear(:);
gear(~task.drv)=0; % open the clutch when speed is zero.

%% Limit upshifts to not more than maxupshift, unless feasibility is affected.
if maxupshift > 0 
    for i=2:task.N
        if (gear(i)-gear(i-1) > maxupshift)
            gear(i)=find(OK(i,gear(i-1)+maxupshift:end),1,'first')+gear(i-1)+maxupshift-1;
        end
    end
end

%% Prevent gear shifting while braking, unless feasibility is affected.
if ~shiftonbrake
    for i=2:task.N
        if (task.TD(i,1) < 0) && gear(i-1) > 0 && OK(i,gear(i-1))
            gear(i)=gear(i-1);
        end
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.