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

if nargin < 3
    maxupshift=-1;
end
if nargin < 4
    shiftonbrake=true;
end
ice=task.ice; 

ON=repmat(on,1,task.Ng);    % matrix of engine on/off state, for all gears.
TICEmax=task.TICEmax; TICEmax(~ON)=0; % maximum engine torque, for all gears.
OK=ice.Wmax >= task.WICE & TICEmax >= task.TD;
TICE=max(task.TD,0);        % the vehicle is driven purely by the ICE.

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
ETA=NaN(task.N,task.Ng);        % efficiency matrix for all gear choices.
ETA(OK)=interp2(ice.wix,ice.Tix,ice.eta,task.WICE(OK),TICE(OK)); 
% choose gear that maximizes engine efficiency.
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
%   Created by Nikolce Murgovski, 2015-11.