% Combined optimization of battery sizing and power-split control of a
% hybrid electric vehicle in a parallel powertrain configuration. The
% engine on/off state and gear decisions are decided by heuristics.
% Detailes on the convex modeling can be found in the following articles:
%
% [1] N. Murgovski, L. Johannesson, J. Sj�berg and B. Egardt. Component
% sizing of a plug-in hybrid electric powertrain via convex optimization.
% Mechatronics, 22(1):106-120, 2012.

% [2] M. Pourabdollah, cN. Murgovski, A. Grauers and B. Egardt. Optimal
% sizing of a parallel PHEV powertrain. IEEE Transactions on Vehicular
% Technology, 62(6):2469-2480, 2013.

addpath ../common;

clc;
task=struct;                % keep all data in one structure
task.nbbase=800;            % baseline number of buffer cells
task.xbmin=0.1;             % min buffer scale
task.xbmax=2;               % max buffer scale
task.socmin=0.3;            % min buffer SOC
task.socmax=0.7;            % max buffer SOC
task.maxupshift=-1;         % max gear upshifts (activated when value is positive)
task.shiftonbrake=true;     % allow shifting gears while braking

init;                       % read remaining data
prethreatdata;              % preprocess data

res=struct; res.cost=Inf;
Pon=33e3; 
% Uncomment the line below to investigate other power thresholds above which
% the engine is turned on.
% Pon=linspace(30e3,40e3,11); 
for j=1:numel(Pon)
    on=(task.Pdemt_base >= Pon(j)) & task.drv; % engine on/off sequence
    fprintf('Selecting gear...');
    [gear,on]=selectgear(task,on,task.maxupshift,task.shiftonbrake);
    processdata;
    fprintf('done. CVX running...\n');
    tic;
    res1=cvxsolvesizing(task,on); T=toc;
    fprintf('%s: cost=%1.4f EUR/100km, buffer scale=%1.2f, Pon=%1.2fkW, t=%1.2f s\n', ...
        res1.status,res1.cost,res1.xb,Pon(j)/1000,T);

    % Save results for the optimal power threshold. 
    if (strcmp(res1.status,'Solved') || strcmp(res1.status,'Inaccurate/Solved')) && res1.cost < res.cost
        res=res1;
        res.Pon=Pon(j);
        res.on=on; res.gear=gear; res.wem=task.w; res.wice=task.wice;
    end
end

plotdata;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.