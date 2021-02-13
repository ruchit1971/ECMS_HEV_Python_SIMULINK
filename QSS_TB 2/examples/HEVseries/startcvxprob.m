% Combined optimization of engine generator unit (EGU) and battery sizing,
% and power-split control of a hybrid electric vehicle in a series
% powertrain configuration. The engine on/off state is decided by
% heuristics, where the engine is turned on when demanded power of the
% baseline vehicle exceeds a certain threshold. Detailes on the convex
% modeling can be found in the following articles:
%
% [1] N. Murgovski, L. Johannesson, J. Sjöberg and B. Egardt. Component
% sizing of a plug-in hybrid electric powertrain via convex optimization.
% Journal of Mechatronics, 22(1):106-120, 2012.
% 
% [2] N. Murgovski, L. Johannesson, A. Grauers and J. Sjöberg. Dimensioning
% and control of a thermally constrained double buffer plug-in HEV
% powertrain. In 51st IEEE Conference on Decision and Control, Maui,
% Hawaii, 2012.
% 
% [3] X. Hu, N. Murgovski, L. Johannesson and B. Egardt. Energy efficiency
% analysis of a series plug-in hybrid electric bus with different energy
% management strategies and battery sizes. Applied Energy, 111:1001-1009,
% 2013.

addpath ../common;

clc;
task=struct;                % keep all data in one structure
task.nbbase=1500;           % baseline number of buffer cells
task.xbmin=0.1;             % min buffer scale
task.xbmax=2;               % max buffer scale
task.xegumin=0.1;           % min EGU scale
task.xegumax=3;             % max EGU scale 
task.socmin=0.3;            % min buffer SOC
task.socmax=0.7;            % max buffer SOC

init;                       % read remaining data
prethreatdata;              % preprocess data

res=struct; res.cost=Inf;
Pon=5e3; 
%uncomment the line below to investigate other power thresholds above which
%the engine is turned on.
% Pon=linspace(0e3,10e3,11); % loop on several power thresholds.
for j=1:numel(Pon)
    on=task.Pdem_base >= Pon(j); % engine on/off sequence

    tic;
    res1=cvxsolvesizing(task,on); T=toc;
    fprintf('%s: cost=%1.4f EUR/100km, buffer scale=%1.2f, EGU scale=%1.2f, Pon=%1.2fkW, t=%1.2f s\n', ...
        res1.status,res1.cost,res1.xb,res1.xegu,Pon(j)/1000,T);

    % Save results for the optimal power threshold. 
    if (strcmp(res1.status,'Solved') || strcmp(res1.status,'Inaccurate/Solved')) && res1.cost < res.cost
        res=res1;
        res.Pon=Pon(j);
        res.on=on;
    end
end

plotdata;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.