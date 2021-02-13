% Control of a conventional vehicle. The engine is always on, while gear is
% decided by heuristics. Since there is no control freedom, optimization is
% not needed in this scenario.

addpath ../common;

clc;
task=struct;                % keep all data in one structure
task.maxupshift=-1;         % max gear upshifts (activated when value is positive)
task.shiftonbrake=true;     % allow shifting gears while braking

init;                       % read remaining data
prethreatdata;              % preprocess data

res=struct; 
on=true(task.N, 1);         % the engine is always on 
[gear,on]=selectgear(task,on,task.maxupshift,task.shiftonbrake);
res.on=on; res.gear=gear; 
processdata;

fprintf('%s: cost=%1.4f EUR/100km, fuel consumption=%1.4f l/100km\n', ... 
    'Solved',res.cost,res.cost/fuelprice);

plotdata;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-11.