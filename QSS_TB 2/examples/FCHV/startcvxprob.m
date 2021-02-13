% Combined optimization of fuel cell system and supercapacitor sizing and
% power-split control of a fuel cell hybrid vehicle. Detailes on the convex
% modeling can be found in the following article:
%
% [1] N. Murgovski, X. Hu, L. Johannesson, and B. Egardt. Combined design
% and control optimization of hybrid vehicles. In A. Conejo and J. Yan,
% editors, Handbook of Clean Energy Systems. Wiley, 2014. Accepted for
% publication.

addpath ../common;

clc;
task=struct;                % keep all data in one structure
task.nbbase=300;            % baseline number of buffer cells
task.xbmin=0.1;             % min buffer scale
task.xbmax=2;               % max buffer scale
task.xfcmin=0.5;            % min fuel cell scale
task.xfcmax=3;              % max fuel cell scale 

init;                       % read remaining data
prethreatdata;              % preprocess data
tic;
%cvx_solver sedumi;
res=cvxsolvesizing(task); T=toc;
fprintf('%s: cost=%1.2f EUR/100km, buffer scale=%1.2f, fuel cell scale=%1.2f, t=%1.2f s\n', ...
    res.status,res.cost,res.xb,res.xfc,T);

plotdata;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.