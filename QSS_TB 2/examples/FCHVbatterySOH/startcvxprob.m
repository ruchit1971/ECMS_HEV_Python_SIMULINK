% Combined optimization of fuel cell system and battery sizing, and
% power-split control of a fuel cell hybrid vehicle, subject to a battery
% state of health (SOH) model. The battery health is a c-rate-weighted
% Ah-throughput model. Further detailes on the convex modeling of fuel cell
% and buffer sizing can be found in the example FCHV. Further detailes on
% the convex modeling of the battery SOH model can be found in the
% following articles:
%
% [1] X. Hu, L. Johannesson, N. Murgovski, and B. Egardt.
% Longevity-conscious dimensioning and power management of the hybrid
% energy storage system for a fuel cell hybrid electric bus. Applied
% Energy, 2014. DOI: 10.1016/j.apenergy.2014.05.013.
%
% [2] L. Johannesson, N. Murgovski, S. Ebbessen, B. Egardt, E. Gelso, and
% J. Hellgren. Including a battery state of health model in the HEV
% component sizing and optimal control problem. In IFAC Symposium on
% Advances in Automotive Control, Tokyo, Japan, 2013.
%
% Note that due to consistency with the existing vehlib library, and for
% the sake of readability, some data in this example may differ from the
% data used in the papers. (The same vehicle model is used as in example
% FCHV.) In addition, this examples also sizes the fuel cell system and
% approximates the SOH derivative with a piecewise affine model.

addpath ../common;

clc;
task=struct;                % keep all data in one structure
task.nbbase=1800;           % baseline number of buffer cells
task.xfcmin=0.5;            % min fuel cell scale
task.xfcmax=3.5;            % max fuel cell scale 
task.xbmin=0.01;            % min buffer scale
task.xbmax=2;               % max buffer scale
task.socmin=0.3;            % min buffer SOC
task.socmax=0.7;            % max buffer SOC
task.batrep=1;              % number of battery replacements within the bus' service period
task.useSOHmodel=true;      % if false, the SOH model is neglected. Make sure to put task.batrep=0, as there is no reason to replace the battery when SOH model is neglected.

init;                       % read remaining data
prethreatdata;              % preprocess data
tic;
res=cvxsolvesizing(task); T=toc;
fprintf('%s: cost=%1.2f EUR/100km, fuel cell scale=%1.2f, buffer scale=%1.2f, SOHfinal=%1.4f %%, t=%1.2f s\n', ...
    res.status,res.cost,res.xfc,res.xb,res.soh(end)*100,T);

plotdata;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-02.