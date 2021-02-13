% Plot optimization data.

addpath ../common/plotfcn;
plotdc=true;
plotfuelcell=true;
plotem=true;
plotbuffer=true;
plotbuffersoc=true;

%% plot driving cycle
if plotdc
    figure;
    plot_dc(dc,62); % initial altitude is 62m.
end

%% Fuel cell system
if plotfuelcell
    figure;
    plot_fuelcell(fc, res.xfc); hold on;

    % plot operating points
    Pfcb=res.Pfc/res.xfc; % baseline FCS power
    Pfuelb=[ones(numel(Pfcb),1),Pfcb,Pfcb.^2]*(fc.a+[0;1;0]);   % baseline fuel power
    eta=Pfcb./Pfuelb;                                           % efficiency
    h=plot(res.Pfc/1000,eta*100,'r*','DisplayName','Operating points');
    legend(h);
end

%% Electric machine
if plotem
    figure;
    h=plot_em(em); hold on;

    % plot operating points
    h(end+1)=plot(task.w*30/pi,res.Tem,'r*','DisplayName','Operating points');
    legend(h);
end

%% Electric buffer
if plotbuffer
    figure;
    n=task.nbbase*res.xb;       % scale
    h=plot_capacitor(cap,n); hold on;

    % Plot operating points
    soc=sqrt(2*res.Eb(1:end-1)/n/cap.C)/cap.Vmax;
    Pbt=res.Pb-cap.C*cap.R*res.Pb.^2/2./res.Eb(1:end-1); % power at pack terminals
    h(end+1)=plot(Pbt/1000,soc*100,'r*','DisplayName','Operating points');
    legend(h);
end

%% Electric buffer state of charge
if plotbuffersoc
    figure; hold on; clear h;
    n=task.nbbase*res.xb;       % scale
    soc=sqrt(2*res.Eb(1:end-1)/n/cap.C)/cap.Vmax;

    % plot SOC limits.
    h(1)=plot([0 dc.t(end)/60],[0 0],'r-.');
    plot([0 dc.t(end)/60],[1 1]*100,'r-.');
    % plot optimal SOC trajectory
    h(2)=plot(dc.t/60,soc*100,'k');
    xlim([0 dc.t(end)/60]); ylim([0 101]);
    xlabel('Time [min]'); ylabel('State of charge [%]');
    legend(h,'SOC limits','Optimal SOC trajectory');
    title('Supercapacitor SOC trajectory');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.