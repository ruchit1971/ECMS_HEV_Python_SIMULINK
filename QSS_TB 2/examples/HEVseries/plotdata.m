% Plot optimization data.

addpath ../common/plotfcn;
plotdc=true;
plotegu=true;
plotem=true;
plotbuffer=true;
plotbuffersoc=true;

%% plot driving cycle
if plotdc
    figure;
    plot_dc(dc,62); % initial altitude is 62m.
end

%% Engine generator unit
if plotegu
    figure;
    plot_egu(egu, res.xegu); hold on;

    % plot operating points
    Pegub=res.Pg/res.xegu; % baseline EGU power
    Pfuelb=[ones(numel(Pegub),1),Pegub,Pegub.^2]*(egu.a+[0;1;0]);   % baseline fuel power
    eta=Pegub./Pfuelb;                                              % efficiency
    h=plot(res.Pg/1000,eta*100,'r*');
    legend(h,'Operating points','Location','SouthEast');
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
    h=plot_battery(bat,n); hold on;

    % Plot operating points
    Vcell=sqrt(2*res.Eb(1:end-1)*bat.a(2)/n/bat.Q+bat.a(1)^2);
    soc=(Vcell-bat.a(1))/bat.a(2);
    Pbt=res.Pb-bat.R*res.Pb.^2/n./Vcell.^2; % power at pack terminals
    h(end+1)=plot(Pbt/1000,soc*100,'r*','DisplayName','Operating points');
    legend(h);
end

%% Electric buffer state of charge
if plotbuffersoc
    figure; 
    hold on; clear h;
    n=task.nbbase*res.xb;       % scale
    Vcell=sqrt(2*res.Eb(1:end-1)*bat.a(2)/n/bat.Q+bat.a(1)^2);
    soc=(Vcell-bat.a(1))/bat.a(2);

    % plot SOC limits.
    h(1)=plot([0 dc.t(end)/60],[1 1]*task.socmin*100,'r-.');
    plot([0 dc.t(end)/60],[1 1]*task.socmax*100,'r-.');
    % plot optimal SOC trajectory
    h(2)=plot(dc.t/60,soc*100,'k');
    xlim([0 dc.t(end)/60]); ylim([0 101]);
    xlabel('Time [min]'); ylabel('Optimal SOC trajectory [%]');
    legend(h,'SOC limits','State of charge');
    title('Battery SOC trajectory');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-04.