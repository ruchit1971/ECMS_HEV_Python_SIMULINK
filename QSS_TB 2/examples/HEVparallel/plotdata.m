% Plot optimization data.

addpath ../common/plotfcn;
plotdc=true;
plotice=true;
plotem=true;
plotgeardist=true;
plotbuffer=true;
plotbuffersoc=true;

%% plot driving cycle
if plotdc
    figure;
    plot_dc(dc,62); % initial altitude is 62m.
end

%% Internal combustion engine
if plotice
    figure;
    h=plot_ice(ice); hold on;

    % plot operating points
    h(end+1)=plot(res.wice(res.on)*30/pi,res.Tice(res.on),'r*','DisplayName','Operating points');
    legend(h);
end

%% Electric machine
if plotem
    figure;
    h=plot_em(em); hold on;

    % plot operating points
    h(end+1)=plot(res.wem*30/pi,res.Tem,'r*','DisplayName','Operating points');
    legend(h);
end

%% Gear distribution
if plotgeardist
    figure;
    % Gear distribution for all time instances.
    nel=histc(gear(res.gear>0),1:task.Ng);
    bar(1:task.Ng,nel,'b'); hold on;   
    % Gear distribution time instances with engine on.
    nel=histc(gear(res.on),1:task.Ng);
    bar(1:task.Ng,nel,'r');
    xlim([0.5 task.Ng+0.5]);
    xlabel('Gears'); ylabel('Distribution');
    title('Gear distribution');
    legend('All time instances','Time instances with engine on', ...
        'Location','NorthWest');
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