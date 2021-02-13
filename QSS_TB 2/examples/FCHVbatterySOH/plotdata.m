% Plot optimization data.

addpath ../common/plotfcn;
plotdc=true;
plotfuelcell=true;
plotem=true;
plotbuffer=true;
plotbufferSOC=true;
plotbufferSOH=true;
plotsohderivative=true;

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
    h=plot(res.Pfc/1000,eta*100,'r*');
    legend(h,'Operating points');
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
if plotbufferSOC
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

%% Electric buffer state of health
if plotbufferSOH && task.useSOHmodel
    figure; 
    hold on; clear h;
    n=task.nbbase*res.xb;       % scale
    Vcell=sqrt(2*res.Eb(1:end-1)*bat.a(2)/n/bat.Q+bat.a(1)^2);
    soc=(Vcell-bat.a(1))/bat.a(2);
    % plot optimal SOH trajectory
    plot(dc.t/60,res.soh(1:end-1)*100,'k');
    xlim([0 dc.t(end)/60]); ylim([1-task.dsohmax-1e-6, 1+1e-6]*100);
    xlabel('Time [min]'); ylabel('Optimal SOH trajectory [%]');
    title('Battery SOH trajectory');
end

%% Plot battery SOH derivative vs. power
if plotsohderivative && task.useSOHmodel
    figure; hold on; clear h;
    n=task.nbbase*res.xb;       % scale
    % approximated SOH model
    Pix=linspace(0,max(bat.Pix),30)';
    dsohapp=Inf(size(Pix));
    for j=1:numel(bat.b)
        dsohapp=min(dsohapp,bat.b{j}(1)+bat.b{j}(2)*Pix);
    end
    h(1)=plot(Pix*n/1000,interp1(bat.Pix,bat.dsoh,Pix),'k');
    h(2)=plot(Pix*n/1000,dsohapp,'b--');
    % Plot operating points
    h(3)=plot(abs(res.Pb)/1000,diff(res.soh)/task.dt,'r*');
    xlabel('Internal pack power [kW]'); 
    ylabel('State of health derivative [1/s]');
    legend(h,'Original SOH model','Approximated SOH model','Operating points');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-02.