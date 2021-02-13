% Plot optimization data.

addpath ../common/plotfcn;
plotdc=true;
plotice=true;
plotgeardist=true;


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

%% Gear distribution
if plotgeardist
    figure;
    % Gear distribution for all time instances.
    nel=histc(res.gear(res.gear>0),1:task.Ng);
    bar(1:task.Ng,nel,'b'); hold on;   
    xlim([0.5 task.Ng+0.5]);
    xlabel('Gears'); ylabel('Distribution');
    title('Gear distribution');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2015-11.