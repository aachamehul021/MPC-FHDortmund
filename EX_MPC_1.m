clc
clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: Open loop system
% In this script we compare the optimal setting
% of K based on the initial state with a regular
% "re-setting" of K 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 1; % Time step to update optimal controler
fak = 0; % This is factor to weight y_1 and y_2 in cost function

OverallSimulationTime = 10;
myt = [0:dt:OverallSimulationTime-dt];

% Starting value
x01 = 1;
x02 = 1;
k1 = 1;

syms s
syms t
tmps = [];
for k = 1:length(myt)
    t0 = myt(k);  
    k2 = sqrt(2*x02^2/x01^2+2);
     
   %k2 = 2;
    % Calculate the state using two approaches:
    % a) Inverse Laplace
    % b) Using function "initial"
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tvec = [0:0.01:OverallSimulationTime-t0];
    f0 = ilaplace((x02+s*x01+x01*k2)/(s^2+k2*s+k1));
    y = double(subs(f0,t,tvec));
    subplot(1,2,1)
    plot(tvec+t0,y,'red');
    hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    H = [0 1;-k1 -k2];
    C = [1 0];
    sys = ss(H,[],C,[]);
    x0 = [x01 x02];
    [y1,tvec,state] = initial(sys,x0,tvec);
    if k == 1
        fprintf('--------------------------------------------\n');
        fprintf('Starting simulation\n');
        fprintf('Cost function: %f\n',sum(state(:,1).*state(:,1))*(1-fak)+fak*sum(state(:,2).*state(:,2)));
        fprintf('Initial value for k_2:%f\n',k2);
    
        plot(tvec+t0,y1,'green','LineWidth',2);
        xlabel('t');
        ylabel('y_1');
        drawnow
        subplot(1,2,2)
        plot(tvec+t0,state(:,2),'green','LineWidth',2);
        hold on
        stateVec = state;
        timeVec = tvec;
    else
        plot(tvec+t0,state(:,1),'green');
        plot(tvec(1)+t0,state(1,1),'.','MarkerSize',10);
        
        subplot(1,2,2)
        plot(tvec+t0,state(:,2),'green');
        
        plot(tvec(1)+t0,state(1,2),'.','MarkerSize',10);
        xlabel('t');
        ylabel('y_1');
        drawnow
        L = length(state);
        stateVec(end-L+1:end,:) = state;
    end
    for ik = 1:length(tmps)
        fprintf('\b');
    end
    tmps = sprintf('Time step %d of %d\nFound minimum: k2 = %4.2f\n',k,length(myt),k2);
    fprintf('%s',tmps);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Now we use the current state predicted
    % for the time of the next update as new 
    % initial state
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if 1==2
        % Find second state by taking derivative
        ind = find(abs(tvec-dt)==min(abs(tvec-dt)));
        ind = ind(1);
        x02 = (y(ind+1)-y(ind-1))/(tvec(ind+1)-tvec(ind-1));
        x01 = y(ind);
    else
        % Find second state by taking the state from simulation
        ind = find(abs(tvec-dt)==min(abs(tvec-dt)));
        ind = ind(1);
        x01 = state(ind,1);
        x02 = state(ind,2);
    end
    
end
fprintf('Cost function using updates:%f\n',sum(stateVec(:,1).*stateVec(:,1))*(1-fak)+fak*sum(stateVec(:,2).*stateVec(:,2)));
 






