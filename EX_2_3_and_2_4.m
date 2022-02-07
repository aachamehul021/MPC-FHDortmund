clc
close all
clear;
myset = 2;

% 1: Full-state controller
% 2: Observer WITHOUT reference input
% 3: Reference input (not part of exercise)
% 4: Reference input (not part of exercise)

factor = 180/pi;

A = [0 1 0 0;0 0 -1 0;0 0 0 1;0 0 100 0];
B = [0;0.1237;0; -1.2621];
C = eye(4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Poles:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Controller
    FullFeedbackPoles = [-8-6i;-8+6i;-0.4-0.3i;-0.4+0.3i];
    % Observer
    ObserverPoles = [-16-1i*21.3 -16-1i*21.3 -16+1i*21.3 -16+1i*21.3];

switch myset
    case 1     
        t = 0:0.01:5;
    case 2
        t = 0:0.01:10; 
    case 4
        t = 0:0.01:10;
    otherwise
        t = 0:0.01:3;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System checks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    PC = [B A*B A^2*B A^3*B];
    %fprintf('Controllability matrix:\n');
    %disp(PC)
    if (abs(det(PC))>0.001)
        fprintf('System is controllable!\n');
    else
        fprintf('System is NOT controllable!\n');
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calc K for full state feedback
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    K = acker(A,B,FullFeedbackPoles);
    %fprintf('K for full state feedback:\n');
    %disp(K);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Place the poles by hand instead of acker
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        den = poly(FullFeedbackPoles);    
        q = zeros(size(A));
        N = length(den);
        for k = 1:N        
            q = q + A^(N-k) * den(k);
        end
        K0 = [0 0 0 1]*inv(PC)*q;
        if norm(K-K0) > 1e-10
            disp('error');
        end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Full-state feedback
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Afull = A - B*K;
    disp(Afull)
    full = ss(Afull,B,C,0);
    x0 = [0 0 0.1 0]; % Initial state
    ufull = lsim(full,zeros(size(t)),t,x0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System respsonse for observer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if myset ~= 1
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    CSL = [1 0 0 0];
    % Calculate the observalibility matrix P0
    P0 =  [CSL;CSL*A;CSL*A^2;CSL*A^3];
    % [num,den] = zp2tf([],ObserverPoles,1);
    den = poly(ObserverPoles);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Place the poles
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    q = zeros(size(A));
    N = length(den);
    for k = 1:N        
        q = q + A^(N-k) * den(k);
    end
    L = q*inv(P0)*[0 0 0 1]';
   
   
    fprintf('L:\n');
    disp(L) 
    

    switch myset
        case 2

            % Without reference input
            AObserver = [A-B*K,B*K;zeros(size(A)),A-L*CSL];
            disp(AObserver)
            BObserver = zeros(8,1);
            observer = ss(AObserver,BObserver,eye(8),0);
            InitialState = [x0 x0];
            % Note the InitialState:
            % the state vector contains the state and the error
            % the initial state for the error is as before
            % The initial error is simply the initial state if initial
            % state estimate is set to zero (error is state - estimate)
            uobserver = lsim(observer,zeros(size(t)),t,InitialState);

        case 3

            AObserver = [A -B*K; L*CSL A-L*CSL-B*K];
            disp(AObserver)
            N = 2.3;
            BObserver = [B;B]*N;
            observer = ss(AObserver,BObserver,eye(8),0);
            % Note that here we use the estimated state.
            % Compared to case 2 we need to change the initial state
            v0 = 1; % m/sec
            r = t*v0;
            r = 0.5*ones(size(t)); % acc signal
            uobserver = lsim(observer,r,t,[x0 0 0 0 0]);

      case 4

            AObserver = [A -B*K; L*CSL A-L*CSL-B*K];
            disp(AObserver)
        
            BObserver = -[zeros(size(L));L];
            observer = ss(AObserver,BObserver,eye(8),0);
            % Note that here we use the estimated state.
            % Compared to case 2 we need to change the initial state
            v0 = 1; % m/sec
            r = t*v0;
            r = 0.5*ones(size(t)); % acc signal
            uobserver = lsim(observer,r,t,[x0 0 0 0 0]);

    end
        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:4
    subplot(2,2,k);
    plot(t,ufull(:,k)*factor,'red');   
    if myset > 1
        hold on
        plot(t,uobserver(:,k)*factor,'green');
        xlabel('t');
        tmps = sprintf('state %d',k);
        ylabel(tmps)
    end
    s = sprintf('state #%d',k);
    title(s)
    legend('full-state feedback','Observer');
end
