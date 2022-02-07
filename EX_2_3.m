clc
close all
clear;

factor = 180/pi;

A = [0 1 0 0;0 0 -1 0;0 0 0 1;0 0 100 0];
B = [0;0.1237;0; -1.2621];
C = eye(4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Poles:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FullFeedbackPoles = [-8-6i;-8+6i;-0.4-0.3i;-0.4+0.3i];

t = 0:0.01:5;
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
    fprintf('K for full state feedback:\n');
    disp(K);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Implement ackermann equation "by hand"
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
% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:4
    subplot(2,2,k);
    plot(t,ufull(:,k)*factor,'red');   
    s = sprintf('state #%d',k);
    title(s)
    legend('full-state feedback');
end
