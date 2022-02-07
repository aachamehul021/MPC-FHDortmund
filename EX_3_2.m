clc
clear
close all

fig1 = figure;
fig2 = figure;

A = [0 1 0 0;23.544 0 0 0;0 0 0 1;-5.886 0 0 0];
B = [0;-1.2;0;0.8];
C = eye(4); % -> Want to get all elements
R = 0.1;

c = {'red','green','blue','cyan'};

mylegend = {'case 1','case 2','case 3','case 4'};


for myset = 1:4
    switch myset
        case 1
            Q = 1*eye(size(A));
        case 2
            Q = [100 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
        case 3
            Q = [1 0 0 0;0 1 0 0;0 0 100 0;0 0 0 1];
        case 4
            Q = 1*eye(size(A));
            R = 10;
    end
    [K,S,E] = lqr(A,B,Q,R);

    Afull = A-B*K;
    fprintf('-------------------------\n');
    fprintf('Q = \n[');
    for ik = 1:4
        fprintf('%4.2f\t%4.2f\t%4.2f\t%4.2f\n',Q(ik,1),Q(ik,2),Q(ik,3),Q(ik,4));
    end
    fprintf(']\nleads to:\n');
    fprintf('K = [%4.2f\t%4.2f\t%4.2f\t%4.2f]\n',K(1),K(2),K(3),K(4));
    full = ss(Afull,B,C,0);
    x0 = [0.1 0 0 0]; % Initial state
    t = 0:0.01:6;
    x = lsim(full,zeros(size(t)),t,x0);
    figure(fig1);
    for ik = 1:4
        subplot(2,2,ik);
        hold on
        plot(t,x(:,ik)*1,'color',c{myset})
        xlabel('t');
        ylabel('Step response');
        tmps = sprintf('state number %d',ik);
        title(tmps)
    end
    figure(fig2);
    for ik = 1:length(x(:,1))
        u = -K*x(ik,:)';
        E_U(ik) = dot(u,u);
    end
    plot(E_U,'color',c{myset});
    hold on
    tmps = sprintf('Control energy: %4.2f',sum(E_U));
    mylegend_2{myset} = sprintf('%s %s',mylegend{myset},tmps);
end
for ik = 1:4
    figure(fig1)
    subplot(2,2,ik);
    legend(mylegend)
end
figure(fig2)
legend(mylegend_2)
ylim([0 20])


