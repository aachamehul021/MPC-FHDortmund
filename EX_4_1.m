clc
close all
clear


f = 0.01:0.01:10;
s = 2*pi*f*1i;

% k = [[0:0.2:1.4] [1.4:0.01:1.6] [1.6:0.5:10]];
k = [1.25:0.1:1.75];
Tmax = 20;

fig1 = figure();
fig2 = figure();
fig3 = figure();

mymax = -1e9*ones(size(s));
mymin = 1e9*ones(size(s));
for ik = 1:length(k)
    K = k(ik);
    tmp = roots([1 1 K]);   
    ind = find(real(tmp)>0);
    if ~isempty(ind)
        fprintf('unstable for %4.2f\n',K);
        stab(ik) = 0;
    else
        stab(ik) = 1;
    end
    sys = tf([K],[1 1 K]);
    [u,t] = step(sys,Tmax);    
    ind = find(u > 1.25);
    if ~isempty(ind)
        fprintf('Overshoot larger 25%% for %4.2f\n',K);
    end
    figure(fig1);
    plot(t,u);
    hold on;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(fig2);    
    T = K./(s.^2+s+K);
    T = 20*log10(abs(T));
    mymax = max(T,mymax);
    mymin = min(T,mymin);
    semilogx(f,T)
    ylim([-20 10]);
    hold on;

    
end
figure(fig1);
line([0 Tmax],[1.25 1.25]);
figure(fig2);
plot(f,mymax,'linewidth',5,'color','red','LineStyle','--');
plot(f,mymin,'linewidth',5,'color','red','LineStyle','--');
figure(fig3)
semilogx(f,mymax-mymin);
figure

K = 1.5;
T = K./(s.^2+s+K);
T = 20*log10(abs(T));
semilogx(f,T)
hold on;
SKT = (s.^2+s)./(s.^2+s+K);
SKT = 20*log10(abs(SKT));
semilogx(f,SKT,'color','red')
legend('Transfer function','Sensitivity','max')
ylim([-20 10]);
ind = find(SKT == max(SKT));
line([f(ind) f(ind)],[ -20 10]);
figure(fig2)
line([f(ind) f(ind)],[ -20 10]);
figure(fig3)
line([f(ind) f(ind)],[min(mymax-mymin) max(mymax-mymin) ]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms K s;
F = K/(s^2+s+K)/s;
ilaplace(F)

