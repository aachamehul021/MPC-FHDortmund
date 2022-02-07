% From HW-6
%solution for problem 1 and 2
%if the Effect of weight then the output takes a long tme to converge
%becasue
set(groot,'DefaultAxesColorOrder',[0 0 1; 1 0 0; 0 0 0],'DefaultLineLineWidth',1);
set(groot,'DefaultAxesFontSize',16,'defaultAxesFontName', 'Times','defaultTextFontName', 'Times')
%% Step response model
G=tf(1.25,[5 1],'inputdelay',1.4); h=1;
n=25;
[y,t]=step(G,0:h:n*h);
S=y(2:end);
clear G y t

%% Controller Parameters

ySP=0.8;          % Setpoint
m=4;            % Control horizon
p=10;           % Prediction horizon
Q=1;            % Output weight
R=0.1;         % Input weight
duMax=0.1;     % Input rate constraint
duMin=-duMax;
uMin=-1;      % Input constraint
uMax=1;

param=4;
switch(param)
    
    case 1
        disp('setpoint at 0.8')
    case 2
        R =100;
        disp('output converges after a long time as weight is given to change in input');
    case 3
        p=4;
        disp('convergence takes slighlty longer time ')
    case 4
        m=1;
        disp(' less robust and slower')
end
%% Initialization
uPrev=0.0;
Yk0=zeros(n,1);
maxTime=50;

Y_SAVE=zeros(maxTime+1,1);
T_SAVE=zeros(maxTime+1,1);
U_SAVE=zeros(maxTime,1);

%% Pre-compute Matrices
Su_col=S(1:p,:);
Im_col=ones(m,1);
bigSu=[];
bigIm=[];
for i=1:m
    bigSu=[bigSu,Su_col];
    bigIm=[bigIm,Im_col];
    Su_col=[0;Su_col(1:end-1,:)];
    Im_col=[0;Im_col(1:end-1)];
end
% For Objective
bigR=ones(p,1)*ySP;
GammaY=diag(ones(p,1)*Q);
GammaU=diag(ones(m,1)*R);
% For constraints
Im=eye(m);

% Pre-compute Hessian
Hess=bigSu'*GammaY*bigSu + GammaU;
% Pre-compute LHS of constraints
C_LHS=[Im;-Im;bigIm;-bigIm];


%% Implementation
for k=1:maxTime+1
    time=(k-1)*h;   % Current time 
    
    % Calculate error
    if (k==1)
        YHAT=Yk0;
        err=0;
    else
        %err=0; % Replace this by "y_plant - YHat(1)"
        
        err=du -YHAT(1);
    end
    
    % Calculate gradient
    predErr=YHAT(2:p+1)-bigR;
    grad=bigSu'*GammaY*predErr;
    % Calculate RHS of constraint
    cRHS=[repmat(duMax,m,1); repmat(-duMin,m,1)];
    cRHS=[cRHS; repmat( (uMax-uPrev),m,1)];
    cRHS=[cRHS; repmat(-(uMin-uPrev),m,1)];
    
    % Calculating current step
    % big_dU=-inv(Hess)*grad;
    big_dU=quadprog(Hess,grad,C_LHS,cRHS);
    du=big_dU(1);
    uk=uPrev+du;
    
    % Store results
    U_SAVE(k)=uk;
    uPrev=uk;
    % plant equations
    % Implementing current step
    YHAT=[YHAT(2:end);YHAT(end)] + S*du;
    yk=YHAT(1);
    
    % Storing results
    Y_SAVE(k+1)=yk;
    T_SAVE(k+1)=k*h; 
end

%% Plotting results
subplot('position',[0.12 0.58 0.8 0.4]); 
plot(T_SAVE(1:maxTime),Y_SAVE(1:maxTime),'-b','linewidth',1); 
ylabel('Output, y_k');
subplot('position',[0.12 0.12 0.8 0.4]); 
stairs(T_SAVE(1:maxTime),U_SAVE(1:maxTime),'-b','linewidth',1); 
ylabel('Input, u_k'); xlabel('time, t')

clear S n Y0 k du
