% From HW-3
set(groot,'DefaultAxesColorOrder',[0 0 1; 1 0 0; 0 0 0],'DefaultLineLineWidth',1);
set(groot,'DefaultAxesFontSize',16,'defaultAxesFontName', 'Times','defaultTextFontName', 'Times')
%% Step response model
h=1;
[y,t]=filter([0 0 0.2713],[1 -0.8351],ones(50,1));
[y2,t]=filter([0 0 0 0.05],[ 1 -0.9],ones(30,1));
S=y(2:end);
D=y2(2:end);
dD=
N=size(S);
n=N(1,1);
clear G y t
%%
%% Controller Parameters
ySP=zeros(120,1);
ySP([1:30 ])=1;
ySP([61:90])=1;      % Setpoint
m=5;            % Control horizon
p=10;           % Prediction horizon
Q=1;            % Output weight
R=1;         % Input weight
% duMax=0.05;     % Input rate constraint
% duMin=-duMax;
% uMin=-0.5;      % Input constraint
% uMax=0.5;

%% Initialization
uPrev=0.0;
Yk0=zeros(n,1);
maxTime=120;

Y_SAVE=zeros(maxTime+1,1);
T_SAVE=zeros(maxTime+1,1);
U_SAVE=zeros(maxTime,1);

%% Pre-compute Matrices
Su_col=S(1:p,:);
Im_col=ones(m,1);
Dis_col=D(1:p,:);
bigSu=[];
bigIm=[];
bigDu=[];
for i=1:m
    bigSu=[bigSu,Su_col];
    bigIm=[bigIm,Im_col];
    bigDu=[bigDu,Dis_col];
    Su_col=[0;Su_col(1:end-1,:)];
    Im_col=[0;Im_col(1:end-1)];
    Dis_col=[0;Dis_col(1:end-1)];
end
% For Objective

GammaY=diag(ones(p,1)*Q);
GammaU=diag(ones(m,1)*R);
% For constraints
Im=eye(m);

% Pre-compute Hessian
Hess=bigSu'*GammaY*bigSu + GammaU;
% Pre-compute LHS of constraints
% C_LHS=[Im;-Im;bigIm;-bigIm];


%% Implementation
for k=1:maxTime+1
    if k+p-1 >120
        bigR=[bigR(2:end); bigR(end)];
    else
        bigR=ySP(k:k+p-1);
    end
    
%     time=(k-1)*h;   % Current time 
    
    % Calculate error
    if (k==1)
        YHAT=Yk0;
        err=0;
    else
        err=0
%          % Replace this by "y_plant - YHat(1)"
%         y_plant= [YHAT(2:end) YHAT(end)] + bigSu*du 
%         %errror = yplant(k)-y_0(k)
%         err=du -YHAT(1);
    end
    
    % Calculate gradient
    
    predErr=YHAT(2:p+1)-bigR;
    
    grad=bigSu'*GammaY*predErr;
    % Calculate RHS of constraint
%     cRHS=[repmat(duMax,m,1); repmat(-duMin,m,1)];
%     cRHS=[cRHS; repmat( (uMax-uPrev),m,1)];
%     cRHS=[cRHS; repmat(-(uMin-uPrev),m,1)];
    
    % Calculating current step
    big_dU=-inv(Hess)*grad;
%     big_dU=quadprog(Hess,grad,C_LHS,cRHS);
    du=big_dU(1);
    uk=uPrev+du;
    
    % Store results
    U_SAVE(k)=uk;
    uPrev=uk;

    % Implementing current step state update
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
