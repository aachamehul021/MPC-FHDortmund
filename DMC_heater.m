% From HW-3
set(groot,'DefaultAxesColorOrder',[0 0 1; 1 0 0; 0 0 0],'DefaultLineLineWidth',1);
set(groot,'DefaultAxesFontSize',16,'defaultAxesFontName', 'Times','defaultTextFontName', 'Times')
%% Step response model


S=[0 0 0.271 0.0498 0.687 0.845 0.977 1.087 1.179 1.256 1.320 1.374 1.419 1.456 1.487 1.513 1.535 1.553 1.565 1.581 1.592 1.600 1.608 1.614 1.619 1.623 1.627 1.630 1.633 1.635];
S=S'
h=1;
n=120;

%% Controller Parameters
ySP=[ones(1,30),zeros(1,30),ones(1,30),zeros(1,30)];          % Setpoint
m=5;            % Control horizon
p=10;           % Prediction horizon
Q=10;            % Output weight
R=350;         % Input weight
% duMax=0.1;     % Input rate constraint
% duMin=-duMax;
% uMin=-1;      % Input constraint
% uMax=1;

%% Initialization
uPrev=0.0;
Yk0=zeros(n,1);
maxTime=300;

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
bigR=ySP';
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
    predErr=YHAT(2:p+1)-bigR(k:1:k+p-1);
    grad=bigSu'*GammaY*predErr;
%     % Calculate RHS of constraint
%     cRHS=[repmat(duMax,m,1); repmat(-duMin,m,1)];
%     cRHS=[cRHS; repmat( (uMax-uPrev),m,1)];
%     cRHS=[cRHS; repmat(-(uMin-uPrev),m,1)];
    
    % Calculating current step
    big_dU=-inv(Hess)*grad;
    %big_dU=quadprog(Hess,grad,C_LHS,cRHS);
    du=big_dU(1);
    uk=uPrev+du;
    
    % Store results
    U_SAVE(k)=uk;
    uPrev=uk;

    % Implementing current step
    a=size(YHAT);
    YHAT=[YHAT(2:end);YHAT(end)] + [S ; zeros(90,1)]*du;
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
