%% System matrices
A=[0.3, -0.4; 0.4, 0.25];
Be=[0.25; 1];
C=[1 0];
P=zeros(2);
Pbar=zeros(2);
R1=eye(2);
R2=0.64;
kg=[0;0];
% Noise variances and realizations
rng(25);
epsilon=randn(1,200);   % Realization of state error
nu=0.8*randn(1,200);    % Realization of measurement noise

%% Initialization
xk=[1; 1];          % This is x(0)
XALL=zeros(2,200);  % For storing xk at all times
YALL=zeros(1,200);  % For storing yk at all times

%% Generating data
for k=1:200
    xk=A*xk+Be*epsilon(k);
    yk=C*xk+nu(k);
    
    XALL(:,k)=xk;    % Storing xk at all times
    YALL(:,k)=yk;    % Storing yk at all times
end

%% Running the estimator
xhat=[0;0];
XHAT=zeros(2,200);
kg=0

% Estimator calculations
for k=1:200
    % 1. Calculate xhat(k|k-1) from xhat(k-1|k-1)
    xhat_= A*xhat+Be*epsilon(k)+kg*(YALL(k)-C*xhat);
    % 2. Use Kinf, xhat(k|k-1) and y(k) to calculate xhat(k|k)
    Pbar= A*P*A'+R1-k*C*Pbar;
    kg=A*Pbar*C'*inv(C*Pbar*C'+R2);
    P=(eye(2)-kg*C)*Pbar;

    xhat=xhat_+kg*(YALL(k)-C*xhat_)
    
    
    
    % Store the results
    XHAT(:,k)=xhat;    
end

%% Plot the results
subplot(2,1,1)
plot(1:200,XALL(1,:), 1:200,XHAT(1,:));
subplot(2,1,2)
plot(1:200,XALL(2,:), 1:200,XHAT(2,:));

%% Compute SSE
sqErr=(XALL-XHAT).^2;   % [x(k)-xhat(k|k)]^2
SSE=sum(sqErr,2);       % Summing sqErr over all k
