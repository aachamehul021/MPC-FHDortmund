x=zeros(20,1);
P=0.1*eye(20);
XHAT=[];
YHAT=[];  
KG=[];
SSE=0;
z= zeros(22,1)

epsilon=randn(1,200);
for k=1:200
Xhat=A*x+B*[L(k); V(k)];
Pbar= A*P*A'+blkdiag(R1,R1,R1,R1,R1,R1,R1,R1,R1,R1);
% Pbar=dare(A,C',blkdiag(R1,R1,R1,R1,R1,R1,R1,R1,R1,R1),R2)
kg=Pbar*C'*inv(C*Pbar*C'+R2);
P=(eye(20)-kg*C)*Pbar;
% KG=A*kg;
XHAT(:,k)=x+kg*(ym(k,:)'-C*Xhat)
YHAT(:,k)=H*XHAT(:,k);

end

plot(yc)
hold on
plot (YHAT')
legend('YC(1)','YC(2)','YHAT(1)','YHAT(2)')
%% Compute SSE
sqErr=(yc'-YHAT).^2;   % [x(k)-xhat(k|k)]^2
SSE=sum(sqErr,2);       % Summing sqErr over all k
