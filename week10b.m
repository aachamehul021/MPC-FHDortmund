x=zeros(20,1);
y=zeros(2,1);
P=0.1*eye(22);
XHAT=[];
YHAT=[];
KG=[];
SSE=0;
rng(25);
epsilon=randn(1,200);
%[20*1 (zeros) for delta_x and 2*1 zeros for y]
z= zeros(22,1);
Phi=[A zeros(20,2); C*A eye(2)];
gamma=[B ;C*B];
gammae=[eye(20); C];
zeta=[zeros(2,20) eye(2,2)];
zm=[zeros(2,20) ;eye(20)];
Q_=blkdiag(R1,R1,R1,R1,R1,R1,R1,R1,R1,R1);
qq=zm*Q_*zm';

gain=lqr(Phi,[gamma gammae], qq,eye(22))