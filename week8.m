
A=[0.3 -0.4 ;0.4 0.25];
B=[1;0];
phic=[B A*B];
Q=eye(2);
R=10000;
[X,L,K] = idare(A,B,Q,R)

