% DMC   Dynamic Matrix Control
% P=DMC(P) determines the dynamic matrix control (input change) based on
% the plant model (step response) and current measurement stored in the
% structure P. 
% Input:
%   P.sr - unit step response data
%   P.u  - current input, initially 0
%   P.v  - past input, initially empty
%   P.G  - dynamic matrix, set by the initial call
%   P.F  - matrix to calculate free response, set by the initial call
%   P.k  - DMC gain, set by the initial call
%   P.r  - reference (set point)
%   P.a  - reference smooth factor
%   P.p  - prediction horizon
%   P.m  - moving horizon
%   P.y  - current mrasurement
%   P.la - performance criterion weight, i.e. J = ||r-y|| + p.la*||du||
%          where du is the input change
% Output:
%   P.u  - new input for next step
%   P.f  - updated free response
%   P.G  - dynamic matrix, if it is the first step.
%   P.k  - DMC gain, if it is the first step
%
% See Also: mpc
% Version 1.0 created by Yi Cao at Cranfield University on 6th April 2008.
% Example: 
p= struct;
p.sr=filter([0 0 0.2713],[1 -0.8351],ones(30,1));
%p.dr=filter([0 0 0 0.05],[ 1 -0.9],ones(30,1));
p.p=10;
p.m=5;
p.y=0;
p.v=[];
u=zeros(1,3);
N=120;
Y=zeros(N,1);
U=zeros(N,1);
R=zeros(N,1);
R([1:30 61:90])=1;
p.la=1;
for k=1:120
    p.a=0;
    p.r=R(k:min(N,k+p.p));
    if k>60
        p.a=0;
    end
    p=dmc(p);
    Y(k)=p.y;
    U(k)=p.u;
    u=[u(2:3) p.u];
    %possibly add disturbance here for time 30 to 70
    p.y=0.8351*p.y+0.2713*u(1);
end
subplot(211)
plot(1:N,Y,'b-',1:N,R,'r--',[60 60],[-0.5 1.5],':','linewidth',2)
title('solid: output, dashed: reference')
text(35,1,'\alpha=0')
text(95,1,'\alpha=0.7')
axis([0 120 -0.5 1.5])
subplot(212)
[xx,yy]=stairs(1:N,U);
plot(xx,yy,'-',[60 60],[-0.5 1.5],':','linewidth',2)
axis([0 120 -0.5 1.5])
title('input')
xlabel('time, min')
