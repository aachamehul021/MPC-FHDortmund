%convert tf to step response model
sr=filter([0 0 0.2713],[1 -0.8351],ones(50,1));
%prediction horizon
p=10;
%control horizon
m=5;
%initial output
y=0;
%past input
v=[];
%input
u=zeros(1,3);
N=120;
%store output, input and reference
Y=zeros(N,1);
U=zeros(N,1);
Ref=zeros(N,1);

Ref([1:30])=1;
Ref([61:90])=1;

%smoothing factor
la=1;
N=numel(sr);
% initial setup
if isempty(v)
    % number of past inputs to keep
    n=N-p;
    % storage for past input
    v=zeros(n,1);
    % matrix to calculate free response from past input
    x=sr(1:n);
%     F=hankel(sr(2:p+1),sr(p+1:N))-repmat(x(:)',p,1);
    % dynamic matrix
    G=toeplitz(sr(1:p),sr(1)*eye(1,m));
    % calculate DMC gain
    R=chol(G'*G+la*eye(m));
    K=R\(R'\G');
    % only the first input will be used
    k_in=K(1,:)';
    current_u=0;
end
for k=1:120
    a=0;
    r=Ref(k:min(N,k+p));
    if k>60
        a=0.7;
    end
    
   
    % free response
    f=F*v+y;
    % smooth reference
%     nr=numel(r)
%     if nr>=p
%         ref=r(1:p);
%     else
%         r
%         ref=[r(:);r(end)+zeros(p-nr,1)];
%     end
    w=filter([0 (1-a)],[1 -a],ref,y);
    % DMC input change
    u=k_in.*(w-f);
    % past input change for next step
    v=[u;v(1:end-1)];
    % next input
    current_u=current_u+u(1);
    Y(k)=y;
    U(k)=current_u;
    u=[u(2:3)' current_u];
    y=0.8351*y+0.2713*u(1);
end
subplot(211)
plot(1:120,Y','b-',1:120,Ref,'r--',[60 60],[-0.5 1.5],':','linewidth',2)
title('solid: output, dashed: reference')
text(35,1,'\alpha=0')
text(95,1,'\alpha=0.7')
axis([0 120 -0.5 1.5])
subplot(212)
[xx,yy]=stairs(1:120,U);
plot(xx,yy,'-',[60 60],[-0.5 1.5],':','linewidth',2)
axis([0 120 -0.5 1.5])
title('input')
xlabel('time, min')
