
NumT=600;
reference=[ones(1,30),zeros(1,30),ones(1,30),zeros(1,30)];

current_input=[];
p_horizon=5 ;
c_horizon=1;
control_weight=5;
reference=ones(N,1)*75;
initial_op=65;
past_input=[];
Y= ones(NumT,1);
U= ones(NumT,1);
if isempty(current_ip)
    
    %initialization
    stepr=[0 0 0.271 0.0498 0.687 0.845 0.977 1.087 1.179 1.256 1.320 1.374 1.419 1.456 1.487 1.513 1.535 1.553 1.565 1.581 1.592 1.600 1.608 1.614 1.619 1.623 1.627 1.630 1.633 1.635];

    N=numel(stepr);
    %no of past inputs to keep
    n=N-p_horizon;

    %storage for past input
    past_input=zeros(n,1);

    %free reponse from past input
    x=stepr(1:n);
    F=hankel(stepr(2:p_horizon+1),stepr(p+1:N))-repmat(x(:)',p_horizon,1);
    %dynamic matrix
    G=toeplitz(stepr(1:p_horizon),[stepr(1) zeros(1,c_horizon-1)])


    %DMC Gain
    R=chol(G'*G+Smooth_factor*eye(c_horizon);
    K=R\(R'\G');
    K=K(1,:);
    current_ip=0
    %only first input
end

for K=1:600
    smooth_factor=0;
    dmc_ref=R(k:min(N,k+p_horizon)
    %free response
    