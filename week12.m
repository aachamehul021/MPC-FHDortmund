Gp_p=[ tf(4.05,[50 1],'InputDelay',27) tf(1.77,[60 1],'InputDelay',28) tf(5.88,[50 1],'InputDelay',27); 
    tf(5.39,[50 1],'InputDelay',18) tf(5.72,[60 1],'InputDelay',14) tf(6.9,[40 1],'InputDelay',15);
    tf(4.38,[33 1],'InputDelay',20) tf(4.42,[44 1],'InputDelay',22) tf(7.2,[19 1])];
Gw_p=[tf(1.2,[45 1],'InputDelay',27) tf(1.44,[40 1],'InputDelay',27);
    tf(1.52,[25 1],'InputDelay',15) tf(1.83,[20 1],'InputDelay',15);
    tf(1.14,[27 1]) tf(1.26,[32 1])];
Ts=2;
H=ss(Gp_p);
H=setmpcsignals(H,'MV',[1:3]);
cont=mpc(c2d(H,2),2,40,10)
setoutdist(cont,'model',Gw_p);
cont.MV.min={ones(1,3)*-0.5}
cont.MV.max=0.5;
cont.OV.min=-0.5;
cont.OV.max=0.5;
cont.MV.RateMin=-0.05;
cont.MV.RateMax=0.05;

%%
Gp_m=[ tf(6,[50 1],'InputDelay',27) tf(1.4,[60 1],'InputDelay',28) tf(6.2,[50 1],'InputDelay',27); 
    tf(8,[50 1],'InputDelay',18) tf(5.2,[60 1],'InputDelay',14) tf(7.6,[40 1],'InputDelay',15);
    tf(7.5,[33 1],'InputDelay',20) tf(3.7,[44 1],'InputDelay',22) tf(8.5,[19 1])];
Gw_m=[tf(1.3,[45 1],'InputDelay',27) tf(1.6,[40 1],'InputDelay',27);
    tf(1.6,[25 1],'InputDelay',15) tf(2,[20 1],'InputDelay',15);
    tf(1.3,[27 1]) tf(1.4,[32 1])];
Ts=2;
H=ss(Gp);
cont2=mpc(c2d(H,2),2,40,10);
setoutdist(cont2,'model',Gw);


