G=c2d(ss(tf([1.25],[5 1], 'InputDelay',1.4)),0.5);
cont=mpc(G,0.5,12,5)
%Q
cont.w.OV=1;
%R
cont.w.MVRate=0.1;
%constrains
cont.MV.Min=-0.4;
cont.MV.Max=0.4;
cont.MV.RateMin=-0.025;
cont.MV.RateMax=0.025;
sim(cont,100,0.8)
%%
clear
G1= tf([1.25],[5 1], 'InputDelay',1.4);
G2=tf([0.2],[6 1], 'InputDelay',0.7);
H=ss([G1 G2])
H=setmpcsignals(H,'MV',1,'MD',2)
cont=mpc(c2d(H,0.5),0.5,12,5)
%Q
cont.w.OV=1;
%R
cont.w.MVRate=0.1;
%constrains
cont.MV.Min=-0.4;
cont.MV.Max=0.4;
cont.MV.RateMin=-0.025;
cont.MV.RateMax=0.025;
v=0.5*ones(100,1);
sim(cont,100,0.8,v);

%%
clear
G1= tf([1.25],[5 1], 'InputDelay',1.4);
G2=tf([0.2],[6 1], 'InputDelay',0.7);
G3=tf([1.2],[5.5 1],'InputDelay',1.2);
%model
H=ss([G1 G2])
H=setmpcsignals(H,'MV',1,'MD',2);
cont=mpc(c2d(H,0.5),0.5,12,5);
%Q
cont.w.OV=1;
%R
cont.w.MVRate=0.1;
%constrains
cont.MV.Min=-0.4;
cont.MV.Max=0.4;
cont.MV.RateMin=-0.025;
cont.MV.RateMax=0.025;
v=0.5*ones(100,1);

%real plant model
I=ss([G3 G2]);
I=setmpcsignals(I,'MV',1,'MD',2);

options=mpcsimopt(cont);
options.model=I;

sim(cont,60,0.8,v,options);
