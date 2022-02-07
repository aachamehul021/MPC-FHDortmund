clc
close all
clear;



l = 0.098;
g = 9.8;
m = 825*1e-3;
M = 8085*1e-3;

A = [0 1 0 0;0 0 -m*g/M 0;0 0 0 1;0 0 g/l 0];
disp(A)
B = [0;1/M;0;-1/(M*l)];
disp(B)

PC = [B A*B A^2*B A^3*B];
fprintf('Controllability matrix:\n');
disp(PC)
if (abs(det(PC))>0.001)
    fprintf('System is controllable!\n');
else
    fprintf('System is NOT controllable!\n');
end
ctrb(A,B)

