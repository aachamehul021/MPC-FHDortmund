clc
clear
close all

k1 = 1;
k2 = 0.1:0.001:5;
J = (k2.^2+2*k2+4)./k2/2;
plot(k2,J);
ind = find(J==min(J));
fprintf('Found minimum for k2 = %4.2f\n',k2(ind));
hold on
plot(k2(ind),J(ind),'.','MarkerSize',10);
xlabel('k_2');
ylabel('J');


  

