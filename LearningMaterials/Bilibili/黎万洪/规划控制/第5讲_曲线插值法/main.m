% 曲线插值法
% 作者：Ally
% 日期：2021/1/16
clc
clear
close all

%% 
d = 3.5;          
len_line = 30;    
W = 1.75;         
L = 4.7;          
x1 = 20;          

t0 = 0;
t1 = 3;
state_t0 = [0, -d/2; 5, 0; 0, 0];  % x,y; vx,vy; ax,ay
state_t1 = [20, d/2; 5, 0; 0, 0];
x2 = state_t0(1);

%%
figure(1)
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on

fill([x1,x1,x1+L,x1+L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  %1号车
fill([x2,x2,x2-L,x2-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'y')  %2号车

plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %分界线
plot([-5,len_line],[d,d],'w','linewidth',2);  %左边界线
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %左边界线

axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

X = [state_t0(:,1); state_t1(:,1)];
Y = [state_t0(:,2); state_t1(:,2)];
T = [ t0^5      t0^4      t0^3     t0^2    t0   1;
      5*t0^4    4*t0^3    3*t0^2   2*t0    1    0;
      20*t0^3   12*t0^2   6*t0     1       0    0;
      t1^5      t1^4      t1^3     t1^2    t1   1;
      5*t1^4    4*t1^3    3*t1^2   2*t1    1    0;
      20*t1^3   12*t1^2   6*t1     1       0    0];
A = T \ X;
B = T \ Y;

t=(t0:0.05:t1)';
path=zeros(length(t),4);
for i = 1:length(t)
    path(i,1) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * A;    
    path(i,2) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * B;
    path(i,3) = [5*t(i)^4,  4*t(i)^3,  3*t(i)^2,  2*t(i), 1, 0] * A;
    path(i,4) = [5*t(i)^4,  4*t(i)^3,  3*t(i)^2,  2*t(i), 1, 0] * B;
end

plot(path(:,1),path(:,2),'r--','linewidth',1.5); 
figure 
plot(t, path(:,4), 'k'); 
xlabel('t / s ');
ylabel('vy / m/s ');
figure 
plot(t, path(:,3), 'k'); 
xlabel('t / s ');
ylabel('vx / m/s ');