function[sys, x0, str, ts] = MY_MPCController(t, x, u, flag)
switch flag,
    case 0   %初始化
        [sys, x0, str, ts] = mdlInitializeSizes;  %初始化
    case 2  %更新离散状态
        sys = mdlUpdates(t, x, u);  
    case 3  %计算输出
        sys = mdlOutputs(t, x, u);
    case {1, 4, 9}  % Unused flags
        sys = [];
    otherwise  %未知的flag
        error(['unhandled flag =' ,num2str(flag)]);  %Error handling
end
% s函数主程序结束

% 初始化子函数
function[sys, x0, str, ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0;  % 连续状态量的个数
sizes.NumDiscStates = 3;  % 离散状态量的个数
sizes.NumOutputs = 2;  % 离散状态量的个数
sizes.NumInputs = 3;  %输入量的个数
sizes.DirFeedthrough = 1;  % 矩阵D非空， 直接贯通标志
sizes.NumSampleTimes = 1;  % 采样时间的个数
sys = simsizes(sizes);
x0 = [0; 0; 0];  % 状态量初始化
global U;
U = [0; 0];
str = [];  % 设置一个str空矩阵
ts = [0.05 0];  % sample time : [period, offset]
% 初始化子函数结束

% 更新离散量状态量子函数
function sys = mdlUpdates(t, x, u)
sys = x;
% 离散量状态量子函数结束

%计算输出子函数
function sys = mdlOutputs(t, x, u)
global a b u_piao;
global U;
global kesi;
Nx = 3;  % 状态量的个数
Nu = 2;  % 控制量的个数
Np = 60;  % 预测步长
Nc = 30;  % 控制步长
Row = 10;  % 松弛因子
fprintf('Update start, t = %6.3f\n', t)
%t_d = u(3) * 3.1415926 / 180;  % 输出的为角度，角度转化为弧度
t_d = u(3)
% 期望轨迹为圆形轨迹
% 半径为25m，速度为5m/s
r(1) = 25 * sin(0.2 * t);
r(2) = 25 + 10 - 25 * cos(0.2 * t);
r(3) = 0.2 * t;
vd1 = 5;
vd2 = 0.104;

% 半径为25m，速度为3m/s
% r(1) = 25 * sin(0.12 * t);
% r(2) = 25 + 10 - 25 * cos(0.12 * t);
% r(3) = 0.12 * t;
% vd1 = 3;
% vd2 = 0.104;

% 半径为25m，速度为10m/s
% r(1) = 25 * sin(0.4 * t);
% r(2) = 25 + 10 - 25 * cos(0.4 * t);
% r(3) = 0.4 * t;
% vd1 = 10;
% vd2 = 0.104;

% 半径为25m，速度为4m/s
% r(1) = 25 * sin(0.16 * t);
% r(2) = 25 + 10 - 25 * cos(0.16 * t);
% r(3) = 0.16 * t;
% vd1 = 4;
% vd2 = 0.104;

% 参数设计
kesi = zeros(Nx + Nu, 1);
kesi(1) = u(1) - r(1);  % u(1) == X(1)
kesi(2) = u(2) - r(2);  % u(2) == X(2)
kesi(3) = t_d - r(3);   % u(3) == X(3)
kesi(4) = U(1);
kesi(5) = U(2);
fprintf('Update start,u(1) = %4.2f\n' , U(1))
fprintf('Update start,u(2) = %4.2f\n' , U(2))
T = 0.05;
%t = 0.05;   % 加
T_all = 40;  % 总的仿真时间，防止计算期望轨迹越界
% 汽车参数
L = 2.6;
% 矩阵初始化
u_piao = zeros(Nx , Nu);
Q = eye(Nx * Np , Nx * Np);
R = 5 * eye(Nu * Nc);
a = [1 0 -vd1 * sin(t_d) * T;
   0 1 vd1 * cos(t_d) * T;
   0 0 1;];
b = [cos(t_d) * T 0;
    sin(t_d) * T 0;
    tan(vd2) * T/L vd1 * T/(cos(vd2)^2) ;];
% 对应（4.6）中的参数
A_cell = cell(2,2);
B_cell = cell(2,1);
A_cell{1,1} = a;
A_cell{1,2} = b;
A_cell{2,1} = zeros(Nu , Nx);
A_cell{2,2} = eye(Nu);
B_cell{1,1} = b;
B_cell{2,1} = eye(Nu);
A = cell2mat(A_cell);
B = cell2mat(B_cell);
C = [1 0 0 0 0 ; 0 1 0 0 0 ; 0 0 1 0 0 ;];
% 对应于（4.10）中的参数
PHI_cell = cell(Np , 1);
THETA_cell = cell(Np ,Nc);
for j = 1:1:Np
    PHI_cell{j , 1}=C * A ^ j;
    for k = 1:1:Nc
        if k <= j
            THETA_cell{j,k} = C * A ^ (j-k) * B;
        else 
            THETA_cell{j,k} = zeros(Nx,Nu);
        end
    end
end
PHI = cell2mat(PHI_cell);  % size(PHI) = [Nx * Np  Nx + Nu]
THETA = cell2mat(THETA_cell);  % size(THETA) = [Nx * Np  Nu + (Nc+1)]
% 以上对应（4.12）参数

H_cell = cell(2,2);
H_cell{1,1} = THETA' * Q * THETA + R;
H_cell{1,2} = zeros(Nu*Nc , 1);
H_cell{2,1} = zeros(1 , Nu * Nc);
H_cell{2,2} = Row;
H = cell2mat(H_cell);
error = PHI * kesi;
f_cell = cell(1,2);
f_cell{1,1} = 2 * error' * Q * THETA;
f_cell{1,2} = 0;
f = cell2mat(f_cell);
% 以上对应（4.19）中的参数

% 等式约束

A_t = zeros(Nc , Nc);
for p = 1:1:Nc
    for q = 1:1:Nc
        if q <= p
            A_t(p,q) = 1;
        else
            A_t(p,q) = 0;
        end
    end
end
A_I = kron(A_t,eye(Nu));   % 对应（4.17）参数
Ut = kron(ones(Nc,1),U);
umin = [-0.2 ; -0.54 ;];  % 维数于控制变量个数相同
umax = [0.2 ; 0.332];
delta_umin = [-0.05;-0.0082;];
delta_umax = [0.05 ; 0.0082];
Umin = kron(ones(Nc,1),umin);
Umax = kron(ones(Nc,1),umax);
A_cons_cell = {A_I zeros(Nu * Nc,1); -A_I zeros(Nu * Nc, 1)};
b_cons_cell = {Umax-Ut;-Umin+Ut};
A_cons = cell2mat(A_cons_cell);     
% (求解方程)状态量不等式约束增益矩阵，装换为绝对值的取值范围
b_cons = cell2mat(b_cons_cell);
% (求解方程)状态量不等式约束的取值

% 状态量约束

M = 10;
delta_Umin = kron(ones(Nc,1),delta_umin);
delta_Umax = kron(ones(Nc,1),delta_umax);
lb = [delta_Umin ; 0];
% 求解方程状态量下界，包含控制时域内控制增量和松弛因子
ub = [delta_Umax ; M];
% 求解方程状态量上界，包含控制时域内控制增量和松弛因子

% 求解开始
options = optimset('Algorithm','interior-point-convex');
[X,fval,exitflag] = quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);

% 计算输出
u_piao(1) = X(1);
u_piao(2) = X(2);
U(1) = kesi(4) + u_piao(1);  % 用于存储上个时刻的控制量
U(2) = kesi(5) + u_piao(2);
u_real(1) = U(1) + vd1;
u_real(2) = U(2) + vd2;
sys = u_real;
%toc
% 输出结束





        







        
    