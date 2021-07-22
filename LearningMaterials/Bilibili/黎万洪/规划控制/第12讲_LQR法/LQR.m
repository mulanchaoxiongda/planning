clc
clear
close all
load  path.mat

%% 参数定义
dt = 0.1;
L = 2.9 ;
Q = [1, 0,  0;
      0, 1, 0;
      0, 0,  1];
R = eye(2)* 2;
refSpeed = 40/3.6;


%% 轨迹处理
% 定义参考轨迹
refPos_x = path(:,1);
refPos_y = path(:,2);
refPos = [refPos_x, refPos_y];

% 计算一阶导数
for i = 1:length(refPos_x)-1
    refPos_d(i) = (refPos(i+1,2)-refPos(i,2))/(refPos(i+1,1)-refPos(i,1));
end
refPos_d(end+1) = refPos_d(end);

% 计算二阶导数
for i =2: length(refPos_x)-1
    refPos_dd(i) = (refPos(i+1,2)-2*refPos(i,2) + refPos(i-1,2))/(0.5*(-refPos(i-1,1)+refPos(i+1,1)))^2;
end
refPos_dd(1) = refPos_dd(2);
refPos_dd(length(refPos_x)) = refPos_dd(length(refPos_x)-1);

% 计算曲率
for i  = 1:length(refPos_x)-1
    k(i) = (refPos_dd(i))/(1+refPos_d(i)^2)^(1.5);
end

refPos_x = refPos_x';
refPos_y = refPos_y';
refPos_yaw = atan(refPos_d');
refPos_k = k';
refPos_Delta = atan(L*refPos_k);

%% 主程序
% 赋初值
pos_x = 0; 
pos_y = 0; 
pos_yaw = 0; 
v = 10;
Delta = 0;
idx = 1;

% 单步长实际量
pos_actual = [pos_x,pos_y];
v_actual  = v;
Delta_actual = Delta;
idx_actual = 1;
% 循环
while idx < length(refPos_x)-1
    % 寻找参考轨迹最近目标点
    idx = calc_target_index(pos_x,pos_y,refPos_x,refPos_y);
    refDelta = 0;%refPos_Delta(idx);
    
    % LQR控制器
    [v_delta,delta,yaw_error] =  LQR_control(idx,pos_x,pos_y,v,pos_yaw,refPos_x,refPos_y,refPos_yaw,refPos_k,L,Q,R,dt);    
    
    % 更新状态
    [pos_x,pos_y,pos_yaw,v,Delta] = update(pos_x,pos_y,pos_yaw,v, v_delta,delta, dt,L, refSpeed,refDelta);
    pos_actual(end+1,:) = [pos_x,pos_y];
    v_actual(end+1,:)  = v;
    Delta_actual(end+1)  = Delta;
    idx_actual(end+1) = idx;
end

% 画图
figure
yyaxis left
plot(refPos_x,refPos_y,'b-')
hold on
for i = 1:size(pos_actual,1)
    scatter(pos_actual(i,1), pos_actual(i,2),150,'b.')
    pause(0.05);
end

% 计算平均跟踪误差
pos_refer = refPos(idx_actual);
for i = 1:length(idx_actual)
     LQR_error(i) = norm(refPos(idx_actual(i),:) - pos_actual(i,:));
end
yyaxis right
plot(LQR_error, 'r');

% 保存
path_LQR = pos_actual;
save path_LQR.mat path_LQR

%% 寻找参考轨迹最近目标点
function target_idx = calc_target_index(pos_x,pos_y, refPos_x,refPos_y)
i = 1:length(refPos_x)-1;
dist = sqrt((refPos_x(i)-pos_x).^2 + (refPos_y(i)-pos_y).^2);
[~, target_idx] = min(dist);
end


%% LQR控制
function [v_delta,delta,yaw_error] =  LQR_control(idx,pos_x,pos_y,v,pos_yaw,refPos_x,refPos_y,refPos_yaw,refPos_k,L,Q,R,dt)

% 求位置、航向角状态量计算
x_error  = pos_x - refPos_x(idx);
y_error = pos_y - refPos_y(idx);
yaw_error =  pipi(pos_yaw - refPos_yaw(idx));
X(1,1) = x_error; 
X(2,1) = y_error;  
X(3,1) = yaw_error;

% 由状态方程矩阵系数，计算K
A = [1,  0,  -v*dt*sin(pos_yaw);
     0,  1,  v * dt * cos(pos_yaw);
     0,  0,  1];
B = [dt * cos(pos_yaw),    0;
     dt * sin(pos_yaw),    0;
     dt * tan(pos_yaw)/L,  v*dt/(L * cos(pos_yaw)^2)];


K = calcu_K(A,B,Q,R);

% 获得前轮速度变化量、前轮转角变化量两个控制量
u = -K * X;  % 2行1列
v_delta = u(1);
delta = pipi(u(2));

end

%% 角度转换到[-pi, pi]
function angle_out = pipi(angle_in)
if (angle_in > pi)
    angle_out =  angle_in - 2*pi;
elseif (angle_in < -pi)
    angle_out = angle_in + 2*pi;
else
    angle_out = angle_in;
end
end

%% 计算增益
function K = calcu_K (A,B,Q,R)

% 终止条件定义
iter_max = 500;
epsilon = 0.01;

% 循环
P_old = Q;
for i = 1:iter_max
    P_new = A' * P_old * A - (A' * P_old * B) / (R + B' * P_old * B) *( B' * P_old * A) +Q;
    if abs(P_new - P_old) <= epsilon
        break
    else
        P_old = P_new; 
    end
end

P = P_new;
K = (B' * P * B + R) \ (B' * P * A);  % 2行3列
end

%% 更新状态
function [x, y, yaw, v, Delta] = update(x, y, yaw, v, v_delta,delta,dt,L,refSpeed,refDelta)
Delta = refDelta + delta;
x = x + v * cos(yaw) * dt;
y = y + v * sin(yaw) * dt;
yaw = yaw + v / L * tan(Delta) * dt;
v = refSpeed + v_delta;
end
