% 纯跟踪（Pure Pursuit）法
% 作者：Ally
% 日期：20210227
clc
clear
close all
load  path.mat

%% 相关参数定义
RefPos = path;
targetSpeed = 10;      % m/s
Kv = 0.1;              % 前视距离系数
Kp = 0.8;              % 速度P控制器系数
Ld0 = 2;               % Ld0是预瞄距离的下限值
dt = 0.1;              % 时间间隔，单位：s
L = 2.9;               % 车辆轴距，单位：m

%% 主程序

% 车辆初始状态定义
pos = RefPos(1,:);
v = 0;
heading = 0;

% 将初始状态纳入实际状态数组中
pos_actual = pos;
heading_actual = heading;
v_actual = v;
idx_target = 1;

% 循环遍历轨迹点
while idx_target < size(RefPos,1)-1
    % 寻找预瞄距离范围内最近路径点
    [lookaheadPoint,idx_target] = findLookaheadPoint(pos, v, RefPos,Kv, Ld0);
   
    % 计算控制量
    delta = pure_pursuit_control(lookaheadPoint,idx_target,pos, heading, v, RefPos, Kv, Ld0,L);
    
    % 计算加速度
    a = Kp* (targetSpeed-v)/dt;
    
    % 更新状态量
    [pos, heading, v] = updateState(a,pos, heading, v,delta,L, dt);
    
    % 保存每一步的实际状态量
    pos_actual(end+1,:) = pos;
    heading_actual(end+1,:) = heading;
    v_actual(end+1,:) = v;
end

% 画图
figure
plot(path(:,1), path(:,2), 'b');
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
hold on 
for i = 1:size(pos_actual,1)
    scatter(pos_actual(i,1), pos_actual(i,2),150, '.r');
    pause(0.05)
end
legend('规划车辆轨迹', '实际行驶轨迹')

% 保存
path_PP = pos_actual;


%% 首先在参考轨迹上搜索离当前车辆位置最近的点
function  [lookaheadPoint,idx_target] = findLookaheadPoint(pos, v, RefPos, Kv, Ld0)

% 找到距离当前位置最近的一个参考轨迹点的序号
sizeOfRefPos = size(RefPos,1);
for i = 1:sizeOfRefPos
    dist(i,1) = norm(RefPos(i,:) - pos);   
end
[~,idx] = min(dist); 


% 从该点开始向轨迹前方搜索，找到与预瞄距离最相近的一个轨迹点
L_steps = 0;           % 参考轨迹上几个相邻点的累计距离
Ld = Kv*v + Ld0;       % Ld0是预瞄距离的下限值；
while L_steps < Ld && idx < sizeOfRefPos
    L_steps = L_steps + norm(RefPos(idx + 1,:) - RefPos(idx,:));
    idx = idx+1;
end
idx_target = idx;
lookaheadPoint = RefPos(idx,:);
end


%% 获得控制量：前轮转向
function delta = pure_pursuit_control(lookaheadPoint,idx_target,pos, heading, v,RefPos, Kv, Ld0, L)
sizeOfRefPos = size(RefPos,1);
if idx_target < sizeOfRefPos
    Point_temp = lookaheadPoint;
else
    Point_temp = RefPos(end,1:2);
end
alpha = atan2(Point_temp(1,2) - pos(2), Point_temp(1,1) - pos(1))  - heading;
Ld = Kv*v + Ld0;

%前轮转角
delta = atan2(2*L*sin(alpha), Ld);

end

%% 更新状态量
function [pos_new, heading_new, v_new] = updateState(a,pos_old, heading_old, v_old,delta,wheelbase, dt)
pos_new(1) = pos_old(1) + v_old*cos(heading_old)*dt;
pos_new(2) =  pos_old(2) + v_old*sin(heading_old)*dt;
heading_new=  heading_old + v_old*dt*tan(delta)/wheelbase;
v_new =  v_old + a*dt;
end

