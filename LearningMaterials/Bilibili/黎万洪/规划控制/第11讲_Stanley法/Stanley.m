% Stanley法
% 作者：Ally
% 日期：20210312
clc
clear
close all
load  path.mat

%% 相关参数定义
RefPos = path;              % 参考轨迹
targetSpeed = 20;           % 目标速度，单位： m /s
InitialState = [0,-2,0,0];  % 纵向位置、横向位置、航向角、速度
k = 0.1;                    % 增益参数
Kp = 1;                     % 速度P控制器系数
dt = 0.1;                   % 时间间隔，单位：s
L = 2;                      % 车辆轴距，单位：m

%% 主程序

% 车辆初始状态定义
state = InitialState;
state_actual = state;
target_idx = 1;

while target_idx < size(RefPos,1)-1
    % 寻找预瞄距离范围内最近路径点
    [target_idx,latError] = findTargetIdx(state,RefPos);
    
    % 计算控制量
    delta = stanley_control(target_idx,state,latError,RefPos,k);
    
    % 计算加速度
    a = Kp* (targetSpeed-state(4));
    
    % 更新状态量
    state_new = UpdateState(a,state,delta,dt,L);
    state = state_new;
    
    % 保存每一步的实际状态量
    state_actual(end+1,:) = state_new;
end

% 画图
figure
plot(path(:,1), path(:,2), 'b');
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
hold on
for i = 1:size(state_actual,1)
    scatter(state_actual(i,1), state_actual(i,2),150, '.r');
    pause(0.01)
end
legend('规划车辆轨迹', '实际行驶轨迹')

%  保存
path_stanley = state_actual(:,1:2);
save path_stanley.mat path_stanley;

%% 首先在参考轨迹上搜索离当前位置最近的点
function [target_idx,latError] = findTargetIdx(state,RefPos)
for i = 1:size(RefPos,1)
    d(i,1) = norm(RefPos(i,:) - state(1:2));
end
[latError_temp,target_idx] = min(d);  % 找到距离当前位置最近的一个参考轨迹点的序号

if state(2) < RefPos(target_idx,2)    % 当前位置纵坐标小于参考轨迹点纵坐标时
    latError = -latError_temp;
else
    latError = latError_temp;
end
end

%% 获得控制量
function delta = stanley_control(target_idx,state,latError,RefPos,k)
sizeOfRefPos = size(RefPos,1);
if target_idx < sizeOfRefPos-5
    Point = RefPos(target_idx+5,1:2);  % 注意，target_idx往前第5个视为
else
    Point = RefPos(end,1:2);
end

theta_fai = pipi(atan((Point(2)-state(2))/(Point(1)-state(1))) -state(3));
theta_y = atan(k*latError / state(4));

% 前轮转角
delta = theta_fai + theta_y;
end


%% 更新状态量
function state_new = UpdateState(a,state_old,delta,dt,L)
state_new(1) =  state_old(1) + state_old(4)*cos(state_old(3))*dt; %纵向坐标
state_new(2) =  state_old(2) + state_old(4)*sin(state_old(3))*dt; %横向坐标
state_new(3) =  state_old(3) + state_old(4)*dt*tan(delta)/L;      %航向角
state_new(4) =  state_old(4) + a*dt;                              %纵向速度
end
