close all;
clear;
clc;

%% Read .txt
[ReadFileName,ReadFilePathName,ReadFilterIndex] = ...
        uigetfile('*.txt;*.log','获取txt或者log', '');

str = [ReadFilePathName ReadFileName];

fid = fopen(str, 'r');


%% Searching symbol: Tracking Analysis
RefTra_Index      = 1;
RefPoi_Index      = 1;
TraErr_Index      = 1;
ConCom_Index      = 1;
StaRob_Index      = 1;

RefTra_Ide      = '%s %s %f %s %f %s %f %s %f %s %f %s %f %s %f';
RefPoi_Ide      = '%s %s %f %s %f %s %f %s %f %s %f %s %f %s %f';
TraErr_Ide      = '%s %s %f %s %f %s %f %s %f %s %f';
ConCom_Ide      = '%s %s %s %s %f %s %f %s %f';
StaRob_Ide      = '%s %s %f %s %f %s %f %s %f %s %f %s %f %s %f';


%% Searching symbol: Planning Analysis
RefTrajPlan_Index   = 1;
GoalState_Index     = 1;
GlobalRefPoint_Index = 1;
PlanCmd_Index       = 1;

RefTrajPlan_Ide   = "%s %s %f %s %f %s %f %s %f %s %f %s %f %s %f %s %f";
GoalState_Ide     = "%s %s %f %s %f %s %f %s %f %s %f %s %f %s %f %s %f";
GlobalRefPoint_Ide = "%s %s %f %s %f %s %f %s %f %s %f %s %f %s %f";
PlanCmd_Ide       = "%s %s %f %s %f %s %f %s %f";


%% Searching Info: Tracking && Planning
i = 0;
while ~feof(fid)
    i = i + 1;

    t1 = fgetl(fid);

    id_MOTOR = strfind(t1, '[reference_trajectory]');
    if ~isempty(id_MOTOR)
        data = textscan(t1, RefTra_Ide);
        RefTra(RefTra_Index, (1 : 12)) = data(1, (4 : 15));
        RefTra_Index = RefTra_Index + 1;
    end

    id_MOTOR = strfind(t1, '[reference_point]');
    if ~isempty(id_MOTOR)
        data = textscan(t1, RefPoi_Ide);
        RefPoi(RefPoi_Index, (1 : 12)) = data(1, (4 : 15));
        RefPoi_Index = RefPoi_Index + 1;
    end

    id_MOTOR = strfind(t1, '[tracking_error]');
    if ~isempty(id_MOTOR)
        data = textscan(t1, TraErr_Ide);
        TraErr(TraErr_Index, (1 : 8)) = data(1, (4 : 11));
        TraErr_Index = TraErr_Index + 1;
    end

    id_MOTOR = strfind(t1, '[control_command]');
    if ~isempty(id_MOTOR)
        data = textscan(t1, ConCom_Ide);
        ConCom(ConCom_Index, (1 : 6)) = data(1, (4 : 9));
        ConCom_Index = ConCom_Index + 1;
    end

    id_MOTOR = strfind(t1, '[state_of_robot]');
    if ~isempty(id_MOTOR)
        data = textscan(t1 , StaRob_Ide);
        StaRob(StaRob_Index, (1 : 12)) = data(1, (4 : 15));
        StaRob_Index = StaRob_Index + 1;
    end

    id_MOTOR = strfind(t1, '[reference_trajectory_planning]');
    if ~isempty(id_MOTOR)
        data = textscan(t1 , RefTrajPlan_Ide);
        if cell2mat(data(17)) < 2
            RefTrajPlan(RefTrajPlan_Index, (1 : 14)) = data(1, (4 : 17));
        end
        RefTrajPlan_Index = RefTrajPlan_Index + 1;
    end
    
    id_MOTOR = strfind(t1, '[goal_state_planning]');
    if ~isempty(id_MOTOR)
        data = textscan(t1 , GoalState_Ide);
        GoalState(GoalState_Index, (1 : 14)) = data(1, (4 : 17));
        GoalState_Index = GoalState_Index + 1;
    end
    
    id_MOTOR = strfind(t1, '[plainning_global_reference_point]');
    if ~isempty(id_MOTOR)
        data = textscan(t1 , GlobalRefPoint_Ide);
        GlobalRefPoint(GlobalRefPoint_Index, (1 : 12)) = data(1, (4 : 15));
        GlobalRefPoint_Index = GlobalRefPoint_Index + 1;
    end
    
    id_MOTOR = strfind(t1, '[planning_control_command]');
    if ~isempty(id_MOTOR)
        data = textscan(t1 , PlanCmd_Ide);
        PlanCmd(PlanCmd_Index, (1 : 6)) = data(1, (4 : 9));
        PlanCmd_Index = PlanCmd_Index + 1;
    end
end

fprintf('%s 读取完成\n',ReadFileName);
fclose(fid);


%% Plot figure: Tracking_reference_trajectory&&tracking_error
% figure('name','tracking_error');
% subplot(3, 2, 1);
% plot(cell2mat(RefTra(:, 2)), cell2mat(RefTra(:, 4)), 'r', ...
%      cell2mat(StaRob(:, 2)), cell2mat(StaRob(:, 4)), 'b'); grid on;
% xlabel('横向位置(米)'); ylabel('纵向位置(米)'); set(gca, 'FontSize', 16);
% title('运动轨迹'); legend('规划轨迹', '真实轨迹');
% 
% subplot(3, 2, 2);
% plot(cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 8)), 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 8)), 'b'); grid on;
% xlabel('时间(秒)'); ylabel('速度(米/秒)'); set(gca, 'FontSize', 16);
% title('速度-时间曲线'); legend('规划速度', '真实速度');
% 
% subplot(3, 2, 3);
% plot(cell2mat(TraErr(:, 8)), cell2mat(TraErr(:, 2)) * 1000); grid on;
% xlabel('时间(秒)'); ylabel('轨迹系纵向误差(毫米)'); set(gca, 'FontSize', 16);
% title('轨迹系纵向误差-时间曲线');
% 
% subplot(3, 2, 4);
% plot(cell2mat(TraErr(:, 8)), cell2mat(TraErr(:, 4)) * 1000); grid on;
% xlabel('时间(秒)'); ylabel('轨迹系横向误差(毫米)'); set(gca, 'FontSize', 16);
% title('轨迹系横向误差-时间曲线');
% 
% subplot(3, 2, 5);
% plot(cell2mat(ConCom(:, 6)),  cell2mat(ConCom(:, 2)), 'r', ...
%      cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 8)), 'b', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 8)), 'g'); grid on;
% xlabel('时间(秒)'); ylabel('速度(米/秒)'); set(gca, 'FontSize', 16);
% title('速度-时间曲线'); legend('速度指令','规划速度','真实速度');
% 
% subplot(3, 2, 6);
% plot(cell2mat(ConCom(:, 6)),  cell2mat(ConCom(:, 4)) * 57.3,  'r',...
%      cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 10)) * 57.3, 'b',...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 10)) * 57.3, 'g'); grid on;
% xlabel('时间(秒)'); ylabel('横摆角速度(度/秒)'); set(gca, 'FontSize', 16);
% title('横摆角速度-时间曲线'); legend('横摆角速度指令','规划横摆角速度','真实横摆角速度');


% % Plot figure: Control_state_of_robot
% figure('name','state_of_robot');
%
% subplot(3, 2, 1);
% plot(cell2mat(RefTra(:, 2)), cell2mat(RefTra(:, 4)), 'r', ...
%      cell2mat(StaRob(:, 2)), cell2mat(StaRob(:, 4)), 'b'); grid on;
% xlabel('横向位置(米)'); ylabel('纵向位置(米)'); set(gca, 'FontSize', 16);
% title('运动轨迹'); legend('规划轨迹', '真实轨迹');
%
% subplot(3, 2, 2);
% plot(cell2mat(ConCom(:, 6)),  cell2mat(ConCom(:, 2)), 'r', ...
%      cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 8)), 'b', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 8)), 'g'); grid on;
% xlabel('时间(秒)'); ylabel('速度(米/秒)'); set(gca, 'FontSize', 16);
% title('速度-时间曲线'); legend('速度指令','规划速度','真实速度');
%
% subplot(3, 2, 3);
% plot(cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 2)), 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 2)), 'b'); grid on;
% xlabel('时间(秒)'); ylabel('横向位置(米)'); set(gca, 'FontSize', 16);
% title('速度-横向位置曲线'); legend('规划', '真实');
%
% subplot(3, 2, 4);
% plot(cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 4)), 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 4)), 'b'); grid on;
% xlabel('时间(秒)'); ylabel('纵向位置(米)'); set(gca, 'FontSize', 16);
% title('时间-纵向位置曲线'); legend('规划', '真实');
%
% subplot(3, 2, 5);
% plot(cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 6)) * 57.3, 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 6)) * 57.3, 'b'); grid on;
% xlabel('时间(秒)'); ylabel('横摆角(度)'); set(gca, 'FontSize', 16);
% title('时间-横摆角曲线'); legend('规划', '真实');
%
% subplot(3, 2, 6);
% plot(cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 10)) * 57.3, 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 10)) * 57.3, 'b'); grid on;
% xlabel('时间(秒)'); ylabel('横摆角速度(度每秒)'); set(gca, 'FontSize', 16);
% title('时间-横摆角速度曲线'); legend('规划', '真实');
%
% % Plot figure: Control_reference_point
% figure('name','reference_point');
%
% subplot(3, 2, 1);
% plot(cell2mat(RefPoi(:, 2)), cell2mat(RefPoi(:, 4)), 'r', ...
%      cell2mat(StaRob(:, 2)), cell2mat(StaRob(:, 4)), 'b'); grid on;
% xlabel('横向位置(米)'); ylabel('纵向位置(米)'); set(gca, 'FontSize', 16);
% title('运动轨迹'); legend('规划轨迹', '真实轨迹');
%
% subplot(3, 2, 2);
% plot(cell2mat(ConCom(:, 6)),  cell2mat(ConCom(:, 2)), 'r', ...
%      cell2mat(RefPoi(:, 12)), cell2mat(RefPoi(:, 8)), 'b', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 8)), 'g'); grid on;
% xlabel('时间(秒)'); ylabel('速度(米/秒)'); set(gca, 'FontSize', 16);
% title('速度-时间曲线'); legend('速度指令','规划速度','真实速度');5
%
% subplot(3, 2, 3);
% plot(cell2mat(RefPoi(:, 12)), cell2mat(RefPoi(:, 2)), 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 2)), 'b'); grid on;
% xlabel('时间(秒)'); ylabel('横向位置(米)'); set(gca, 'FontSize', 16);
% title('时间-横向位置曲线'); legend('规划', '真实');
%
% subplot(3, 2, 4);
% plot(cell2mat(RefPoi(:, 12)), cell2mat(RefPoi(:, 4)), 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 4)), 'b'); grid on;
% xlabel('时间(秒)'); ylabel('纵向位置(米)'); set(gca, 'FontSize', 16);
% title('时间-纵向位置曲线'); legend('规划', '真实');
%
% subplot(3, 2, 5);
% plot(cell2mat(RefPoi(:, 12)), cell2mat(RefPoi(:, 6)) * 57.3, 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 6)) * 57.3, 'b'); grid on;
% xlabel('时间(秒)'); ylabel('横摆角(度)'); set(gca, 'FontSize', 16);
% title('时间-横摆角曲线'); legend('规划', '真实');
%
% subplot(3, 2, 6);
% plot(cell2mat(RefPoi(:, 12)), cell2mat(RefPoi(:, 10)) * 57.3, 'r', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 10)) * 57.3, 'b'); grid on;
% xlabel('时间(秒)'); ylabel('横摆角速度(度每秒)'); set(gca, 'FontSize', 16);
% title('时间-横摆角速度曲线'); legend('规划', '真实');


%% Plot figure: MPC Planner Analysis
% figure('name','trajectory planning');
% subplot(3, 2, 1);
% plot(cell2mat(GlobalRefPoint(:, 2)), cell2mat(GlobalRefPoint(:, 4)), 'r', ...          %% 近似全局轨迹             plot
%        ...                                                                                                                   %% 真实全局轨迹散点图   plot
%        ...                                                                                                                   %% 近似局部轨迹             plot
%        cell2mat(RefTrajPlan(:, 2)), cell2mat(RefTrajPlan(:, 4)), 'g.', ...                    %% 真实局部轨迹散点图   plot
%        cell2mat(StaRob(:, 2)), cell2mat(StaRob(:, 4)), 'b', ...
%        cell2mat(GoalState(:, 2)), cell2mat(GoalState(:, 4)), 'black*'); grid on;
% xlabel('横向位置(米)'); ylabel('纵向位置(米)'); set(gca, 'FontSize', 16);
% title('运动轨迹'); legend('近似全局轨迹', '真实局部轨迹散点图', '小车轨迹', '停车点');
% 
% subplot(3, 2, 2);
% plot(cell2mat(GlobalRefPoint(:, 12)), cell2mat(GlobalRefPoint(:, 8)), 'r', ...
%        cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 8)), 'g', ...
%        cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 8)), 'b'); grid on;
% xlabel('时间(秒)'); ylabel('速度(米/秒)'); set(gca, 'FontSize', 16);
% title('速度-时间曲线'); legend('近似全局规划速度', '近似局部规划速度', '小车速度');
% 
% subplot(3, 2, 3);
% plot(cell2mat(TraErr(:, 8)), cell2mat(TraErr(:, 2)) * 1000); grid on;
% xlabel('时间(秒)'); ylabel('轨迹系纵向误差(毫米)'); set(gca, 'FontSize', 16);
% title('轨迹系纵向误差-时间曲线');
% 
% subplot(3, 2, 4);
% plot(cell2mat(TraErr(:, 8)), cell2mat(TraErr(:, 4)) * 1000); grid on;
% xlabel('时间(秒)'); ylabel('轨迹系横向误差(毫米)'); set(gca, 'FontSize', 16);
% title('轨迹系横向误差-时间曲线');
% 
% subplot(3, 2, 5);
% plot(cell2mat(PlanCmd(:, 6)),  cell2mat(PlanCmd(:, 2)), 'r', ...
%      cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 8)), 'b', ...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 8)), 'g'); grid on;
% xlabel('时间(秒)'); ylabel('速度(米/秒)'); set(gca, 'FontSize', 16);
% title('速度-时间曲线'); legend('速度指令','规划速度','真实速度');
% 
% subplot(3, 2, 6);
% plot(cell2mat(PlanCmd(:, 6)),  cell2mat(PlanCmd(:, 4)) * 57.3,  'r',...
%      cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 10)) * 57.3, 'b',...
%      cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 10)) * 57.3, 'g'); grid on;
% xlabel('时间(秒)'); ylabel('横摆角速度(度/秒)'); set(gca, 'FontSize', 16);
% title('横摆角速度-时间曲线'); legend('横摆角速度指令','规划横摆角速度','真实横摆角速度');


%% Plot figure: MPC Planner && Controller Analysis
figure('name','trajectory planning');
subplot(3, 2, 1);
plot(cell2mat(GlobalRefPoint(:, 2)), cell2mat(GlobalRefPoint(:, 4)), 'r', ...          %% 近似全局轨迹             plot
       ...                                                                                                                   %% 真实全局轨迹散点图   plot
       ...                                                                                                                   %% 近似局部轨迹             plot
       cell2mat(RefTrajPlan(:, 2)), cell2mat(RefTrajPlan(:, 4)), 'g.', ...                    %% 真实局部轨迹散点图   plot
       cell2mat(StaRob(:, 2)), cell2mat(StaRob(:, 4)), 'b', ...
       cell2mat(GoalState(:, 2)), cell2mat(GoalState(:, 4)), 'black*'); grid on;
xlabel('横向位置(米)'); ylabel('纵向位置(米)'); set(gca, 'FontSize', 16);
title('运动轨迹'); legend('近似全局轨迹', '真实局部轨迹散点图', '小车轨迹', '停车点');

subplot(3, 2, 2);
plot(cell2mat(GlobalRefPoint(:, 12)), cell2mat(GlobalRefPoint(:, 8)), 'r', ...
       cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 8)), 'g', ...
       cell2mat(ConCom(:, 6)),  cell2mat(ConCom(:, 2)), 'y', ...
       cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 8)), 'b'); grid on;
xlabel('时间(秒)'); ylabel('速度(米/秒)'); set(gca, 'FontSize', 16);
title('速度-时间曲线'); legend('近似全局速度', '近似局部速度', '速度指令', '小车速度');

subplot(3, 2, 3);
plot(cell2mat(TraErr(:, 8)), cell2mat(TraErr(:, 2)) * 1000); grid on;
xlabel('时间(秒)'); ylabel('轨迹系纵向误差(毫米)'); set(gca, 'FontSize', 16);
title('轨迹系纵向误差-时间曲线');

subplot(3, 2, 4);
plot(cell2mat(TraErr(:, 8)), cell2mat(TraErr(:, 4)) * 1000); grid on;
xlabel('时间(秒)'); ylabel('轨迹系横向误差(毫米)'); set(gca, 'FontSize', 16);
title('轨迹系横向误差-时间曲线');

subplot(3, 2, 6);
plot(cell2mat(ConCom(:, 6)),  cell2mat(ConCom(:, 4)) * 57.3,  'r',...
       cell2mat(RefTra(:, 12)), cell2mat(RefTra(:, 10)) * 57.3, 'b',...
       cell2mat(StaRob(:, 12)), cell2mat(StaRob(:, 10)) * 57.3, 'g'); grid on;
xlabel('时间(秒)'); ylabel('横摆角速度(度/秒)'); set(gca, 'FontSize', 16);
title('横摆角速度-时间曲线'); legend('横摆角速度指令','近似局部横摆角速度','小车横摆角速度');