function createfigure(YMatrix1)
%CREATEFIGURE(YMATRIX1)
%  YMATRIX1:  y 数据的矩阵

%  由 MATLAB 于 01-Nov-2018 21:44:03 自动生成

% 创建 figure
figure1 = figure;

% 创建 axes
axes1 = axes('Parent',figure1);
box(axes1,'on');
hold(axes1,'on');

% 使用 plot 的矩阵输入创建多行
plot1 = plot(YMatrix1,'Parent',axes1);
set(plot1(1),'DisplayName','simout(:,1)');
set(plot1(2),'DisplayName','simout(:,2)');
set(plot1(3),'DisplayName','simout(:,3)');


