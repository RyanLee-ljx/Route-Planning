function t = shortpath_Floyd(Wij,o,d)
tic
%shortpWijth_Floyd 用佛洛依德算法求解最短路径，可选择模式1，输入各点间的最短路长；或模式2：输入指定起终点，输出该起终点最短路长以及相应最短路径。
%   Wij为邻接矩阵
% 初始化
n = size(Wij,1);
D = Wij;   % 初始化距离矩阵D，第一次就为邻接矩阵
P = zeros(n,n);
for i=1:n    % 初始化路径矩阵P
    for j=1:n
        P(i,j)=j;
    end
end
% 加入中间结点k，更新D和P
for k=1:n
    for i=1:n
        for j=1:n
            if D(i,k)+D(k,j)<D(i,j)
                D(i,j)=D(i,k)+D(k,j);
                P(i,j)=P(i,k);
            end
        end
    end
end
% 检查是否有路径存在
if D(o,d) == 10000
    error('没有从起点到终点的路径');
end
m = o;
M = [o];
while m~=d    % 从起点到终点找最短路径
    m = P(m,d);
    M = [M,m];
end
disp(['Floyd最短路长为：',num2str(D(o,d))]); %输出最短距离
disp('Floyd最短路径为');
disp(M);
% 可视化
W = Wij;
W(W==100000) = 0;   % 将10000替换为0，不然会影响画图
figure(4)
G = graph(W);
plot(G, 'EdgeLabel', G.Edges.Weight, 'linewidth', 2);    % 原始有向图
set( gca, 'XTick', [], 'YTick', [] );
path_plot = plot(G, 'EdgeLabel', G.Edges.Weight, 'linewidth', 2,'EdgeFontSize',8.5,'NodeFontSize',10);  % 用path_plot作为图像句柄并且调结点编号大小，弧大小等
highlight(path_plot, M, 'EdgeColor', 'r')   %用红色标记最短路径
highlight(path_plot,o ,'NodeColor','g','MarkerSize',13,'EdgeFontSize',8.5,'NodeFontSize',13,'Marker','hexagram');   % 标记起点为绿色
highlight(path_plot,d ,'NodeColor',[255/255, 230/255, 0],'MarkerSize',13,'EdgeFontSize',8.5,'NodeFontSize',13,'Marker','pentagram');   % 标记终点为黄色
title(['Floyd: 起点',num2str(o),'至终点',num2str(d),'的最短路长为',num2str(D(o,d))]);
t = toc;
end
