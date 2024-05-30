function [P,dis,t] = shortpath_guihua(Wij,o,d)
tic
%shortpath_guihua 此函数通过建立0-1整数规划模型求解起点o至终点d的最短路长及最短路径
%   输入：W:邻接矩阵  o：指定起点  d：指定终点  t:运行时间
%   输出：P：最短路径（矩阵）  dis：最短路长
%   基于问题求解方式
n = size(Wij,1);   % 顶点数
if (o<=0||d<=0)
    disp('输入有误，顶点序号应大于0');
elseif(o>n||d>n)
    disp('输入有误，起点/终点序号不可超过最大序号');
else
    prob = optimproblem;
    Xij = optimvar('x',n,n,'Type','integer','LowerBound',0,'UpperBound',1);    % Xij 表示从顶点i到顶点j的弧是否在最短路径上，若在则为1否则为0
    prob.Objective = sum(sum(Wij.*Xij));    % 目标函数
    con1 = optimconstr(n-2);    % 约束条件1为对于除起点与终点外所有点，顶点的入度等于出度（Xij条件下）
    for i =setdiff([1:n],[o,d])   % setdiff函数返回 A 中存在但 B 中不存在的数据，因此可以得到除起点与终点外所有顶点入度等于出度的约束条件，实际上约束条件1中有n-2个约束条件
        con1(i) = sum(Xij(i,:))==sum(Xij(:,i));
    end
    prob.Constraints.con1 = con1;   % 约束条件1
    prob.Constraints.con2 = [sum(Xij(o,:))==1; sum(Xij(:,o))==0; sum(Xij(:,d))==1; sum(Xij(d,:))==0];   % 约束条件2：对于起点，入度为0，出度为1；对于终点，出度为0，入度为1，故共有4个约束条件
    [sol,fval]= solve(prob);
    vertex=sol.x;    % 得到最优解（顶点集矩阵）
    % 以下寻找最短路径
    m = o;  % 从起点开始
    P = [o];  % 最短路径从起点o开始
    while m~=d    % 从起点出发，找到一个点后再将其作为新起点找最短路径（vetex为1的点），直至到终点
        [~,m] = find(vertex(m,:));
        P = [P;m];
    end    
    dis = fval;
    disp(['0-1规划模型最短路长为：',num2str(dis)]);
    disp('0-1规划模型最短路径为')
    disp(P');
    % 可视化
    W = Wij; 
    W(W==100000) = 0;   % 将10000替换为0，不然会影响画图
    figure(2);
    G = graph(W);
    plot(G, 'EdgeLabel', G.Edges.Weight, 'linewidth', 2);    % 原始有向图
    set( gca, 'XTick', [], 'YTick', [] );
    path_plot = plot(G, 'EdgeLabel', G.Edges.Weight, 'linewidth', 2,'EdgeFontSize',8.5,'NodeFontSize',10);  % 用path_plot作为图像句柄并且调结点编号大小，弧大小等
    highlight(path_plot, P, 'EdgeColor', 'r')   %用红色标记最短路径
    highlight(path_plot,o ,'NodeColor','g','MarkerSize',13,'EdgeFontSize',8.5,'NodeFontSize',13,'Marker','hexagram');   % 标记起点为绿色
    highlight(path_plot,d ,'NodeColor',[255/255, 230/255, 0],'MarkerSize',13,'EdgeFontSize',8.5,'NodeFontSize',13,'Marker','pentagram');   % 标记终点为黄色
    title(['0-1规划模型:  起点',num2str(o),'至终点',num2str(d),'的最短路长为',num2str(dis)]);
end
t = toc; % 计时
end










