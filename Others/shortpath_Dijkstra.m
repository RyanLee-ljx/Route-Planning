function t = shortpath_Dijkstra(Wij,o,d)
%shortpath_Dijkstra 用迪杰斯特拉算法求解最短路径，可选择模式1只输入指定起点，输出该起点到其他各点的最短路长；或模式2输入指定起终点，输出该起终点最短路长以及相应最短路径。
% Wij：邻接矩阵
% 先检查权值是否有负数
tic
if(sum(sum(any (Wij<0))))    % 返回true或false
    error('错误！迪杰斯特拉算法权值边不可有负数');
else
    n = size(Wij,1);
    Node = zeros(n,3);  %结点矩阵，每一行代表一个结点，第一列为标记状态（0为为访问，1为访问），第二列为最短距离，第三列为双亲结点编号
    Node(:,2) = 10000;
    num = 1;  % 计数变量
    Node(o,2)=0;  %初始化起点距离
    Node(o,1)=1;  %标记访问
    k = o;   % 标记变量
    while(num < n)
        index = find(Node(:,1)==0);   % 找到未访问结点
        for i = index'    % 更新距离
            if((Node(k,2) + Wij(k,i)) < Node(i,2))
                Node(i,2) = Node(k,2) + Wij(k,i);
                Node(i,3) = k;
            end
        end
        [~,k] = min(Node(index,2)); %只在未访问结点中找最小值
        k = index(k); %找到对应的结点编号
        Node(k,1) = 1;  % 标记访问
        num = num+1;   % 完成一次最短路径过程
        if(Node(d,1)==1)    % 找到终点的最短路径就结束
            break;
        end
    end
    m = d;
    P = [d];
    while m~=o    % 回溯法，从终点到起点找最短路径
        m = Node(m,3);
        P = [m;P];  % 翻转
    end
    disp(['Dijkstra最短路长为：',num2str(Node(d,2))]); %输出最短距离
    disp('Dijkstra最短路径为');
    disp(P');
    % 可视化
    W = Wij;
    W(W==100000) = 0;   % 将10000替换为0，不然会影响画图
    figure(3);
    G = graph(W);
    plot(G, 'EdgeLabel', G.Edges.Weight, 'linewidth', 2);    % 原始有向图
    set( gca, 'XTick', [], 'YTick', [] );
    path_plot = plot(G, 'EdgeLabel', G.Edges.Weight, 'linewidth', 2,'EdgeFontSize',8.5,'NodeFontSize',10);  % 用path_plot作为图像句柄并且调结点编号大小，弧大小等
    highlight(path_plot, P, 'EdgeColor', 'r')   %用红色标记最短路径
    highlight(path_plot,o ,'NodeColor','g','MarkerSize',13,'EdgeFontSize',8.5,'NodeFontSize',13,'Marker','hexagram');   % 标记起点为绿色
    highlight(path_plot,d ,'NodeColor',[255/255, 230/255, 0],'MarkerSize',13,'EdgeFontSize',8.5,'NodeFontSize',13,'Marker','pentagram');   % 标记终点为黄色
    title(['Dijkstra: 起点',num2str(o),'至终点',num2str(d),'的最短路长为',num2str(Node(d,2))]);
end
t = toc;
end
