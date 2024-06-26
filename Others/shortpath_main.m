%% 此函数是主函数
clc,clear
%% 网络参数
load dis_data.mat;    % 按弧编号的距离
W = zeros(24,24);    % 初始化邻接矩阵W
W(1,[2,3])=dis_data(1:2);
W(2,[1,6])=dis_data(3:4);
W(3,[1,4,12])=dis_data(5:7);
W(4,[3,5,11])=dis_data(8:10);
W(5,[4,6,9])=dis_data(11:13);
W(6,[2,5,8])=dis_data(14:16);
W(7,[8,18])=dis_data(17:18);
W(8,[6,7,9,16])=dis_data(19:22);
W(9,[5,8,10])=dis_data(23:25);
W(10,[9,11,15,16,17])=dis_data(26:30);
W(11,[4,10,12,14])=dis_data(31:34);
W(12,[3,11,13])=dis_data(35:37);
W(13,[12,24])=dis_data(38:39);
W(14,[11,15,23])=dis_data(40:42);
W(15,[10,14,19,22])=dis_data(43:46);
W(16,[8,10,17,18])=dis_data(47:50);
W(17,[10,16,19])=dis_data(51:53);
W(18,[7,16,20])=dis_data(54:56);
W(19,[15,17,20])=dis_data(57:59);
W(20,[18,19,21,22])=dis_data(60:63);
W(21,[20,22,24])=dis_data(64:66);
W(22,[15,20,21,23])=dis_data(67:70);
W(23,[14,22,24])=dis_data(71:73);
W(24,[13,21,23])=dis_data(74:76);
W(1,3)=-1;

W(W==0)= 10000;   %将邻接矩阵中0的元素替换为一个特别大的数10000，表示不连接距离均视为无穷大
W(diag(true(size(W,1),1))) = 0;    % 把对角线元素选出来替换为0，自己与自己的距离当然为0
o = mod(18,24);   % 指定起点
d = mod(18+18,24);   % 指定终点
%% 0-1整数规划模型求最短路径
% [P,dis] = shortpath_guihua(W,o,d);

%% 迪杰斯特拉算法
shortpath_Dijkstra(W);

%% 弗洛伊德算法
% shortpath_Floyd(W);












