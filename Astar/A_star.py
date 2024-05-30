"""
本代码为原始Astar算法
"""
import math
import time
import numpy as np
from matplotlib import pyplot as plt

'''
Astar算法流程：
划分空间：栅格化或者用三角形等其他形状将空间划分为二维（此处只讨论二维）数组，类似于元胞自动机，每一个方格代表一个位置，根据地图信息，将方格标记为可行或者不可行（即确定障碍），未来路径就是一系列可行方格的有序集合。
启发函数/估价函数（heuristic function）：路径问题，自然而然会想到的是选取能够衡量远近的函数，一般采用曼哈顿距离或者欧氏距离，当然也有其他的下一节会深入讲解。
开放列表（open list） & 关闭列表（closed list）：open list 用于存放待检测的点，closed list 用于存放已经检查过的点（有一点类似于运筹学里的求解最大流的算法？）。
确定起止点，最大迭代次数等参数。
2．迭代
(1)将起点A添加到open列表中
(2)检查open列表，选取花费F最小的节点M（检查M如果为终点是则结束寻路，如果open列表没有则寻路失败，直接结束）。对于与M相邻的每一节点N
如果N是阻挡障碍，那么不管它。
如果N在closed列表中，那么不管它
如果N不在open列表中：添加它然后计算出它的花费F(n)=G+H。
如果N已在open列表中：当我们使用当前生成的路径时，检查F花费是否更小（实际上是检查G，因为N的H是固定的，只有G可以改变，G的比较方法：M的G加上到N节点的距离（10/14）再与N的G比较）。如果是，更新它的花费F和它的父节点。
一个node具有的属性：位置、g值、h值、f值、父亲节点
(3)重复2，3步。
(4)停止，当你把终点加入到了 openlist 中，此时路径已经找到了或者 openlist 是空的，此时没有路径。
'''

'''
注意，即使position一样，创建的对象也是不一样的，故需要检查邻点是否已经创建：
next() 函数用于获取可迭代对象（例如列表、生成器等）中的下一个元素。它接受两个参数：一个是可迭代对象，另一个是默认值（可选）。
它会返回可迭代对象中的下一个元素，如果没有下一个元素，则返回默认值（如果提供了默认值），否则会抛出 StopIteration 异常。
next((node for node in open_list if node.position==n), None): 寻找openlist中是否有位置等于n的node
'''

"""
def init(self, *args, **kwargs): 其含义代表什么？
这种写法代表这个方法接受任意个数的参数
如果是没有指定key的参数，比如单单’apple’，‘people’，即为无指定，则会以list的形式放在args变量里面
如果是有指定key的参数，比如item='apple’这种形式，即为有指定，则会以dict的形式放在kwargs变量里面
"""

'''
一定要注意，矩阵中的索引坐标与imshow图像所展示的索引是相反的
即imshow中是先看列（x），再看行（y），组成(x,y)，即（列，行）
但矩阵中是，先看行，再看列
所以imshow的x坐标对应map列（1），y对应map行（0）
行是y  列是x.
如起点(41, 10)，在矩阵中引用应该是map[10, 41]
'''


class Node:
    def __init__(self, **kwargs):
        self.f_value = None
        self.position = kwargs.get('position')  # 用get方法，当没有给出改键值时，自动补为None！！！
        self.g_value = kwargs.get('g')
        self.h_value = kwargs.get('h')
        self.parent = kwargs.get('parent')

    def cal_f(self):
        self.f_value = self.g_value + self.h_value
        # self.f_value = f_function(self.h_value, self.g_value)


def heuristic_function(final, position):
    # 欧氏距离
    # f = math.sqrt((final[0] - position[0]) ** 2 + (final[1] - position[1]) ** 2)
    f = abs(final[0] - position[0]) + abs(final[1] - position[1])
    return f


def draw_map(map, star, end, path, ope, clo):
    p = np.array(path)
    idx = p[:, 0]
    idy = p[:, 1]
    map[idy, idx] = 2  # 路径标记为黄色 注意y坐标才是行，x坐标是列
    explore_list = []
    for node in ope:
        if node.position not in path:
            explore_list.append(node.position)
    for position in clo:
        if position not in path:
            explore_list.append(position)
    explore_lists = np.array(explore_list)
    if explore_lists.size > 0:  # 不为空
        idx_explore = explore_lists[:, 0]
        idy_explore = explore_lists[:, 1]
        map[idy_explore, idx_explore] = 3  # y坐标是行，x坐标是列
    palette = ['#FFFFFF', '#301918', '#B15858', '#D2E8FB']
    cmap = plt.cm.colors.ListedColormap(palette)
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.xaxis.set_major_locator(plt.MultipleLocator(1))
    ax.xaxis.set_minor_locator(plt.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(plt.MultipleLocator(1))
    ax.yaxis.set_minor_locator(plt.MultipleLocator(0.5))
    # 显示网格线（仅显示次要刻度上的网格线）
    ax.grid(which='minor', linestyle='--', linewidth=0.5, color='gray')
    ax.set_title("Planning Route")
    ax.set_xlim(-0.5, np.size(map, 1) - 0.5)
    ax.set_ylim(np.size(map, 0) - 0.5, -0.5)
    ax.plot(star[0], star[1], 'ro')  # 起点
    ax.plot(end[0], end[1], 'g*')  # 终点
    # 在imshow中添加aspect='auto'参数
    ax.imshow(map, cmap=cmap, origin='upper', aspect='auto')  # 修改了这里
    ax.plot(idx, idy, 'r-')  # 路线
    ax.xaxis.tick_top()
    ax.xaxis.set_label_position('top')
    ax.tick_params(axis='both', which='major', labelsize=len(map[0]) * 3 / 25)  # 设置主要刻度字体大小
    ax.tick_params(axis='both', which='minor', labelsize=len(map) * 3 / 25)  # 设置次要刻度字体大小
    # 设置主要刻度线和次要刻度线
    # 使用subplots_adjust调整子图参数
    plt.subplots_adjust(left=0.2, right=0.8, bottom=0.05, top=0.95)  # 调整空白大小
    plt.show()
    return fig


def obstacle_process(map):
    obstacle_id = []
    k1 = 0
    for i in range(0, 8, 1):
        k2 = 1
        for j in range(0, 13, 1):
            x = np.arange(k1, k1 + 10, 1)
            y = np.arange(k2, k2 + 2, 1)
            xy = [(a, b) for a in x for b in y]
            obstacle_id.append(xy)
            k2 = k2 + 3
        k1 = k1 + 12
    obstacle_id = np.array(obstacle_id)
    for node in obstacle_id:
        obstacle_x = node[:, 0]
        obstacle_y = node[:, 1]
        map[obstacle_x, obstacle_y] = 1  # 障碍为1
    return map


def astar(start, final, map):
    start_node = Node(position=start, g=0, h=heuristic_function(final, start))  # 初始化起终点
    start_node.cal_f()
    open_list = []
    close_list = []

    open_list.append(start_node)  # 把起点放入open

    while open_list:
        current_node = min(open_list, key=lambda x: x.f_value)  # 比较open中每一个节点的node，找到f最小的node
        open_list.remove(current_node)
        close_list.append(current_node)
        # min函数的高级用法：min(iterable, key=None) 指定一个函数，用于从每个可迭代对象中提取一个用于比较的键值
        neighbours = [(-1, 0), (0, 1), (0, -1), (1, 0)]
        dist = 1
        for neighbour in neighbours:
            n = (current_node.position[0] + neighbour[0], current_node.position[1] + neighbour[1])  # 当前邻居位置
            close_node = next((node for node in close_list if node.position == n), None)  # close中是否有相同节点
            open_node = next((node for node in open_list if node.position == n), None)  # open中是否有相同节点
            if n[0] < 0 or n[0] >= len(map[0]) or n[1] < 0 or n[1] >= len(map):  # 边界   len(map)行数，
                # len(map[0]),返回第一行元素个数，即列数
                continue
            elif map[n[1]][n[0]] == 1:  # 判断障碍，1是障碍,如果是障碍，跳过   n是地图上坐标，n[0]是x坐标对应列，n[1]是y坐标，对应行
                continue

            elif close_node:  # 是否在close中
                continue

            elif open_node:
                if current_node.g_value + dist < open_node.g_value:
                    open_node.g_value = current_node.g_value + dist
                    open_node.cal_f()
                    open_node.parent = current_node

            else:
                neighbour_node = Node(position=n)  # 若节点不在open中，则创建并加入open，并记录其g，h，f，parent
                open_list.append(neighbour_node)
                neighbour_node.g_value = current_node.g_value + dist
                # 对角是14，水平竖直为10
                neighbour_node.h_value = heuristic_function(final, neighbour_node.position)
                neighbour_node.parent = current_node
                neighbour_node.cal_f()

        if next((node for node in open_list if node.position == final), None):  # 找到终点
            path = [final]
            par = next((node for node in open_list if node.position == final), None).parent
            while par.position != start_node.position:
                path.append(par.position)
                par = par.parent
            path.append(start_node.position)
            path = path[::-1]  # 路径反转
            return path, open_list, close_list


if __name__ == '__main__':
    Map_original = np.zeros((96, 39), dtype=np.int16)
    Map_obstacle = obstacle_process(Map_original)
    star = (0, 95) # 前面是x坐标，后面是y坐标，左上角为原点
    end = (36, 9)
    start_time = time.perf_counter()
    path, ope, clo = astar(star, end, Map_obstacle)
    end_time = time.perf_counter()
    duration = end_time - start_time
    print(f'程序用时：{duration:.4f}s')
    if not path:
        print('No path available')
    else:
        print(path)
    final_node = next((node for node in ope if node.position == end), None)
    print(f'最短距离{final_node.g_value}')
    draw_map(Map_obstacle, star, end, path, ope, clo)
