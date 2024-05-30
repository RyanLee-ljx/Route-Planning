# 本代码为改进Astar算法
"""
改进点：
1.堆数据结构   速度提升
2.新代价函数（选）
"""
import heapq
import numpy as np
from matplotlib import pyplot as plt
import time
import math


class Node:
    def __init__(self, **kwargs):
        self.f_value = None
        self.position = kwargs.get('position')  # 用get方法，当没有给出改键值时，自动补为None！！！
        self.g_value = kwargs.get('g')
        self.h_value = kwargs.get('h')
        self.parent = kwargs.get('parent')

    def cal_f(self):
        self.f_value = self.g_value + self.h_value
        # self.f_value = f_function(self.h_value, self.g_value)     # 使用改进函数

    def __lt__(self, other):    # 用于自定义比较两个Node类间的大小，通过f_value
        return self.f_value < other.f_value
        # python富比较方法：自定义进行类比较


def heuristic_function(final, position):
    # 欧氏距离
    f = math.sqrt((final[0] - position[0]) ** 2 + (final[1] - position[1]) ** 2)
    # f = abs(final[0] - position[0]) + abs(final[1] - position[1])
    return int(f * 10)


def f_function(h_value, g_value):
    # 欧氏距离
    if h_value < 0.2 * g_value:
        f_value = g_value + 0.2 * h_value
    else:
        f_value = g_value + 3 * h_value
    # f = abs(final[0] - position[0]) + abs(final[1] - position[1])
    return round(f_value)


def draw_map(map, star, end, path, ope, clo):
    p = np.array(path)
    idx = p[:, 0]
    idy = p[:, 1]
    map[idy, idx] = 2  # 路径标记为黄色   注意y坐标才是行，x坐标是列
    explore_list = []
    for node in clo + ope:
        if node.position not in path:
            explore_list.append(node.position)
    explore_lists = np.array(explore_list)
    idx_explore = explore_lists[:, 0]
    idy_explore = explore_lists[:, 1]
    map[idy_explore, idx_explore] = 3  # y坐标是行， x坐标是列
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
    ax.imshow(map, cmap=cmap, origin='upper')  # origin：原点位置  extend：x，y范围
    ax.plot(idx, idy, 'r-')  # 路线
    ax.xaxis.tick_top()
    ax.xaxis.set_label_position('top')
    ax.tick_params(axis='both', which='major', labelsize=len(map[0]) * 3 / 25)  # 设置主要刻度字体大小
    ax.tick_params(axis='both', which='minor', labelsize=len(map) * 3 / 25)  # 设置次要刻度字体大小
    # 设置主要刻度线和次要刻度线
    plt.show()
    return fig


def obstacle_process(map):
    obstacle_id = []
    for i in range(0, 40, 1):
        obstacle_id.append((10, i))
    for i in range(10, 50, 1):
        obstacle_id.append((20, i))
    for i in range(20, 30, 1):
        obstacle_id.append((30, i))
    obstacle_id = np.array(obstacle_id)
    obstacle_x = obstacle_id[:, 0]
    obstacle_y = obstacle_id[:, 1]
    map[obstacle_y, obstacle_x] = 1  # 障碍为1
    return map


def astar(start, final, map):
    # 创建起点
    start_node = Node(position=start, g=0, h=heuristic_function(final,start))
    start_node.cal_f()
    # 创建openlist 和 closelist
    open_list = []
    close_list = []
    # 把起点加入堆中
    heapq.heappush(open_list, (start_node.f_value, start_node))   # 生成堆，并将节点按照f_value进行堆放，自动形成最小堆，
    # 放进去的是一个tuple(int, Node)类型

    while open_list:
        value, current_node = heapq.heappop(open_list)   # 取出f最小的，并弹出堆
        close_list.append(current_node)
        neighbours = [(-1, 1), (-1, 0), (-1, -1), (0, 1), (0, -1), (1, 1), (1, 0), (1, -1)]
        dist = [14, 10, 14]
        # 开始搜索
        for neighbour in neighbours:
            n = (current_node.position[0] + neighbour[0], current_node.position[1] + neighbour[1])  # 当前邻居位
            open_node = next((node for node in open_list if node[1].position == n), None)
            close_node = next((node for node in close_list if node.position == n), None)
            # 判断边界
            if n[0] < 0 or n[0] >= len(map[0]) or n[1] < 0 or n[1] >= len(map):
                continue
            elif map[n[1]][n[0]] == 1:   # 判断障碍
                continue
            elif close_node:   # 在close中就跳过
                continue
            elif open_node:
                open_node = open_node[1]
                if current_node.g_value + dist[abs(neighbour[0])+abs(neighbour[1])] < open_node.g_value:
                    open_node.g_value = current_node.g_value + dist[abs(neighbour[0])+abs(neighbour[1])]
                    open_node.cal_f()    # 重新计算f值
                    open_node.parent = current_node    # 更新父节点
            else:
                neighbour_node = Node(position=n, g=current_node.g_value + dist[abs(neighbour[0]) + abs(neighbour[1])]
                                      , h=heuristic_function(final, n), parent=current_node)
                neighbour_node.cal_f()
                heapq.heappush(open_list, (neighbour_node.f_value, neighbour_node))  # 加入堆中，必须有Node类__lt__富比较方法才行

        # 判断终止+路径回溯
        if next((node for node in open_list if node[1].position == final), None):
            path = [final]
            par = next((node for node in open_list if node[1].position == final), None)
            par = par[1].parent
            while par.position != start_node.position:
                path.append(par.position)
                par = par.parent
            path.append(start_node.position)
            path = path[::-1]  # 路径反转
            return path, open_list, close_list


if __name__ == '__main__':
    Map_original = np.zeros((50, 50), dtype=np.int16)
    Map_obstacle = obstacle_process(Map_original)
    star = (1, 1)
    end = (41, 20)
    start_time = time.perf_counter()
    path, ope, clo = astar(star, end, Map_obstacle)
    op = []
    for i in ope:
        op.append(i[1])  # 提取出node类
    end_time = time.perf_counter()
    print(f'程序用时：{end_time}')
    if not path:
        print('No path available')
    else:
        print(path)
    final_node = next((node for node in op if node.position == end), None)
    print(f'最短距离{final_node.g_value}')
    draw_map(Map_obstacle, star, end, path, op, clo)









