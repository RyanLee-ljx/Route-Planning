# 本代码为改进Astar算法
import heapq
import numpy as np
from matplotlib import pyplot as plt
import time
import math

'''
本代码为改进Astar
改进点：
1.数据结构
    open_list采用堆结构
    close_list采用哈希表
2.启发函数：
    避免对角穿过障碍，将代价函数改为：F(n) = G(n) + H(n) + E(n) ，注意此时节点判断不能通过判断G来判断F
    0.1%启发偏移量
'''


class Node:
    def __init__(self, **kwargs):
        self.f_value = None
        self.position = kwargs.get('position')  # 用get方法，当没有给出改键值时，自动补为None！！！
        self.g_value = kwargs.get('g')
        self.h_value = kwargs.get('h')
        self.e_value = 0
        self.parent = kwargs.get('parent')

    def __lt__(self, other):  # 用于自定义比较两个Node类间的大小，通过f_value
        return self.f_value < other.f_value
        # python富比较方法：自定义进行类比较


def heuristic_function(final, position):
    # 欧氏距离
    f = math.sqrt((final[0] - position[0]) ** 2 + (final[1] - position[1]) ** 2)
    # f = abs(final[0] - position[0]) + abs(final[1] - position[1])
    return round(f * 10)


def e_value_cal(new_parent_node, open_node, map):
    if new_parent_node is not None:
        direction = (open_node.position[0] - new_parent_node.position[0], open_node.position[1] -
                     new_parent_node.position[1])
        if (abs(direction[0]) + abs(direction[1])) == 2 and direction[0] < open_node.position[0] <= len(map[0]) and \
                direction[1] < open_node.position[1] <= len(map):
            if open_node.g_value > 500:
                e_value = (open_node.g_value + 0.8 * open_node.h_value) * 0.01 * (1 if map[open_node.position[1]][open_node.position[0] - direction[0]] == 1 or
                                  map[open_node.position[1] - direction[1]][open_node.position[0]] == 1 else 0)
            else:
                e_value = (open_node.g_value + 3 * open_node.h_value) * 0.01 * (1 if map[open_node.position[1]][open_node.position[0] - direction[0]] == 1 or
                                  map[open_node.position[1] - direction[1]][open_node.position[0]] == 1 else 0)
        else:
            e_value = 0
    else:
        e_value = 0
    return e_value


def f_value_cal(node, map):
    # if node.g_value > 500:  # 动态加权
    #     f = node.g_value + 0.8 * node.h_value
    # else:
    #     f = node.g_value + 3 * node.h_value
    f = node.g_value + node.h_value * 1.001   # 百分之0.1的偏移
    e_value = round(e_value_cal(node.parent, node, map))
    f_value = round(f + e_value)
    return e_value, f_value


def draw_map(map, star, end, path, ope, clo):
    p = np.array(path)
    idx = p[:, 0]
    idy = p[:, 1]
    map[idy, idx] = 2  # 路径标记为黄色   注意y坐标才是行，x坐标是列
    explore_list = []
    for node in ope:
        if node.position not in path:
            explore_list.append(node.position)
    for position in clo:
        if position not in path:
            explore_list.append(position)
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
    for i in range(5, 40, 1):
        obstacle_id.append((30, i))
    for i in range(10, 30, 1):
        obstacle_id.append((40, i))
    obstacle_id = np.array(obstacle_id)
    obstacle_x = obstacle_id[:, 0]
    obstacle_y = obstacle_id[:, 1]
    map[obstacle_y, obstacle_x] = 1  # 障碍为1
    return map


def astar(start, final, map):
    # 创建起点
    start_node = Node(position=start, g=0, h=heuristic_function(final, start))
    start_node.e_value, start_node.f_value = f_value_cal(start_node, map)
    # 创建openlist 和 closelist
    open_list = []
    close_list = dict()
    # 把起点加入堆中
    heapq.heappush(open_list, (start_node.f_value, start_node))  # 生成堆，并将节点按照f_value进行堆放，自动形成最小堆，
    # 放进去的是一个tuple(int, Node)类型

    while open_list:
        value, current_node = heapq.heappop(open_list)  # 取出f最小的，并弹出堆
        close_list[current_node.position] = current_node
        neighbours = [(-1, 1), (-1, 0), (-1, -1), (0, 1), (0, -1), (1, 1), (1, 0), (1, -1)]
        dist = [14, 10, 14]
        # 开始搜索
        for neighbour in neighbours:
            n = (current_node.position[0] + neighbour[0], current_node.position[1] + neighbour[1])  # 当前邻居位
            open_node = next((node for node in open_list if node[1].position == n), None)
            # 判断边界
            if n[0] < 0 or n[0] >= len(map[0]) or n[1] < 0 or n[1] >= len(map):
                continue
            elif map[n[1]][n[0]] == 1:  # 判断障碍
                continue
            elif n in close_list:  # 在close中就跳过
                continue
            elif open_node:
                new_e = e_value_cal(current_node, open_node[1], map)
                if current_node.g_value + dist[abs(neighbour[0]) + abs(neighbour[1])] + open_node[1].h_value + \
                        new_e < open_node[1].f_value:
                    open_node[1].g_value = current_node.g_value + dist[abs(neighbour[0]) + abs(neighbour[1])]
                    open_node[1].parent = current_node  # 更新父节点
                    open_node[1].f_value = current_node.g_value + dist[abs(neighbour[0]) + abs(neighbour[1])] \
                                           + open_node[1].h_value + new_e

            else:
                next_node = Node(position=n, g=current_node.g_value + dist[abs(neighbour[0]) + abs(neighbour[1])]
                                 , h=heuristic_function(final, n), parent=current_node)
                next_node.e_value, next_node.f_value = f_value_cal(next_node, map)
                heapq.heappush(open_list, (next_node.f_value, next_node))  # 加入堆中，必须有Node类__lt__富比较方法才行

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
            print(type(path))
            return path, open_list, close_list


if __name__ == '__main__':
    Map_original = np.zeros((50, 50), dtype=np.int16)
    Map_obstacle = obstacle_process(Map_original)
    star = (1, 20)
    end = (41, 25)
    start_time = time.perf_counter()
    path, ope, clo = astar(star, end, Map_obstacle)
    end_time = time.perf_counter()
    duration = end_time - start_time
    op = []
    for i in ope:
        op.append(i[1])  # 提取出node类
    print(f'程序用时：{duration:.4f}s')
    if not path:
        print('No path available')
    else:
        print(path)
    final_node = next((node for node in op if node.position == end), None)
    print(f'最短距离{final_node.g_value}')
    draw_map(Map_obstacle, star, end, path, op, clo)

