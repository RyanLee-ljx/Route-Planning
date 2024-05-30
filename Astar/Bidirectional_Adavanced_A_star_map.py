# 本代码为改进Astar算法
import heapq
import numpy as np
from matplotlib import pyplot as plt
import time
import math
'''
本代码为双向改进Astar
改进点：
1.数据结构：
    open_list采用堆结构
    close_list采用哈希表
2.启发函数：
    避免对角穿过障碍，将代价函数改为：F(n) = G(n) + H(n) + E(n) ，注意此时节点判断不能通过判断G来判断F
    
3.邻域搜索：
   采用双向交替搜索方式：分别建立两个open_list 和 close_list，一个从起点搜索一个从终点搜索，采用交替搜索，谁的open_list节点少，优先从该
   部分扩展，分别以对方open_list中最低F进行扩展
'''


class Node:
    def __init__(self, **kwargs):
        self.f_value = None
        self.position = kwargs.get('position')  # 用get方法，当没有给出改键值时，自动补为None！！！
        self.g_value = kwargs.get('g')
        self.h_value = kwargs.get('h')
        self.parent = kwargs.get('parent')

    def __lt__(self, other):  # 用于自定义比较两个Node类间的大小，通过f_value
        return self.f_value < other.f_value
        # python富比较方法：自定义进行类比较


def heuristic_function(final, position):
    # 欧氏距离
    # f = math.sqrt((final[0] - position[0]) ** 2 + (final[1] - position[1]) ** 2)
    f = abs(final[0] - position[0]) + abs(final[1] - position[1])
    return f


def f_value_cal(node, map):
    f_value = node.g_value + node.h_value * 1.01  # 百分之0.1的偏移
    return math.ceil(f_value)


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
    # 创建起点
    start_node = Node(position=start, g=0, h=heuristic_function(final, start))   # 向前传播起点，向后传播终点
    start_node.f_value = f_value_cal(start_node, map)
    final_node = Node(position=final, g=0, h=heuristic_function(start, final))  # 向后传播起点，向前传播终点
    start_node.f_value = f_value_cal(final_node, map)
    # 创建openlist 和 closelist
    open_list_forward = []
    open_list_backward = []
    close_list_forward = dict()
    close_list_backward = dict()
    # 把起点加入堆中
    heapq.heappush(open_list_forward, (start_node.f_value, start_node))  # 生成堆，并将节点按照f_value进行堆放，自动形成最小堆，
    heapq.heappush(open_list_backward, (final_node.f_value, final_node))
    # 放进去的是一个tuple(int, Node)类型

    while open_list_backward and open_list_forward:
        current_node_forward_list = heapq.nsmallest(1, open_list_forward, None)   # [（f_value, node）]
        current_node_backward_list = heapq.nsmallest(1, open_list_backward, None)

        if len(open_list_forward) <= len(open_list_backward):
            current_node = current_node_forward_list[0][1]
            open_list_forward.remove(current_node_forward_list[0])
            target_node = current_node_backward_list[0][1]
            # if current_node.g_value <= 50:
            #     target_node = final_node
            # else:
            #     target_node = current_node_backward_list[0][1]
            close_list_forward[current_node.position] = current_node
            open_list = open_list_forward
            close_list = close_list_forward
        else:
            current_node = current_node_backward_list[0][1]
            open_list_backward.remove(current_node_backward_list[0])
            target_node = current_node_backward_list[0][1]
            # if current_node.g_value <= 50:
            #     target_node = start_node
            # else:
            #     target_node = current_node_backward_list[0][1]
            close_list_backward[current_node.position] = current_node
            open_list = open_list_backward
            close_list = close_list_backward

        # 判断终止+路径回溯
        if current_node_forward_list[0][1].position == current_node_backward_list[0][1].position:
            path_forward = [current_node_forward_list[0][1].position]
            par_forward = current_node_forward_list[0][1].parent
            while par_forward:
                path_forward.append(par_forward.position)
                par_forward = par_forward.parent
            path_forward = path_forward[::-1]  # 路径反转
            path_backward = [current_node_backward_list[0][1].position]
            par_backward = current_node_backward_list[0][1].parent
            while par_backward:
                path_backward.append(par_backward.position)
                par_backward = par_backward.parent
            path = path_forward + path_backward  # 拼接路径
            total_open = open_list_forward + open_list_backward
            close_list_forward.update(close_list_backward)
            distance = current_node_forward_list[0][1].g_value + current_node_backward_list[0][1].g_value
            return path, total_open, close_list_forward, distance

        neighbours = [(-1, 0), (0, 1), (0, -1), (1, 0)]
        dist = 1
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
                # new_e = e_value_cal(current_node, open_node[1], map)
                if current_node.g_value + dist + open_node[1].h_value < open_node[1].f_value:
                    open_node[1].g_value = current_node.g_value + dist
                    open_node[1].parent = current_node  # 更新父节点
                    open_node[1].f_value = open_node[1].g_value + open_node[1].h_value 

            else:
                next_node = Node(position=n, g=current_node.g_value + dist, h=heuristic_function(target_node.position, n), parent=current_node)
                next_node.f_value = f_value_cal(next_node, map)
                heapq.heappush(open_list, (next_node.f_value, next_node))  # 加入堆中，必须有Node类__lt__富比较方法才行


if __name__ == '__main__':
    Map_original = np.zeros((96, 39), dtype=np.int16)
    Map_obstacle = obstacle_process(Map_original)
    star = (0, 95) # 前面是x坐标，后面是y坐标，左上角为原点
    end = (36, 9)
    start_time = time.perf_counter()
    path, ope, clo, distance = astar(star, end, Map_obstacle)
    end_time = time.perf_counter()
    duration = end_time - start_time
    print(f'程序用时：{duration:.4f}s')
    op = []
    for i in ope:
        op.append(i[1])  # 提取出node类
    if not path:
        print('No path available')
    else:
        print(path)
    print(f'最短距离{distance}')
    draw_map(Map_obstacle, star, end, path, op, clo)

