import math
import random

import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString

import Classes


def CheckIfValidConnection(point, node_to_connect):
    is_valid = True
    for obj in obstacles:
        for j in range(len(obj.coord_list) - 1):
            line_a = LineString([point, (node_to_connect.x_coord, node_to_connect.y_coord)])
            line_b = LineString(
                [(obj.coord_list[j][0], obj.coord_list[j][1]), (obj.coord_list[j + 1][0], obj.coord_list[j + 1][1])])
            if line_a.intersects(line_b):
                is_valid = False
                break

    return is_valid


def CheckIfPointInObstacle(point):
    is_valid = True
    for obj in obstacles:
        if obj.polygon.contains(Point(point[0], point[1])):
            is_valid = False
            break

    return is_valid


def SwitchPositionsInTree(index1, index2):
    node_index2 = nodes_in_tree[index2]

    for index in nodes_in_tree[index1].neighbour_indexes:
        i = 0
        for replace_index in nodes_in_tree[index].neighbour_indexes:
            if replace_index == index1:
                replace_index = index2
            i += 1

    for index in nodes_in_tree[index2].neighbour_indexes:
        for replace_index in nodes_in_tree[index].neighbour_indexes:
            if replace_index == index2:
                replace_index = index1

    nodes_in_tree[index2] = nodes_in_tree[index1]
    nodes_in_tree[index1] = node_index2

    # nodes_in_tree[index2].x_coord = nodes_in_tree[index1].x_coord
    # nodes_in_tree[index2].y_coord = nodes_in_tree[index1].y_coord
    # nodes_in_tree[index2].cost = nodes_in_tree[index1].cost
    # nodes_in_tree[index2].parent_index = nodes_in_tree[index1].parent_index
    # nodes_in_tree[index2].neighbour_indexes = nodes_in_tree[index1].neighbour_indexes

    # nodes_in_tree[index1].x_coord = node_index2.x_coord
    # nodes_in_tree[index1].y_coord = node_index2.y_coord
    # nodes_in_tree[index1].cost = node_index2.cost
    # nodes_in_tree[index1].parent_index = node_index2.parent_index
    # nodes_in_tree[index1].neighbour_indexes = node_index2.neighbour_indexes

    nodes_in_tree[index1].self_index = index1
    nodes_in_tree[index2].self_index = index2


def Visualizing_graph():
    # Visualizing graph
    # Drawing nodes
    for node in nodes_in_tree:
        for neighbour_index in node.neighbour_indexes:
            # circle = plt.Circle((node.x_cood, node.y_coord), fixed_distance, color = 'b', fill = False)
            # ax.add_patch(circle)
            plt.plot([node.x_coord, nodes_in_tree[neighbour_index].x_coord],
                     [node.y_coord, nodes_in_tree[neighbour_index].y_coord], "r.-", markersize=3, linewidth=0.3)

    # draw obstacles
    for obj in obstacles:
        xs, ys = zip(*obj.coord_list)
        plt.plot(xs, ys, "c-")
    # Connecting Goal to the start point
    curr_index = len(nodes_in_tree) - 1
    while curr_index != 0:
        parent_index = nodes_in_tree[curr_index].parent_index
        plt.plot([nodes_in_tree[curr_index].x_coord, nodes_in_tree[parent_index].x_coord],
                 [nodes_in_tree[curr_index].y_coord, nodes_in_tree[parent_index].y_coord], 'b.-', markersize=5,
                 linewidth=0.5)
        curr_index = parent_index

    # draw goal
    # xs, ys = zip(*goal.coord_list)
    # plt.plot(xs, ys)

    plt.show()


def Main_loop_of_PRM():
    for i in range(total_nodes):
        rand_point = (random.uniform(0, 100), random.uniform(0, 100))

        if not CheckIfPointInObstacle(rand_point):
            continue

        nodes_in_tree.append(Classes.Nodes(rand_point[0], rand_point[1], [], 0, 0, len(nodes_in_tree)))

        for i in range(len(nodes_in_tree) - 1):
            dist = math.sqrt(
                (nodes_in_tree[i].x_coord - rand_point[0]) ** 2 + (nodes_in_tree[i].y_coord - rand_point[1]) ** 2)

            if dist <= fixed_distance and CheckIfValidConnection(rand_point, nodes_in_tree[i]):
                nodes_in_tree[i].neighbour_indexes.append(len(nodes_in_tree) - 1)
                nodes_in_tree[len(nodes_in_tree) - 1].neighbour_indexes.append(i)


def DijkstraAlgorithm():
    ranking_of_nodes = []
    for node in nodes_in_tree:
        node.cost = 9999999
    nodes_in_tree[0].cost = 0
    for index in nodes_in_tree[0].neighbour_indexes:
        nodes_in_tree[index].cost = math.sqrt((nodes_in_tree[index].x_coord - nodes_in_tree[0].x_coord) ** 2 + (
                nodes_in_tree[index].y_coord - nodes_in_tree[0].y_coord) ** 2) + math.sqrt(
            (nodes_in_tree[index].x_coord - goal[0]) ** 2 + (nodes_in_tree[index].y_coord - goal[1]) ** 2)
        nodes_in_tree[index].parent_index = 0

    for node in nodes_in_tree:
        ranking_of_nodes.append([node.self_index, node.cost])
    ranking_of_nodes.sort(key=lambda x: x[1])

    goal_reached = False
    while len(ranking_of_nodes) > 0 and (not goal_reached):
        if ranking_of_nodes[0][0] == len(nodes_in_tree) - 1:
            goal_reached = True

        for index in nodes_in_tree[ranking_of_nodes[0][0]].neighbour_indexes:
            new_cost = nodes_in_tree[ranking_of_nodes[0][0]].cost + math.sqrt(
                (nodes_in_tree[index].x_coord - nodes_in_tree[ranking_of_nodes[0][0]].x_coord) ** 2 + (
                        nodes_in_tree[index].y_coord - nodes_in_tree[ranking_of_nodes[0][0]].y_coord) ** 2) + math.sqrt(
                (nodes_in_tree[index].x_coord - goal[0]) ** 2 + (nodes_in_tree[index].y_coord - goal[1]) ** 2)
            if new_cost < nodes_in_tree[index].cost:
                nodes_in_tree[index].cost = new_cost
                nodes_in_tree[index].parent_index = ranking_of_nodes[0][0]

        ranking_of_nodes.pop(0)
        for ranking in ranking_of_nodes:
            ranking[1] = nodes_in_tree[ranking[0]].cost
        ranking_of_nodes.sort(key=lambda x: x[1])

    print(len(ranking_of_nodes))


def Sort():
    for i in range(len(nodes_in_tree)):
        if i != 0:
            j = i
            while j > 0 and nodes_in_tree[j].cost < nodes_in_tree[j - 1].cost:
                SwitchPositionsInTree(j, j - 1)
                j -= 1


def Adding_Goal_to_Tree():
    nodes_in_tree.append(Classes.Nodes(goal[0], goal[1], [], 0, 0, len(nodes_in_tree)))

    for i in range(len(nodes_in_tree) - 1):
        dist = math.sqrt((nodes_in_tree[i].x_coord - goal[0]) ** 2 + (nodes_in_tree[i].y_coord - goal[1]) ** 2)

        if dist <= fixed_distance and CheckIfValidConnection(goal, nodes_in_tree[i]):
            nodes_in_tree[i].neighbour_indexes.append(len(nodes_in_tree) - 1)
            nodes_in_tree[len(nodes_in_tree) - 1].neighbour_indexes.append(i)

    print("Tree plotted.")


point_of_origin = (50, 50)  # start point
bounds_of_plane = (100, 100)
fixed_distance = 5
total_nodes = 2000

counter = 0
found_goal = False

obstacles = [Classes.Figures([[10, 70], [20, 80], [40, 80], [40, 60], [20, 60], [10, 70]]),
             Classes.Figures([[60, 80], [65, 80], [65, 20], [60, 20], [60, 80]]),
             Classes.Figures([[20, 30], [30, 10], [10, 10], [20, 30]])]

goal = (96, 50)

nodes_in_tree = [Classes.Nodes(point_of_origin[0], point_of_origin[1], [], 0, 0, 0)]
