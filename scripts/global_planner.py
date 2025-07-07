#!/usr/bin/env python3
import heapq
import numpy as np

def get_neighbors(pos, grid):
    directions = [(-1,0), (1,0), (0,-1), (0,1)]  # 4-connected
    neighbors = []
    for dx, dy in directions:
        nx, ny = pos[0] + dx, pos[1] + dy
        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
            if grid[nx, ny] == 0:
                neighbors.append((nx, ny))
    return neighbors

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def a_star(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    cost = {start: 0}

    while open_set:
        _, g, current = heapq.heappop(open_set)
        if current == goal:
            break
        for neighbor in get_neighbors(current, grid):
            new_cost = g + 1
            if neighbor not in cost or new_cost < cost[neighbor]:
                cost[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(open_set, (priority, new_cost, neighbor))
                came_from[neighbor] = current

    path = []
    if goal in came_from:
        node = goal
        while node != start:
            path.append(node)
            node = came_from[node]
        path.append(start)
        path.reverse()
    return path
