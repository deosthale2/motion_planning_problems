import numpy as np
import sys

DELTA_NAMES = ['v', '>', '<', '^']
STEPS = [[-1, 0], [0, -1], [0, 1], [1, 0]]

def search(grid, init, goal, cost):
	loc, loc_cost = init, 0
	open_list, cost_list, closed_list = [], [], [loc]

	while loc != goal:
		for step in [[-1, 0], [0, -1], [0, 1], [1, 0]]:
			loc_ = [loc[0] + step[0], loc[1] + step[1]]
			if any([pos<0 for pos in loc_]) or loc_[0]>len(grid)-1 or loc_[1]>len(grid[0])-1:
				continue
			if grid[loc_[0]][loc_[1]] == 1:
				closed_list += [loc_]
				continue
			elif loc_ not in closed_list and loc_ not in open_list:
				open_list += [loc_]
				cost_list += [loc_cost + cost]

		if not open_list and loc != goal:
			print("No path found")
			return
		idx_remove = cost_list.index(min(cost_list))
		loc = open_list[idx_remove]
		loc_cost = min(cost_list)
		closed_list += [loc]

		del open_list[idx_remove]
		del cost_list[idx_remove]

	return [loc_cost, loc]


def search_expand(grid, init, goal, cost):
	expand = [[-1 for i in range(len(grid[0]))] for j in range(len(grid))]
	expand[init[0]][init[1]] = 0
	expand_count = 0
	loc, loc_cost = init, 0
	open_list, cost_list, closed_list = [], [], [loc]

	while loc != goal:
		for step in [[-1, 0], [0, -1], [0, 1], [1, 0]]:
			loc_ = [loc[0] + step[0], loc[1] + step[1]]
			if any([pos<0 for pos in loc_]) or loc_[0]>len(grid)-1 or loc_[1]>len(grid[0])-1:
				continue
			if grid[loc_[0]][loc_[1]] == 1:
				closed_list += [loc_]
				continue
			elif loc_ not in closed_list and loc_ not in open_list:
				open_list += [loc_]
				cost_list += [loc_cost + cost]

		if not open_list and loc != goal:
			return expand

		idx_remove = cost_list.index(min(cost_list))
		loc = open_list[idx_remove]
		loc_cost = min(cost_list)
		closed_list += [loc]
		expand_count += 1
		expand[loc[0]][loc[1]] = expand_count

		del open_list[idx_remove]
		del cost_list[idx_remove]

	return expand


def search_path(grid, init, goal, cost):
	loc, loc_cost = init, 0
	open_list, cost_list, closed_list = [], [], [loc]
	cost_nodes = [[init]]
	min_cost = 0

	while loc != goal:
		for step in [[-1, 0], [0, -1], [0, 1], [1, 0]]:
			loc_ = [loc[0] + step[0], loc[1] + step[1]]
			if any([pos<0 for pos in loc_]) or loc_[0]>len(grid)-1 or loc_[1]>len(grid[0])-1:
				continue
			if grid[loc_[0]][loc_[1]] == 1:
				closed_list += [loc_]
				continue
			elif loc_ not in closed_list and loc_ not in open_list:
				open_list += [loc_]
				cost_list += [loc_cost + cost]

		if not open_list and loc != goal:
			print("No path found")
			return
		idx_remove = cost_list.index(min(cost_list))
		loc = open_list[idx_remove]
		loc_cost = min(cost_list)
		closed_list += [loc]

		if loc_cost > min_cost:
			cost_nodes += [[loc]]
			min_cost = loc_cost
		else:
			cost_nodes[min_cost] += [loc]

		del open_list[idx_remove]
		del cost_list[idx_remove]

	path = [goal]
	path_grid = [[' ' for i in row] for row in grid]

	for i in range(len(cost_nodes)-2, -1, -1):
		for j in cost_nodes[i]:
			step = [j[0] - path[-1][0], j[1] - path[-1][1]]
			# print(step)
			if step in STEPS:
				idx = STEPS.index(step)
				path_grid[j[0]][j[1]] = DELTA_NAMES[idx]
				path += [j]
				continue
	path_grid[goal[0]][goal[1]] = '*'
	path.reverse()
	print(path)
	return np.array(path_grid)


if __name__ == '__main__':
	grid = [[0, 0, 1, 0, 0, 0],
	        [0, 0, 0, 0, 0, 0],
	        [0, 0, 1, 0, 1, 0],
	        [0, 0, 1, 0, 1, 0],
	        [0, 0, 1, 0, 1, 0]]
	init = [0, 0]
	goal = [len(grid)-1, len(grid[0])-1]
	# goal = [2, 2]
	cost = 1

	print(search(grid, init, goal, cost))
	# print(np.array(search_expand(grid, init, goal, cost)))
	# print(search_path(grid, init, goal, cost))
