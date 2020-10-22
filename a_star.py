import numpy as np


def search(grid, init, goal, cost, heuristic):
	expand = [[-1 for i in range(len(grid[0]))] for j in range(len(grid))]
	expand[init[0]][init[1]] = 0
	expand_count = 0
	loc, loc_cost = init, 0
	open_list, cost_list, closed_list = [], [], [loc]
	f_values = []

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
				f_values += [loc_cost + cost + heuristic[loc_[0]][loc_[1]]]

		if not open_list and loc != goal:
			return expand

		idx_remove = f_values.index(min(f_values))
		loc = open_list[idx_remove]
		loc_cost = cost_list[idx_remove]
		closed_list += [loc]

		expand_count += 1
		expand[loc[0]][loc[1]] = expand_count

		del open_list[idx_remove]
		del cost_list[idx_remove]
		del f_values[idx_remove]

	return np.array(expand)


if __name__ == '__main__':

	heuristic = [[9, 8, 7, 6, 5, 4],
				 [8, 7, 6, 5, 4, 3],
				 [7, 6, 5, 4, 3, 2],
				 [6, 5, 4, 3, 2, 1],
				 [5, 4, 3, 2, 1, 0]]

	grid = [[0, 1, 0, 0, 0, 0],
        	[0, 1, 0, 0, 0, 0],
        	[0, 1, 0, 0, 0, 0],
        	[0, 1, 0, 0, 0, 0],
        	[0, 0, 0, 0, 0, 0]]

	init = [0, 0]
	goal = [len(grid)-1, len(grid[0])-1]
	cost = 1

	print(search(grid, init, goal, cost, heuristic))