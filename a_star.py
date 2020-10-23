import numpy as np


def search_Astar(grid, init, goal, cost, heuristic):
	'''
	Given a 2D grid with obstacles, initial position and a...
	... goal position, find the shortest path to goal using...
	... the A* algorithm

	Input:
		grid: a 2D grid in the form of a list of lists
			  obstacle locations have value 1; open spaces with 0
		init: initial location [r, c]
		goal: goal location [r, c]
		cost: starting cost of the initial location

	Output:
		expand: 2D grid with cell values representing the step at...
				which the cell was expanded

	The A* algorithm works very similar to the grid expansion...
	... algorithm. The difference lies in choosing the cell to be...
	... expanded. In grid expansion, the cell with lowest cost...
	... of reaching is expanded; whereas, in A* the cell with the...
	... lowest sum of cost-to-reach and cost-to-goal is expanded.

	The cost-to-goal is given by the heuristic matrix. The...
	... heuristic matrix is the optimistic estimate of the...
	... cost-to-goal for each cell in the grid.
	'''
	expand = [[-1 for i in range(len(grid[0]))] for j in range(len(grid))]
	expand[init[0]][init[1]] = 0
	expand_count = 0
	loc, loc_cost = init, 0
	open_list, cost_list, closed_list = [], [], [loc]
	# create a list to store the sum of 'cost-to-reach' and 'cost-to-goal'
	f_values = []

	while loc != goal:
		for step in [[-1, 0], [0, -1], [0, 1], [1, 0]]:
			loc_ = [loc[0] + step[0], loc[1] + step[1]]
			# check for grid boundary
			if any([pos<0 for pos in loc_]) or loc_[0]>len(grid)-1 or loc_[1]>len(grid[0])-1:
				continue
			# check for obstacles
			if grid[loc_[0]][loc_[1]] == 1:
				closed_list += [loc_]
				continue
			# expand into open space
			elif loc_ not in closed_list and loc_ not in open_list:
				open_list += [loc_]
				cost_list += [loc_cost + cost]
				# compute the sum of cost-to-reach and cost-to-goal
				f_values += [loc_cost + cost + heuristic[loc_[0]][loc_[1]]]
		# check for no solution
		if not open_list and loc != goal:
			return expand
		# select the cell with lowest sum of cost-to-reach and cost-to-goal
		idx_remove = f_values.index(min(f_values))
		loc = open_list[idx_remove]
		loc_cost = cost_list[idx_remove]
		closed_list += [loc]
		# assign expansion index
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

	print(search_Astar(grid, init, goal, cost, heuristic))