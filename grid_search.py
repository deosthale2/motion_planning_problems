import numpy as np
import sys

DELTA_NAMES = ['v', '>', '<', '^']
STEPS = [[-1, 0], [0, -1], [0, 1], [1, 0]]

def search(grid, init, goal, cost):
	'''
	Given a 2D grid, an initial position and a goal position, ...
	... find the cost of reaching to the goal from the...
	... initial position.

	Input:
		grid: a 2D grid in the form of a list of lists
			  obstacle locations have value 1; open spaces with 0
		init: initial location [r, c]
		goal: goal location [r, c]
		cost: starting cost of the initial location

	Output:
		loc_cost: number of steps to get to goal + initial cost
		loc: goal if reached
	
	The cost is the number of steps to get to the goal position

	This algorithm works by expanding the grid from the start position...
	... and the expansion always takes place in the direction of...
	... the lowest cost.

	Algorithm maintains a list of expanded grid-cells in an open_list...
	and a list of visited grid-cells in a closed_list

	The robot is allowed to move in 4 directions (up, down, left, right)
	'''
	loc, loc_cost = init, 0
	open_list, cost_list, closed_list = [], [], [loc]

	while loc != goal:
		for step in [[-1, 0], [0, -1], [0, 1], [1, 0]]:
			loc_ = [loc[0] + step[0], loc[1] + step[1]]
			# check for grid boundary
			if any([pos<0 for pos in loc_]) or loc_[0]>len(grid)-1 or loc_[1]>len(grid[0])-1:
				continue
			#check for obstacles
			if grid[loc_[0]][loc_[1]] == 1:
				closed_list += [loc_]
				continue
			# expand into open spaces
			elif loc_ not in closed_list and loc_ not in open_list:
				open_list += [loc_]
				cost_list += [loc_cost + cost]
		# check for no solution
		if not open_list and loc != goal:
			print("No path found")
			return
		# update open_list and closed_list by removing the cell...
		# ... with loswest cost for further expansion
		idx_remove = cost_list.index(min(cost_list))
		loc = open_list[idx_remove]
		loc_cost = min(cost_list)
		closed_list += [loc]

		del open_list[idx_remove]
		del cost_list[idx_remove]

	return [loc_cost, loc]

def search_expand(grid, init, goal, cost):
	'''
	Given a 2D grid, perform a grid expansion that tells at which...
	... step a particular grid cell was expanded thereby giving ...
	... the shortest path to virtually each grid cell.

	All the cells that either have obstacles or could not be...
	... expanded into will have the value of -1.
	
	Input:
		grid: a 2D grid in the form of a list of lists
			  obstacle locations have value 1; open spaces with 0
		init: initial location [r, c]
		goal: goal location [r, c]
		cost: starting cost of the initial location

	Output:
		expand: 2D grid with cell values representing the step at...
				which the cell was expanded

	The cost is the number of steps to get to the goal position

	This algorithm works by expanding the grid from the start position...
	... and the expansion always takes place in the direction of...
	... the lowest cost.

	Algorithm maintains a list of expanded grid-cells in an open_list...
	and a list of visited grid-cells in a closed_list

	The robot is allowed to move in 4 directions (up, down, left, right)
	'''

	# initialize the expansion grid with all values as -1
	# set the initial locatino value to 0
	expand = [[-1 for i in range(len(grid[0]))] for j in range(len(grid))]
	expand[init[0]][init[1]] = 0
	expand_count = 0
	loc, loc_cost = init, 0
	open_list, cost_list, closed_list = [], [], [loc]

	while loc != goal:
		for step in [[-1, 0], [0, -1], [0, 1], [1, 0]]:
			loc_ = [loc[0] + step[0], loc[1] + step[1]]
			# check for grid bounday
			if any([pos<0 for pos in loc_]) or loc_[0]>len(grid)-1 or loc_[1]>len(grid[0])-1:
				continue
			# check for obstacles
			if grid[loc_[0]][loc_[1]] == 1:
				closed_list += [loc_]
				continue
			# expand into open spaces
			elif loc_ not in closed_list and loc_ not in open_list:
				open_list += [loc_]
				cost_list += [loc_cost + cost]
		# check for no solution
		if not open_list and loc != goal:
			return expand
		# update open_list and closed_list by removing the cell...
		# ... with loswest cost for further expansion
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
	'''
	Given a 2D grid, an initial position and a goal position, ...
	... find the shortest path to reach the goal from the...
	... initial position.

	The path also indicates the maneuver that had to be done.

	[^, v, <, >] indicate [up, down, left, right] maneuvers.
	'''
	loc, loc_cost = init, 0
	open_list, cost_list, closed_list = [], [], [loc]
	cost_nodes = [[init]]
	min_cost = 0

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
			# expand into open spaces
			elif loc_ not in closed_list and loc_ not in open_list:
				open_list += [loc_]
				cost_list += [loc_cost + cost]
		# check for no solution
		if not open_list and loc != goal:
			print("No path found")
			return
		idx_remove = cost_list.index(min(cost_list))
		loc = open_list[idx_remove]
		loc_cost = min(cost_list)
		closed_list += [loc]
		### up to here same as search_expand

		# avoid having cells with the same cost twice in the path
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

	# print(search(grid, init, goal, cost))
	# print(np.array(search_expand(grid, init, goal, cost)))
	print(search_path(grid, init, goal, cost))
