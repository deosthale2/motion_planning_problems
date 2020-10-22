# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

import numpy as np
import time

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def compute_value(grid, goal, cost):
	value = [[0 for val in row] for row in grid]
	checked = [[0 for val in row] for row in grid]
	to_stop = False
	x = goal[0]
	y = goal[1]
	g = 0
	open = [[g, x, y]]
	checked[x][y] = 1

	while to_stop is False:
		if sum(sum(row) for row in checked) == len(checked)*len(checked[0]) or len(open) == 0:
			to_stop = True
		else:
			open.sort()
			open.reverse()
			next = open.pop()
			x = next[1]
			y = next[2]
			g = next[0]
			for i in range(len(delta)):
			    x2 = x + delta[i][0]
			    y2 = y + delta[i][1]
			    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
			        if checked[x2][y2] == 0 and grid[x2][y2] == 0:
			            g2 = g + cost
			            open.append([g2, x2, y2])
			            checked[x2][y2] = 1
			            value[x2][y2] = g2
			        elif grid[x2][y2] == 1:
		           		checked[x2][y2] = 1
		           		value[x2][y2] = 99
	
	for i, row in enumerate(checked):
		for j, val in enumerate(row):
			if val == 0 and [i, j] != goal:
				value[i][j] = 99

	return(np.array(value))


def optimal_policy(grid, goal, cost):
	value = [[0 for val in row] for row in grid]
	checked = [[0 for val in row] for row in grid]
	policy = [[' ' for val in row] for row in grid]

	to_stop = False
	
	x = goal[0]
	y = goal[1]
	g = 0
	open = [[g, x, y]]
	
	checked[x][y] = 1
	policy[x][y] = '*'

	count = 1
	while to_stop is False:
		count += 1
		if sum(sum(row) for row in checked) == len(checked)*len(checked[0]) or len(open) == 0:
			to_stop = True
		else:
			open.sort()
			open.reverse()
			next = open.pop()
			x = next[1]
			y = next[2]
			g = next[0]
			for i in range(len(delta)):
			    x2 = x + delta[i][0]
			    y2 = y + delta[i][1]
			    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
			        if checked[x2][y2] == 0 and grid[x2][y2] == 0:
			            g2 = g + cost
			            open.append([g2, x2, y2])
			            checked[x2][y2] = 1
			            value[x2][y2] = g2
			            step_idx = delta.index([-1*delta[i][0], -1*delta[i][1]])
			            policy[x2][y2] = delta_name[step_idx]
			        elif grid[x2][y2] == 1:
		           		checked[x2][y2] = 1
		           		value[x2][y2] = 99
	
	for i, row in enumerate(checked):
		for j, val in enumerate(row):
			if val == 0 and [i, j] != goal:
				value[i][j] = 99

	# print('while Count: ', count)
	return np.array(policy)


if __name__ == '__main__':
	# grid = [[0, 1, 0, 0, 0, 0],
 #        	[0, 1, 1, 0, 1, 0],
 #        	[0, 0, 0, 0, 1, 0],
 #        	[0, 1, 1, 1, 1, 0],
 #        	[0, 1, 0, 1, 1, 0]]

	grid = [[0, 1, 0, 0, 0, 0],
	    	[0, 1, 1, 0, 0, 0],
	    	[0, 0, 0, 0, 0, 0],
	    	[0, 1, 1, 1, 0, 0],
	    	[0, 1, 0, 1, 0, 0]]

	goal = [len(grid)-1, len(grid[0])-1]
	cost = 1 # the cost associated with moving from a cell to an adjacent one

	print(compute_value(grid, goal, cost))
	
	start = time.time()
	for i in range(100):
		optimal_policy(grid, goal, cost)
		# print(optimal_policy(grid, goal, cost))
	print(time.time() - start)