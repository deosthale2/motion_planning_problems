import numpy as np

# ----------
# User Instructions:
# 
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's 
# optimal path to the position specified in goal; 
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]

cost = [2, 1, 20] # cost has 3 values, corresponding to making 
                  # a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------


def optimum_policy2D(grid,init,goal,cost):
	value = [[[999 for i in row] for row in grid], 
			 [[999 for i in row] for row in grid], 
			 [[999 for i in row] for row in grid], 
			 [[999 for i in row] for row in grid]]

	policy = [[[' ' for i in row] for row in grid], 
			 [[' ' for i in row] for row in grid], 
			 [[' ' for i in row] for row in grid], 
			 [[' ' for i in row] for row in grid]]

	checked = [[[0 for i in row] for row in grid], 
			 [[0 for i in row] for row in grid], 
			 [[0 for i in row] for row in grid], 
			 [[0 for i in row] for row in grid]]

	x = goal[0]
	y = goal[1]
	g = 0
	open = [[g, x, y]]

	for orientation in range(4):
		policy[orientation][x][y] = '*'
		checked[orientation][x][y] = 1
		value[orientation][x][y] = 0

	to_stop = False
	while to_stop is False:
		if len(open) == 0:
			to_stop = True
		else:
			# print(open)
			open.sort()
			open.reverse()
			next = open.pop()
			x = next[1]
			y = next[2]
			g = next[0]

			for orientation in range(4):
				for i in range(len(action)):
					o2 = (orientation - action[i])%4
					x2 = x - forward[orientation][0]
					y2 = y - forward[orientation][1]
					if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
						if checked[o2][x2][y2] == 0 and grid[x2][y2] == 0:
							g2 = value[orientation][x][y] + cost[i]
							if [x, y] == [2, 3] or [x2, y2] == [3, 3] or [x2, y2] == [1, 3]:
								print([x2, y2], forward_name[o2], action_name[i], [x, y], forward_name[orientation], 'cost: ', g2)
								raw_input('Enter')
							open.append([g2, x2, y2])
							if g2 < value[o2][x2][y2]:
								checked[o2][x2][y2] = 1
								value[o2][x2][y2] = g2
								policy[o2][x2][y2] = action_name[i]
							# else:
							# 	open.append([g2, x, y])
						elif grid[x2][y2] == 1:
							value[o2][x2][y2] = 999

	print(np.array(checked))

	for orientation in range(4):
		for i, row in enumerate(checked[orientation]):
			for j, val in enumerate(row):
				if val == 0 and [i, j] != goal:
					value[orientation][i][j] = 999

	print(np.array(policy))

	policy2D = [[' ' for i in row] for row in grid]
	# x = init[0]
	# y = init[1]
	# policy_idx = init[2]
	# orientation = policy_idx

	# act = policy[policy_idx][x][y]
	# action_idx_hist = action_name.index(act)
	# while [x, y] != goal:
	# 	act = policy[policy_idx][x][y]
	# 	action_idx = action_name.index(act)

	# 	print(policy_idx, orientation, [x, y], act, action_idx)
	# 	raw_input('Enter')

	# 	if action_idx != action_idx_hist:
	# 		policy_idx = (policy_idx + action[action_idx])%4
	# 		act = policy[policy_idx][x][y]
	# 		action_idx = action_name.index(act)
	# 		action_idx_hist = action_idx
	# 		print(policy_idx, orientation, [x, y], act, action_idx)
	# 		continue
	# 	action_idx_hist = action_idx
	# 	policy2D[x][y] = act
	# 	orientation = (orientation + action[action_idx])%4
	# 	x = x + forward[orientation][0]
	# 	y = y + forward[orientation][1]
	# 	print(orientation)

	return np.array(value), np.array(policy), np.array(policy2D)

if __name__ == '__main__':
	value, policy, policy2D = optimum_policy2D(grid, init, goal, cost)
	# print(value)
	# print(policy)
	# print(policy2D)