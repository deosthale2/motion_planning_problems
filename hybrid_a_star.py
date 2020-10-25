from collections import namedtuple
import numpy as np
import matplotlib.pyplot as plt

WHEELBASE = 0.5
SPEED = 0.5
NUM_THETA_CELLS = 20

PseudoState = namedtuple('PseudoState',['x_idx','y_idx','stack_num'])

def get_heuristic(grid, init, goal, cost):
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
		expand[loc[0]][loc[1]] = loc_cost #expand_count

		del open_list[idx_remove]
		del cost_list[idx_remove]

	return expand

def motion_model(init_state, v, delta_t, steer_angle):
	[x, y, theta] = init_state

	theta_ = (theta + (v/WHEELBASE)*np.tan(steer_angle)*delta_t)%(2*np.pi)
	x_ = x + v*np.cos(theta_)*delta_t
	y_ = y + v*np.sin(theta_)*delta_t
	return [x_, y_, theta_]

def expand_states(init_state, v, delta_t):
	future_states = []
	# [-np.deg2rad(35.0), 0., np.deg2rad(35.0)]
	for steer_angle in np.linspace(-np.deg2rad(35.0), np.deg2rad(35.0), 20):
		future_states += [motion_model(init_state, v, delta_t, steer_angle)]
	return future_states

def get_index(val):
	return int(np.floor(val))

def get_stack_num(theta):
	return int(np.floor(NUM_THETA_CELLS*theta/(2*np.pi)))%NUM_THETA_CELLS

def search_hybrid_a_star(grid, init, goal, cost, heuristic):
	loc_state_ = PseudoState(get_index(init[0]), get_index(init[1]), get_stack_num(init[2]))
	loc_state = init
	loc, loc_cost = [loc_state_.x_idx, loc_state_.y_idx], 0
	open_list, cost_list = [], []

	closed_list = [[[0 for i in range(len(grid[0]))] for j in range(len(grid))] for k in range(NUM_THETA_CELLS)]
	closed_list[loc_state_.stack_num][loc_state_.x_idx][loc_state_.y_idx] = 0

	came_from = [init]

	# create a list to store the sum of 'cost-to-reach' and 'cost-to-goal'
	f_values = []

	while loc != goal:
		print(loc)
		next_states = expand_states(loc_state, SPEED, 1.0)
		# print(next_states)
		for next_s in next_states:
			loc_state_ = PseudoState(get_index(next_s[0]), get_index(next_s[1]), get_stack_num(next_s[2]))
			# print(next_s)
			# print([loc_state_.stack_num, loc_state_.x_idx, loc_state_.y_idx])

			# check for grid boundary
			if any([pos<0 for pos in (loc_state_.x_idx, loc_state_.y_idx)]) or loc_state_.x_idx>len(grid)-1 or loc_state_.y_idx>len(grid[0])-1:
				continue

			# check for obstacles
			if grid[loc_state_.x_idx][loc_state_.y_idx] == 1:
				closed_list[loc_state_.stack_num][loc_state_.x_idx][loc_state_.y_idx] = 1

			# expand into open space
			elif closed_list[loc_state_.stack_num][loc_state_.x_idx][loc_state_.y_idx] == 0 and next_s not in open_list:
				closed_list[loc_state_.stack_num][loc_state_.x_idx][loc_state_.y_idx] = 1
				open_list += [next_s]
				cost_list += [loc_cost + cost]
				# compute the sum of cost-to-reach and cost-to-goal
				f_values += [loc_cost + cost + heuristic[loc_state_.x_idx][loc_state_.y_idx]]

		# check for no solution
		if not open_list and loc != goal:
			return came_from
		# select the cell with lowest sum of cost-to-reach and cost-to-goal
		idx_remove = f_values.index(min(f_values))
		loc_state = open_list[idx_remove]
		loc_state_cost = cost_list[idx_remove]
		# loc_state_ = PseudoState(get_index(loc_state[0]), get_index(loc_state[1]), get_stack_num(loc_state[2]))
		# closed_list[loc_state_.x_idx][loc_state_.x_idx][loc_state_.y_idx] = 1
		came_from += [loc_state]
		loc = [get_index(loc_state[0]), get_index(loc_state[1])]

		del open_list[idx_remove]
		del cost_list[idx_remove]
		del f_values[idx_remove]

	return came_from

def plot_path(states):
	x, y = [], []

	for state in states:
		x += [state[0]]
		y += [state[1]]

	plt.figure()
	plt.plot(y, x, '-')
	plt.show()

def main():
	# grid = [[0,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,],
	# 		[0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,],
	# 		[0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,],
	# 		[0,1,1,0,0,0,0,1,1,0,0,0,1,1,1,0,],
	# 		[0,1,1,0,0,0,1,1,0,0,0,1,1,1,0,0,],
	# 		[0,1,1,0,0,1,1,0,0,0,1,1,1,0,0,0,],
	# 		[0,1,1,0,1,1,0,0,0,1,1,1,0,0,0,0,],
	# 		[0,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,],
	# 		[0,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,],
	# 		[0,1,1,0,0,0,1,1,1,0,0,1,1,1,1,1,],
	# 		[0,1,0,0,0,1,1,1,0,0,1,1,1,1,1,1,],
	# 		[0,0,0,0,1,1,1,0,0,1,1,1,1,1,1,1,],
	# 		[0,0,0,1,1,1,0,0,1,1,1,1,1,1,1,1,],
	# 		[0,0,1,1,1,0,0,1,1,1,1,1,1,1,1,1,],
	# 		[0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,],
	# 		[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,]]

	grid = [[0, 0, 1, 0, 1, 0],
	        [0, 0, 0, 0, 0, 0],
	        [0, 0, 0, 0, 0, 0],
	        [0, 0, 0, 0, 1, 0],
	        [0, 0, 1, 0, 1, 0]]

	empty_grid = [[0 for i in range(len(grid[0]))] for j in range(len(grid))]

	init = [0, 0, 0.]
	# goal = [len(grid)-1, len(grid[0])-1]
	goal = [3, 0]
	cost = 1

	heuristic = get_heuristic(empty_grid, init, goal, cost)
	# print(np.array(heuristic))

	path = search_hybrid_a_star(grid, init, goal, cost, heuristic)
	plot_path(path)


if __name__ == '__main__':
	main()