from collections import namedtuple
import numpy as np
import matplotlib.pyplot as plt

WHEELBASE = 0.5
SPEED = 1.1
NUM_THETA_CELLS = 20

PseudoState = namedtuple('PseudoState',['x_idx','y_idx','stack_num'])

def get_heuristic(init, goal):
	return np.abs(goal[0]-init[0]) + np.abs(goal[1]-init[1])

def motion_model(init_state, v, delta_t, steer_angle):
	[x, y, theta] = init_state

	theta_ = (theta + (v/WHEELBASE)*np.tan(steer_angle)*delta_t)%(2*np.pi)
	x_ = x + v*np.cos(theta_)*delta_t
	y_ = y + v*np.sin(theta_)*delta_t
	return [x_, y_, theta_]

def expand_states(init_state, v, delta_t):
	future_states = []
	# [-np.deg2rad(35.0), 0., np.deg2rad(35.0)]
	for steer_angle in np.linspace(-np.deg2rad(35.0), np.deg2rad(35.0), 10):
		future_states += [motion_model(init_state, v, delta_t, steer_angle)]
	return future_states

def get_index(val):
	return int(np.floor(val))

def get_stack_num(theta):
	return int(np.floor(NUM_THETA_CELLS*theta/(2*np.pi)))%NUM_THETA_CELLS

def search_hybrid_a_star(grid, init, goal, cost):
	loc_state_ = PseudoState(get_index(init[0]), get_index(init[1]), get_stack_num(init[2]))
	loc_state = init
	loc, loc_cost = [loc_state_.x_idx, loc_state_.y_idx], 0
	open_list, cost_list = [], []

	closed_list = [[[0 for i in range(len(grid[0]))] for j in range(len(grid))] for k in range(NUM_THETA_CELLS)]
	closed_list[loc_state_.stack_num][loc_state_.x_idx][loc_state_.y_idx] = 0

	came_from = []
	valid_expand = 0

	# create a list to store the sum of 'cost-to-reach' and 'cost-to-goal'
	f_values = []

	while loc != goal:
		# print(loc)
		next_states = expand_states(loc_state, SPEED, 1.0)
		# print(next_states)
		for next_s in next_states:
			loc_state_ = PseudoState(get_index(next_s[0]), get_index(next_s[1]), get_stack_num(next_s[2]))
			# print(next_s)
			# print([loc_state_.stack_num, loc_state_.x_idx, loc_state_.y_idx])

			# check for grid boundary
			# if any([pos<0 for pos in (next_s[0], next_s[1])]) or next_s[0]>len(grid)-1 or next_s[1]>len(grid[0])-1:
			# 	continue

			if any([pos<0 for pos in (loc_state_.x_idx, loc_state_.y_idx)]) or loc_state_.x_idx>len(grid)-1 or loc_state_.y_idx>len(grid[0])-1:
				continue

			# check for obstacles
			if grid[loc_state_.x_idx][loc_state_.y_idx] == 1:
				closed_list[loc_state_.stack_num][loc_state_.x_idx][loc_state_.y_idx] = 1

			# expand into open space
			elif closed_list[loc_state_.stack_num][loc_state_.x_idx][loc_state_.y_idx] == 0 and next_s not in open_list:
				valid_expand = 1
				closed_list[loc_state_.stack_num][loc_state_.x_idx][loc_state_.y_idx] = 1
				open_list += [next_s]
				cost_list += [loc_cost + cost]
				# compute the sum of cost-to-reach and cost-to-goal
				# f_values += [loc_cost + cost + get_heuristic([loc_state_.x_idx, loc_state_.y_idx], goal)] # Use grid index for cost
				f_values += [loc_cost + cost + get_heuristic([next_s[0], next_s[1]], goal)] # use exact location

		# check if at least one exapnsion is valid from previous state
		if valid_expand == 1:
			came_from += [loc_state]
			valid_expand == 0

		# check for no solution
		if not open_list and loc != goal:
			return came_from
		# select the cell with lowest sum of cost-to-reach and cost-to-goal
		idx_remove = f_values.index(min(f_values))
		loc_state = open_list[idx_remove]
		loc_state_cost = cost_list[idx_remove]
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

	# grid = [[0, 1, 0, 0, 0, 0],
	#         [0, 1, 0, 0, 0, 0],
	#         [0, 0, 0, 0, 1, 0],
	#         [0, 0, 0, 0, 1, 0]]

	grid = [[0, 1, 0, 0],
	        [0, 1, 0, 0],
	        [0, 0, 0, 0],
	        [0, 0, 1, 0]]

	init = [0, 0, 0.]
	goal = [len(grid)-1, len(grid[0])-1]
	# goal = [3, 5]
	cost = 1

	heuristic = [[0 for i in range(len(grid[0]))] for j in range(len(grid))]
	for i in range(len(grid)):
		for j in range(len(grid[0])):
			heuristic[i][j] = get_heuristic([i, j], goal)
	print(np.array(heuristic))

	path = search_hybrid_a_star(grid, init, goal, cost)
	for node in path:
		print(node)

	plot_path(path)


if __name__ == '__main__':
	main()